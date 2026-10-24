// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  pluginosd.cpp - headless OSD for running a machine inside
//  an audio plugin
//
//============================================================

#include "emu.h"

#include "pluginosd.h"
#include "pluginmidi.h"

#include "interface/midiport.h"

#include "render.h"
#include "rendersw.hxx"
#include "rendlay.h"

#include "strformat.h"

#include <algorithm>
#include <bit>
#include <type_traits>
#include <vector>
#include <cstdio>


namespace {

// the panel renderer writes R, G, B, A bytes
using panel_renderer = std::conditional_t<
		std::endian::native == std::endian::little,
		software_renderer<u32, 0,0,0, 0,8,16>,
		software_renderer<u32, 0,0,0, 24,16,8> >;
constexpr u32 PANEL_ALPHA = (std::endian::native == std::endian::little) ? 0xff000000 : 0x000000ff;

// most frames the panel is rendered per second
constexpr int PANEL_MAX_FPS = 60;


// stereo sink node presented to the sound manager
constexpr uint32_t PLUGIN_NODE_ID = 1;
constexpr uint32_t PLUGIN_STREAM_ID = 1;


//============================================================
//  plugin_midi_input - delivers the host's MIDI bytes once
//  the emulation reaches their due time
//============================================================

class plugin_midi_input : public osd::midi_input_port
{
public:
	plugin_midi_input(running_machine &machine, plugin_io &io) : m_machine(machine), m_io(io) { }

	virtual bool poll() override
	{
		plugin_io::midi_byte const *const next = m_io.midi.front();
		return next && (next->frame <= current_frame());
	}

	virtual int read(uint8_t *out) override
	{
		// midiin_device reads into a 32K buffer
		constexpr int MAX_BYTES = 8192 * 4;

		std::uint64_t const now = current_frame();
		int count = 0;
		for (plugin_io::midi_byte const *next = m_io.midi.front(); next && (next->frame <= now) && (count < MAX_BYTES); next = m_io.midi.front())
		{
			out[count++] = next->data;
			m_io.midi.pop(nullptr, 1);
		}
		return count;
	}

private:
	std::uint64_t current_frame() const { return m_machine.time().as_ticks(m_io.sample_rate); }

	running_machine &m_machine;
	plugin_io &m_io;
};


//============================================================
//  plugin_midi_output - passes the machine's MIDI output to
//  the host as complete messages, timestamped with the frame
//  at which their last byte was sent
//============================================================

class plugin_midi_output : public osd::midi_output_port
{
public:
	plugin_midi_output(running_machine &machine, plugin_io &io) : m_machine(machine), m_io(io) { }

	virtual void write(uint8_t data) override
	{
		m_assembler.write(
				data,
				[this] (uint8_t const *message, std::size_t size)
				{
					std::uint64_t const frame = m_machine.time().as_ticks(m_io.sample_rate);
					if (!plugin_midi_record::push(m_io.midi_out, frame, message, size, m_record))
						m_io.midi_out_dropped.fetch_add(1, std::memory_order_relaxed);
				},
				[this] () { m_io.midi_out_dropped.fetch_add(1, std::memory_order_relaxed); });
	}

private:
	running_machine &m_machine;
	plugin_io &m_io;
	plugin_midi_assembler m_assembler;
	std::vector<uint8_t> m_record;
};

} // anonymous namespace


//============================================================
//  plugin_io
//============================================================

void plugin_io::log(osd_output_channel channel, std::string const &text)
{
	if ((channel != OSD_OUTPUT_CHANNEL_ERROR) && (channel != OSD_OUTPUT_CHANNEL_WARNING))
		return;

	std::lock_guard<std::mutex> lock(log_mutex);
	log_text.append(text);
}


//============================================================
//  plugin_panel
//============================================================

std::uint64_t plugin_panel::draw(void *pixels, std::uint32_t width, std::uint32_t height, std::uint64_t last)
{
	std::unique_lock<std::mutex> lock(mutex);
	std::uint64_t const current = serial;
	if (!list || (current == last) || (list_width != width) || (list_height != height))
		return last;
	render_primitive_list &primitives = *list;
	std::unique_lock<std::mutex> drawing(draw_mutex);
	lock.unlock();

	// the list keeps the textures it uses alive, but the screens keep
	// updating their bitmaps, so a frame drawn late may show parts of a
	// newer one
	u32 *const dest = reinterpret_cast<u32 *>(pixels);
	std::fill_n(dest, std::size_t(width) * height, 0);
	primitives.acquire_lock();
	panel_renderer::draw_primitives(primitives, dest, width, height, width);
	primitives.release_lock();
	for (std::size_t i = 0; i < std::size_t(width) * height; i++)
		dest[i] |= PANEL_ALPHA;

	drawing.unlock();
	lock.lock();
	fetched = current;
	return current;
}

void plugin_panel::detach()
{
	{
		std::lock_guard<std::mutex> lock(mutex);
		list = nullptr;
	}
	std::lock_guard<std::mutex> drawing(draw_mutex);
}


//============================================================
//  plugin_osd_interface
//============================================================

plugin_osd_interface::plugin_osd_interface(osd_options &options, plugin_io &io)
	: osd_common_t(options)
	, m_io(io)
{
}

plugin_osd_interface::~plugin_osd_interface()
{
}

void plugin_osd_interface::init(running_machine &machine)
{
	osd_common_t::init(machine);
	osd_common_t::init_subsystems();

	// there are no windows, but the core expects a UI target; it renders
	// the machine's layout for the plugin GUI
	m_target = machine.render().target_alloc();
	machine.render().set_ui_target(*m_target);

	// natural size: the view's aspect at no less than one pixel per
	// screen pixel
	s32 min_width, min_height;
	m_target->compute_minimum_size(min_width, min_height);
	float const aspect = m_target->current_view().effective_aspect();
	s32 width = s32(min_height * aspect + 0.5f);
	s32 height = min_height;
	if (width < min_width)
	{
		width = min_width;
		height = s32(min_width / aspect + 0.5f);
	}

	std::lock_guard<std::mutex> lock(m_io.panel.mutex);
	m_io.panel.natural_width = std::max(width, 1);
	m_io.panel.natural_height = std::max(height, 1);
}

void plugin_osd_interface::update(bool skip_redraw)
{
	osd_common_t::update(skip_redraw);
	update_panel();
}

void plugin_osd_interface::update_panel()
{
	plugin_panel &panel = m_io.panel;
	u32 width, height;
	bool consumed;
	{
		std::lock_guard<std::mutex> lock(panel.mutex);
		width = panel.width;
		height = panel.height;
		consumed = panel.fetched == panel.serial;
		m_pointer_events.swap(panel.pointers);
		panel.pointers.clear();
	}

	if (!width || !height)
		return;

	bool const resized = (s32(width) != m_target->width()) || (s32(height) != m_target->height());
	if (resized)
		m_target->set_bounds(width, height);

	// clicks on the layout's controls
	if (!m_pointer_events.empty())
	{
		for (plugin_panel::pointer_event const &event : m_pointer_events)
		{
			if (m_pointer_buttons.size() <= event.id)
				m_pointer_buttons.resize(event.id + 1, 0);
			u32 const previous = m_pointer_buttons[event.id];
			if (event.leave)
			{
				m_target->pointer_left(osd::ui_event_handler::pointer::MOUSE, event.id, 0, event.x, event.y, previous, 0);
				m_pointer_buttons[event.id] = 0;
			}
			else
			{
				u32 const pressed = event.buttons & ~previous;
				u32 const released = previous & ~event.buttons;
				m_target->pointer_updated(osd::ui_event_handler::pointer::MOUSE, event.id, 0, event.x, event.y, event.buttons, pressed, released, BIT(pressed, 0) ? 1 : 0);
				m_pointer_buttons[event.id] = event.buttons;
			}
		}
		m_target->update_pointer_fields();
		m_pointer_events.clear();
	}

	// build a new list when the GUI drew the last one, at a limited rate;
	// the target alternates between two lists, so the one rebuilt here is
	// never the one the GUI may still be drawing
	osd_ticks_t const now = osd_ticks();
	if (!resized && (!consumed || ((now - m_panel_last) < (osd_ticks_per_second() / PANEL_MAX_FPS))))
		return;
	m_panel_last = now;

	render_primitive_list &list = m_target->get_primitives();

	std::lock_guard<std::mutex> lock(panel.mutex);
	panel.list = &list;
	panel.list_width = width;
	panel.list_height = height;
	panel.serial++;
}

osd::audio_info plugin_osd_interface::sound_get_information()
{
	osd::audio_info result;
	result.m_generation = 1;
	result.m_default_sink = PLUGIN_NODE_ID;
	result.m_default_source = 0;
	result.m_nodes.resize(1);

	osd::audio_info::node_info &node = result.m_nodes[0];
	node.m_name = "plugin";
	node.m_display_name = "Plugin output";
	node.m_id = PLUGIN_NODE_ID;
	node.m_rate = osd::audio_rate_range{ m_io.sample_rate, m_io.sample_rate, m_io.sample_rate };
	node.m_port_names = { "FL", "FR" };
	node.m_port_positions = { osd::channel_position::FL(), osd::channel_position::FR() };
	node.m_sinks = 2;
	node.m_sources = 0;
	return result;
}

uint32_t plugin_osd_interface::sound_stream_sink_open(uint32_t node, std::string name, uint32_t rate)
{
	if ((node != PLUGIN_NODE_ID) || (rate != m_io.sample_rate))
	{
		osd_printf_error("plugin: unexpected sound stream %s on node %u at %u Hz\n", name, node, rate);
		return 0;
	}
	return PLUGIN_STREAM_ID;
}

void plugin_osd_interface::sound_stream_sink_update(uint32_t id, const int16_t *buffer, int samples_this_frame)
{
	if (id != PLUGIN_STREAM_ID)
		return;

	std::size_t remaining = std::size_t(samples_this_frame) * 2;
	while (remaining)
	{
		std::size_t const chunk = std::min(remaining, std::size(m_convert));
		for (std::size_t i = 0; i < chunk; i++)
			m_convert[i] = float(buffer[i]) * (1.0f / 32768.0f);
		if (m_io.audio.push(m_convert, chunk) != chunk)
			osd_printf_verbose("plugin: audio queue overflow\n");
		buffer += chunk;
		remaining -= chunk;
	}
}

std::unique_ptr<osd::midi_input_port> plugin_osd_interface::create_midi_input(std::string_view name)
{
	if (name != PLUGIN_MIDI_PORT_NAME)
		return nullptr;
	return std::make_unique<plugin_midi_input>(machine(), m_io);
}

std::unique_ptr<osd::midi_output_port> plugin_osd_interface::create_midi_output(std::string_view name)
{
	if (name != PLUGIN_MIDI_PORT_NAME)
		return nullptr;
	return std::make_unique<plugin_midi_output>(machine(), m_io);
}

std::vector<osd::midi_port_info> plugin_osd_interface::list_midi_ports()
{
	return { osd::midi_port_info{ PLUGIN_MIDI_PORT_NAME, true, true, true, true } };
}

void plugin_osd_interface::output_callback(osd_output_channel channel, const util::format_argument_pack<char> &args)
{
	if ((channel == OSD_OUTPUT_CHANNEL_VERBOSE) && !verbose())
		return;
	if (channel == OSD_OUTPUT_CHANNEL_DEBUG)
		return;

	std::string const text = util::string_format(args);
	m_io.log(channel, text);

	// mirror to stderr for hosts that capture it
	std::fputs(text.c_str(), stderr);
}
