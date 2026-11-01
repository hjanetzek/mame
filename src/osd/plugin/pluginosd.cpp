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

#include "imagedev/midiin.h"
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

// shortest time a button state is held, in emulated time
attotime const PANEL_MIN_HOLD = attotime::from_msec(50);

// most frames the panel is rendered per second
constexpr int PANEL_MAX_FPS = 60;

// how often MIDI notes are turned into input changes, and the shortest time
// an input pressed by a note keeps a state: the firmware has to see it when
// it scans its keys
attotime const NOTE_PERIOD = attotime::from_hz(2000);
attotime const NOTE_MIN_HOLD = attotime::from_msec(10);


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
	std::uint64_t current_frame() const { return m_io.frame(m_machine.time()); }

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
					std::uint64_t const frame = m_io.frame(m_machine.time());
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


//============================================================
//  panel_fingerprint - identifies what a primitive list
//  shows, so unchanged frames needn't be drawn again
//============================================================

class panel_fingerprint
{
public:
	template <typename T> void add(T const &value) { add(&value, sizeof(value)); }

	void add(void const *data, std::size_t size)
	{
		u8 const *bytes = reinterpret_cast<u8 const *>(data);
		for (std::size_t i = 0; i < size; i++)
			m_hash = (m_hash ^ bytes[i]) * 0x100000001b3ULL;
	}

	u64 value() const { return m_hash; }

private:
	u64 m_hash = 0xcbf29ce484222325ULL;   // FNV-1a
};

u64 fingerprint(render_primitive_list &list)
{
	panel_fingerprint result;
	list.acquire_lock();
	for (render_primitive const &prim : list)
	{
		result.add(prim.type);
		result.add(prim.bounds);
		result.add(prim.color);
		result.add(prim.flags);
		render_texinfo const &tex = prim.texture;
		if (!tex.base)
			continue;
		result.add(tex.width);
		result.add(tex.height);
		if (!PRIMFLAG_GET_SCREENTEX(prim.flags))
		{
			// layout elements keep their image and sequence ID while the
			// scaled image is reused
			result.add(tex.base);
			result.add(tex.seqid);
			continue;
		}

		// screens get a new sequence ID each time and alternate between
		// two bitmaps, but are small on the machines this is for:
		// compare their pixels instead
		u32 const bytes = (PRIMFLAG_GET_TEXFORMAT(prim.flags) == TEXFORMAT_PALETTE16 || PRIMFLAG_GET_TEXFORMAT(prim.flags) == TEXFORMAT_YUY16) ? 2 : 4;
		u8 const *row = reinterpret_cast<u8 const *>(tex.base);
		for (u32 y = 0; y < tex.height; y++, row += std::size_t(tex.rowpixels) * bytes)
			result.add(row, std::size_t(tex.width) * bytes);
		if (tex.palette)
			result.add(tex.palette, std::size_t(tex.palette_length) * sizeof(rgb_t));
	}
	list.release_lock();
	return result.value();
}

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
	std::fill(std::begin(m_note_held), std::end(m_note_held), -1);
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

	// a machine with a MIDI input reads the host's MIDI itself, other machines
	// can have notes press their inputs
	if (!m_io.note_map.empty() && !midiin_device_enumerator(machine.root_device()).first())
	{
		m_note_timer = machine.scheduler().timer_alloc(timer_expired_delegate(FUNC(plugin_osd_interface::update_notes), this));
		m_note_timer->adjust(NOTE_PERIOD, 0, NOTE_PERIOD);
	}

	std::lock_guard<std::mutex> lock(m_io.panel.mutex);
	m_io.panel.natural_width = std::max(width, 1);
	m_io.panel.natural_height = std::max(height, 1);
}

void plugin_osd_interface::update_notes(s32 param)
{
	// the inputs are set up after the OSD, so the names are looked up here
	if (m_note_map.empty())
	{
		for (plugin_io::note_mapping const &mapping : m_io.note_map)
		{
			ioport_field *found = nullptr;
			for (auto const &port : machine().ioport().ports())
			{
				for (ioport_field &field : port.second->fields())
				{
					if (!field.is_analog() && (field.name() == mapping.input))
						found = &field;
				}
			}
			if (!found)
			{
				osd_printf_warning("plugin: note %d: machine has no input \"%s\"\n", mapping.note, mapping.input);
				continue;
			}

			auto const known = std::find_if(m_note_inputs.begin(), m_note_inputs.end(), [found] (note_input const &input) { return input.field == found; });
			int const index = int(known - m_note_inputs.begin());
			if (known == m_note_inputs.end())
				m_note_inputs.push_back(note_input{ found });
			m_note_map.push_back(note_mapping{ mapping.note, mapping.min_velocity, index });
		}
		if (m_note_map.empty())
		{
			m_note_timer->enable(false);
			return;
		}
	}

	// the host's MIDI that is due
	std::uint64_t const now_frame = m_io.frame(machine().time());
	for (plugin_io::midi_byte const *next = m_io.midi.front(); next && (next->frame <= now_frame); next = m_io.midi.front())
	{
		std::uint8_t const data = next->data;
		m_io.midi.pop(nullptr, 1);
		m_note_assembler.write(data, [this] (std::uint8_t const *message, std::size_t size) { note_message(message, size); }, [] () { });
	}

	// apply the next state of each input once the current one was held long enough
	attotime const now = machine().time();
	for (note_input &input : m_note_inputs)
	{
		while (!input.pending.empty() && (input.pending.front() == input.pressed))
			input.pending.pop_front();
		if (input.pending.empty() || ((now - input.changed) < NOTE_MIN_HOLD))
			continue;

		input.pressed = input.pending.front();
		input.pending.pop_front();
		input.changed = now;
		input.field->set_value(input.pressed ? 1 : 0);
		input.field->port().frame_update();
	}
}

void plugin_osd_interface::note_message(std::uint8_t const *data, std::size_t size)
{
	// notes on any channel
	if ((size != 3) || ((data[0] & 0xe0) != 0x80))
		return;
	std::uint8_t const note = data[1];
	std::uint8_t const velocity = data[2];
	bool const on = ((data[0] & 0xf0) == 0x90) && velocity;

	int &held = m_note_held[note];
	if (held >= 0)
	{
		queue_input(held, false);
		held = -1;
	}
	if (!on)
		return;

	note_mapping const *best = nullptr;
	for (note_mapping const &mapping : m_note_map)
	{
		if ((mapping.note == note) && (mapping.min_velocity <= velocity) && (!best || (mapping.min_velocity > best->min_velocity)))
			best = &mapping;
	}
	if (!best)
		return;

	queue_input(best->input, true);
	held = best->input;
}

void plugin_osd_interface::queue_input(int input, bool pressed)
{
	// a press of an input that is down, by another note, releases it first
	std::deque<bool> &pending = m_note_inputs[input].pending;
	bool const last = pending.empty() ? m_note_inputs[input].pressed : pending.back();
	if (pressed && last)
		pending.push_back(false);
	if (pressed || last)
		pending.push_back(pressed);
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
		m_pointer_events.insert(m_pointer_events.end(), panel.pointers.begin(), panel.pointers.end());
		panel.pointers.clear();
	}

	if (!width || !height)
	{
		m_pointer_events.clear();
		return;
	}
	while (m_pointer_events.size() > plugin_panel::MAX_POINTER_EVENTS)
		m_pointer_events.pop_front();

	// the layout's knobs and sliders, once the inputs they show are resolved
	if (!m_layout_controls_attached && (machine().phase() == machine_phase::RUNNING))
	{
		m_layout_controls.attach(*m_target);
		m_layout_controls_attached = true;
	}

	bool const resized = (s32(width) != m_target->width()) || (s32(height) != m_target->height());
	if (resized)
		m_target->set_bounds(width, height);

	// clicks on the layout's controls; the inputs are only looked at once
	// a frame and machines scan them even less often, so each button state
	// is held for a while before the next change is applied
	bool updated = false;
	attotime const now_emulated = machine().time();
	while (!m_pointer_events.empty())
	{
		plugin_panel::pointer_event const &event = m_pointer_events.front();
		if (m_pointer_buttons.size() <= event.id)
		{
			m_pointer_buttons.resize(event.id + 1, 0);
			m_pointer_changed.resize(event.id + 1, attotime::zero);
		}
		u32 const previous = m_pointer_buttons[event.id];
		u32 const buttons = event.leave ? 0 : event.buttons;
		if (buttons != previous)
		{
			if ((now_emulated - m_pointer_changed[event.id]) < PANEL_MIN_HOLD)
				break;
			m_pointer_changed[event.id] = now_emulated;
		}

		if (event.leave)
		{
			m_target->pointer_left(osd::ui_event_handler::pointer::MOUSE, event.id, 0, event.x, event.y, previous, 0);
		}
		else
		{
			u32 const pressed = buttons & ~previous;
			u32 const released = previous & ~buttons;
			m_target->pointer_updated(osd::ui_event_handler::pointer::MOUSE, event.id, 0, event.x, event.y, buttons, pressed, released, BIT(pressed, 0) ? 1 : 0);
		}
		m_pointer_buttons[event.id] = buttons;
		m_pointer_events.pop_front();
		updated = true;
	}
	if (updated)
		m_target->update_pointer_fields();

	// build a new list when the GUI drew the last one, at a limited rate;
	// the target alternates between two lists, so the one rebuilt here is
	// never the one the GUI may still be drawing
	osd_ticks_t const now = osd_ticks();
	if (!resized && (!consumed || ((now - m_panel_last) < (osd_ticks_per_second() / PANEL_MAX_FPS))))
		return;
	m_panel_last = now;

	// never rebuild a list while the GUI draws one; try again next time
	// rather than hold up the emulation
	std::unique_lock<std::mutex> drawing(panel.draw_mutex, std::try_to_lock);
	if (!drawing)
		return;
	render_primitive_list &list = m_target->get_primitives();

	// drawing the panel in software is costly, so the GUI is only told
	// about a frame that shows something new; the list is passed on
	// anyway, so the one the GUI can draw is never the one rebuilt next
	u64 const shown = fingerprint(list);
	bool const changed = resized || (shown != m_panel_fingerprint);
	m_panel_fingerprint = shown;

	// draw() takes draw_mutex while holding mutex
	drawing.unlock();
	std::lock_guard<std::mutex> lock(panel.mutex);
	panel.list = &list;
	panel.list_width = width;
	panel.list_height = height;
	if (changed)
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

	// should the sound system deliver less audio than the emulated time,
	// the output frames run behind by as much, and the emulation makes up
	// for it instead of leaving the host short of audio for good
	m_io.audio_frames += samples_this_frame;
	m_io.audio_owed = s64(machine().time().as_ticks(m_io.sample_rate)) - s64(m_io.audio_frames);
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
