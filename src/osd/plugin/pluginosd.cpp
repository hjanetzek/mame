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

#include "strformat.h"

#include <algorithm>
#include <vector>
#include <cstdio>


namespace {

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

	// there are no windows, but the core expects a UI target; it can
	// later be used to render the machine's layout into a plugin GUI
	render_target *const target = machine.render().target_alloc();
	machine.render().set_ui_target(*target);
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
