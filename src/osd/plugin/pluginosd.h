// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  pluginosd.h - headless OSD for running a machine inside
//  an audio plugin
//
//============================================================
#ifndef MAME_OSD_PLUGIN_PLUGINOSD_H
#define MAME_OSD_PLUGIN_PLUGINOSD_H

#pragma once

#include "pluginmidi.h"
#include "spscqueue.h"

#include "modules/lib/osdobj_common.h"

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>


// name of the MIDI port fed by the host
#define PLUGIN_MIDI_PORT_NAME "plugin"


class render_primitive_list;


// the machine's layout rendered for a plugin GUI: the emulation thread
// builds primitive lists, the GUI thread rasterizes them, holding the
// list's lock (as the core expects from renderers on other threads)
struct plugin_panel
{
	struct pointer_event
	{
		std::uint32_t id;
		std::int32_t x, y;       // pixels of the requested size
		std::uint32_t buttons;   // bit 0 is the primary button
		bool leave;
	};

	static constexpr std::size_t MAX_POINTER_EVENTS = 256;

	// GUI thread: rasterize the latest list as RGBA bytes if it is newer
	// than serial and has the given size; returns the serial drawn
	std::uint64_t draw(void *pixels, std::uint32_t width, std::uint32_t height, std::uint64_t serial);

	// emulation thread: stop handing out lists before the render
	// manager goes away, waiting for a draw in progress
	void detach();

	std::mutex mutex;
	std::mutex draw_mutex;   // held while drawing, taken while holding mutex

	// natural size of the layout's current view
	std::uint32_t natural_width = 0;
	std::uint32_t natural_height = 0;

	// set by the GUI, a size of 0 stops rendering
	std::uint32_t width = 0;
	std::uint32_t height = 0;
	std::vector<pointer_event> pointers;

	// latest list built by the emulation thread
	render_primitive_list *list = nullptr;
	std::uint32_t list_width = 0;
	std::uint32_t list_height = 0;
	std::uint64_t serial = 0;
	std::uint64_t fetched = 0;   // serial of the list the GUI last drew
};


// state shared between the host-facing API and the OSD
struct plugin_io
{
	struct midi_byte
	{
		std::uint64_t frame;   // output frame at which the byte is due
		std::uint8_t data;
	};

	plugin_io(std::uint32_t rate, std::size_t audio_frames, std::size_t midi_bytes)
		: sample_rate(rate)
		, audio(audio_frames * 2)
		, midi(midi_bytes)
		, midi_out(midi_bytes * 4)
	{
	}

	void log(osd_output_channel channel, std::string const &text);

	std::uint32_t const sample_rate;

	// interleaved stereo, emulation thread to host
	spsc_queue<float> audio;

	// timestamped bytes, host to emulation thread
	spsc_queue<midi_byte> midi;

	// complete timestamped messages (see plugin_midi_record), emulation
	// thread to host
	spsc_queue<std::uint8_t> midi_out;
	std::atomic<std::uint64_t> midi_out_dropped{ 0 };

	std::mutex log_mutex;
	std::string log_text;

	plugin_panel panel;

	// notes that press inputs of a machine without MIDI input, set before
	// the machine starts
	struct note_mapping
	{
		std::uint8_t note;
		std::uint8_t min_velocity;
		std::string input;
	};
	std::vector<note_mapping> note_map;
};


class plugin_osd_interface : public osd_common_t
{
public:
	plugin_osd_interface(osd_options &options, plugin_io &io);
	virtual ~plugin_osd_interface();

	// general overridables
	virtual void init(running_machine &machine) override;
	virtual void update(bool skip_redraw) override;
	virtual void input_update(bool relative_reset) override { }
	virtual void check_osd_inputs() override { }

	// audio overridables
	virtual bool no_sound() override { return false; }
	virtual bool sound_external_per_channel_volume() override { return false; }
	virtual bool sound_split_streams_per_source() override { return false; }
	virtual uint32_t sound_get_generation() override { return 1; }
	virtual osd::audio_info sound_get_information() override;
	virtual uint32_t sound_stream_sink_open(uint32_t node, std::string name, uint32_t rate) override;
	virtual uint32_t sound_stream_source_open(uint32_t node, std::string name, uint32_t rate) override { return 0; }
	virtual void sound_stream_set_volumes(uint32_t id, const std::vector<float> &db) override { }
	virtual void sound_stream_close(uint32_t id) override { }
	virtual void sound_stream_sink_update(uint32_t id, const int16_t *buffer, int samples_this_frame) override;
	virtual void sound_stream_source_update(uint32_t id, int16_t *buffer, int samples_this_frame) override { }
	virtual void sound_begin_update() override { }
	virtual void sound_end_update() override { }

	// MIDI interface
	virtual std::unique_ptr<osd::midi_input_port> create_midi_input(std::string_view name) override;
	virtual std::unique_ptr<osd::midi_output_port> create_midi_output(std::string_view name) override;
	virtual std::vector<osd::midi_port_info> list_midi_ports() override;

	// osd_output interface
	virtual void output_callback(osd_output_channel channel, const util::format_argument_pack<char> &args) override;

	virtual void process_events() override { }
	virtual bool has_focus() const override { return false; }

private:
	void update_panel();
	void update_notes(s32 param);
	void note_message(std::uint8_t const *data, std::size_t size);
	void queue_input(int input, bool pressed);

	plugin_io &m_io;
	float m_convert[2048];

	// panel rendering, emulation thread
	render_target *m_target = nullptr;
	std::deque<plugin_panel::pointer_event> m_pointer_events;
	std::vector<std::uint32_t> m_pointer_buttons;
	std::vector<attotime> m_pointer_changed;   // last button change per pointer
	osd_ticks_t m_panel_last = 0;

	// MIDI notes pressing inputs (plugin_io::note_map), emulation thread
	struct note_input
	{
		ioport_field *field;
		bool pressed = false;
		attotime changed;
		std::deque<bool> pending;   // states still to apply, each held for a while
	};
	struct note_mapping
	{
		std::uint8_t note;
		std::uint8_t min_velocity;
		int input;   // index into m_note_inputs
	};
	emu_timer *m_note_timer = nullptr;
	std::vector<note_input> m_note_inputs;
	std::vector<note_mapping> m_note_map;
	int m_note_held[128];   // input pressed by each note, -1 for none
	plugin_midi_assembler m_note_assembler;
};

#endif // MAME_OSD_PLUGIN_PLUGINOSD_H
