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

#include "spscqueue.h"

#include "modules/lib/osdobj_common.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>


// name of the MIDI port fed by the host
#define PLUGIN_MIDI_PORT_NAME "plugin"


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
};


class plugin_osd_interface : public osd_common_t
{
public:
	plugin_osd_interface(osd_options &options, plugin_io &io);
	virtual ~plugin_osd_interface();

	// general overridables
	virtual void init(running_machine &machine) override;
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
	plugin_io &m_io;
	float m_convert[2048];
};

#endif // MAME_OSD_PLUGIN_PLUGINOSD_H
