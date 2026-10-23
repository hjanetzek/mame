// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  pluginmidi.h - MIDI output from the machine to the host:
//  assembling the byte stream into messages and passing them
//  through a queue as timestamped records
//
//  Independent of the emulator so it can be tested on its
//  own (see test/miditest.cpp).
//
//============================================================
#ifndef MAME_OSD_PLUGIN_PLUGINMIDI_H
#define MAME_OSD_PLUGIN_PLUGINMIDI_H

#pragma once

#include "mame_plugin.h"
#include "spscqueue.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>


//============================================================
//  plugin_midi_assembler - turns a MIDI byte stream into
//  complete messages, expanding running status
//============================================================

class plugin_midi_assembler
{
public:
	static constexpr std::size_t MAX_SYSEX = 65536;

	plugin_midi_assembler() { m_message.reserve(MAX_SYSEX); }

	// calls emit(data, size) for each complete message and dropped()
	// for an overlong SysEx message
	template <typename Emit, typename Dropped>
	void write(std::uint8_t data, Emit &&emit, Dropped &&dropped)
	{
		if (data >= 0xf8)
		{
			// system realtime, may appear anywhere, even within SysEx
			emit(&data, 1);
		}
		else if (data == 0xf0)
		{
			m_sysex = true;
			m_running = 0;
			m_message.assign(1, data);
		}
		else if (data == 0xf7)
		{
			if (m_sysex)
			{
				if (m_message.size() < MAX_SYSEX)
				{
					m_message.push_back(data);
					emit(m_message.data(), m_message.size());
				}
				else
				{
					dropped();
				}
			}
			m_sysex = false;
			m_message.clear();
		}
		else if (data & 0x80)
		{
			// any other status ends SysEx; running status only applies to
			// channel messages
			m_sysex = false;
			m_running = (data < 0xf0) ? data : 0;
			m_message.assign(1, data);
			m_expected = message_length(data);
			if (m_message.size() == m_expected)
				complete(emit);
		}
		else if (m_sysex)
		{
			// overlong SysEx is dropped when it ends
			if (m_message.size() < MAX_SYSEX)
				m_message.push_back(data);
		}
		else
		{
			if (m_message.empty())
			{
				// data without status: continue with running status if any
				if (!m_running)
					return;
				m_message.assign(1, m_running);
				m_expected = message_length(m_running);
			}
			m_message.push_back(data);
			if (m_message.size() == m_expected)
				complete(emit);
		}
	}

	// number of bytes in a message with the given status byte, other than SysEx
	static std::size_t message_length(std::uint8_t status)
	{
		switch (status & 0xf0)
		{
		case 0xc0: case 0xd0: return 2;
		case 0xf0:
			switch (status)
			{
			case 0xf1: case 0xf3: return 2;
			case 0xf2: return 3;
			default: return 1;
			}
		default: return 3;
		}
	}

private:
	template <typename Emit>
	void complete(Emit &&emit)
	{
		emit(m_message.data(), m_message.size());
		m_message.clear();
	}

	std::vector<std::uint8_t> m_message;
	std::size_t m_expected = 0;
	std::uint8_t m_running = 0;
	bool m_sysex = false;
};


//============================================================
//  plugin_midi_record - a message in the queue: u64 frame,
//  u32 length, then the message bytes (little endian)
//============================================================

struct plugin_midi_record
{
	static constexpr std::size_t HEADER = 12;

	// producer: append a record, all or nothing; scratch avoids allocating
	// for each message
	static bool push(spsc_queue<std::uint8_t> &queue, std::uint64_t frame, std::uint8_t const *data, std::size_t size, std::vector<std::uint8_t> &scratch)
	{
		scratch.resize(HEADER + size);
		for (int i = 0; i < 8; i++)
			scratch[i] = std::uint8_t(frame >> (i * 8));
		for (int i = 0; i < 4; i++)
			scratch[8 + i] = std::uint8_t(size >> (i * 8));
		std::copy_n(data, size, scratch.begin() + HEADER);
		return queue.push_all(scratch.data(), scratch.size());
	}

	// consumer: take the messages due before end, with event frames
	// relative to start (late messages at 0); messages that don't fit
	// into events or buffer stay queued, a single message larger than
	// buffer is dropped and counted in dropped
	static std::uint32_t read(
			spsc_queue<std::uint8_t> &queue, std::uint64_t start, std::uint64_t end,
			mp_midi_event *events, std::uint32_t max_events, std::uint8_t *buffer, std::size_t buffer_size,
			std::uint64_t &dropped) noexcept
	{
		std::size_t used = 0;
		std::uint32_t count = 0;
		while (count < max_events)
		{
			std::uint8_t header[HEADER];
			if (queue.peek(header, HEADER) < HEADER)
				break;
			std::uint64_t frame = 0;
			for (int i = 0; i < 8; i++)
				frame |= std::uint64_t(header[i]) << (i * 8);
			std::size_t size = 0;
			for (int i = 0; i < 4; i++)
				size |= std::size_t(header[8 + i]) << (i * 8);

			// leave messages of later blocks queued
			if (frame >= end)
				break;

			if (size > buffer_size)
			{
				queue.pop(nullptr, HEADER + size);
				dropped++;
				continue;
			}
			if ((used + size) > buffer_size)
				break;

			queue.pop(nullptr, HEADER);
			queue.pop(buffer + used, size);
			events[count].frame = std::uint32_t((frame > start) ? (frame - start) : 0);
			events[count].size = std::uint32_t(size);
			events[count].data = buffer + used;
			used += size;
			count++;
		}
		return count;
	}
};

#endif // MAME_OSD_PLUGIN_PLUGINMIDI_H
