// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  miditest.cpp - tests for the plugin's MIDI output path
//
//    c++ -std=c++20 -Wall -o miditest src/osd/plugin/test/miditest.cpp -Isrc/osd/plugin
//
//============================================================

#include "pluginmidi.h"

#include <cstdio>
#include <string>
#include <vector>


namespace {

int failures = 0;

#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

using bytes = std::vector<std::uint8_t>;

// feed bytes, return the messages and number of dropped ones
std::vector<bytes> assemble(bytes const &input, unsigned *dropped = nullptr)
{
	plugin_midi_assembler assembler;
	std::vector<bytes> result;
	unsigned drops = 0;
	for (std::uint8_t b : input)
		assembler.write(b, [&] (std::uint8_t const *data, std::size_t size) { result.emplace_back(data, data + size); }, [&] () { drops++; });
	if (dropped)
		*dropped = drops;
	return result;
}

void test_assembler()
{
	// running status
	CHECK((assemble({ 0x90, 0x3c, 0x64, 0x3e, 0x64 }) == std::vector<bytes>{ { 0x90, 0x3c, 0x64 }, { 0x90, 0x3e, 0x64 } }));

	// two byte messages with running status
	CHECK((assemble({ 0xc0, 0x05, 0x06 }) == std::vector<bytes>{ { 0xc0, 0x05 }, { 0xc0, 0x06 } }));

	// realtime inside SysEx comes out first and doesn't break it
	CHECK((assemble({ 0xf0, 0x43, 0xf8, 0x00, 0x01, 0xf7 }) == std::vector<bytes>{ { 0xf8 }, { 0xf0, 0x43, 0x00, 0x01, 0xf7 } }));

	// realtime doesn't cancel running status
	CHECK((assemble({ 0x90, 0x3c, 0x64, 0xfe, 0x3e, 0x64 }) == std::vector<bytes>{ { 0x90, 0x3c, 0x64 }, { 0xfe }, { 0x90, 0x3e, 0x64 } }));

	// data without any status is ignored
	CHECK(assemble({ 0x3c, 0x64 }).empty());

	// system common cancels running status
	CHECK((assemble({ 0x90, 0x3c, 0x64, 0xf3, 0x01, 0x3c, 0x64 }) == std::vector<bytes>{ { 0x90, 0x3c, 0x64 }, { 0xf3, 0x01 } }));

	// a status byte aborts unterminated SysEx
	CHECK((assemble({ 0xf0, 0x01, 0x02, 0x90, 0x3c, 0x64 }) == std::vector<bytes>{ { 0x90, 0x3c, 0x64 } }));

	// stray end of SysEx is ignored
	CHECK((assemble({ 0xf7, 0x90, 0x3c, 0x64 }) == std::vector<bytes>{ { 0x90, 0x3c, 0x64 } }));

	// single byte system common
	CHECK((assemble({ 0xf6 }) == std::vector<bytes>{ { 0xf6 } }));

	// overlong SysEx is dropped, following messages still work
	bytes big(plugin_midi_assembler::MAX_SYSEX + 10, 0x11);
	big.front() = 0xf0;
	big.back() = 0xf7;
	big.insert(big.end(), { 0x80, 0x3c, 0x00 });
	unsigned dropped = 0;
	CHECK((assemble(big, &dropped) == std::vector<bytes>{ { 0x80, 0x3c, 0x00 } }));
	CHECK(dropped == 1);
}

void test_records()
{
	spsc_queue<std::uint8_t> queue(1024);
	std::vector<std::uint8_t> scratch;
	bytes const note{ 0x90, 0x3c, 0x64 };
	bytes sysex(300, 0x22);
	sysex.front() = 0xf0;
	sysex.back() = 0xf7;

	CHECK(plugin_midi_record::push(queue, 10, note.data(), note.size(), scratch));
	CHECK(plugin_midi_record::push(queue, 20, sysex.data(), sysex.size(), scratch));
	CHECK(plugin_midi_record::push(queue, 300, note.data(), note.size(), scratch));

	mp_midi_event events[8];
	std::uint8_t buffer[1024];
	std::uint64_t dropped = 0;

	// block 0..256 gets the first two, frames relative to the block
	std::uint32_t count = plugin_midi_record::read(queue, 0, 256, events, 8, buffer, sizeof(buffer), dropped);
	CHECK(count == 2);
	CHECK((events[0].frame == 10) && (bytes(events[0].data, events[0].data + events[0].size) == note));
	CHECK((events[1].frame == 20) && (bytes(events[1].data, events[1].data + events[1].size) == sysex));

	// next block gets the third
	count = plugin_midi_record::read(queue, 256, 512, events, 8, buffer, sizeof(buffer), dropped);
	CHECK((count == 1) && (events[0].frame == 44));

	// late messages go to the start of the block
	CHECK(plugin_midi_record::push(queue, 5, note.data(), note.size(), scratch));
	count = plugin_midi_record::read(queue, 512, 768, events, 8, buffer, sizeof(buffer), dropped);
	CHECK((count == 1) && (events[0].frame == 0));

	// what doesn't fit the event array or buffer stays queued
	CHECK(plugin_midi_record::push(queue, 800, note.data(), note.size(), scratch));
	CHECK(plugin_midi_record::push(queue, 801, note.data(), note.size(), scratch));
	count = plugin_midi_record::read(queue, 768, 1024, events, 1, buffer, sizeof(buffer), dropped);
	CHECK((count == 1) && (events[0].frame == 32));
	count = plugin_midi_record::read(queue, 768, 1024, events, 8, buffer, 2, dropped);
	CHECK((count == 0) && (dropped == 1));   // 3 bytes never fit into 2

	// a full queue rejects a record as a whole
	bytes huge(2000, 0x33);
	CHECK(!plugin_midi_record::push(queue, 900, huge.data(), huge.size(), scratch));
	CHECK(queue.size() == 0);
	CHECK(plugin_midi_record::push(queue, 901, note.data(), note.size(), scratch));
	count = plugin_midi_record::read(queue, 768, 1024, events, 8, buffer, sizeof(buffer), dropped);
	CHECK((count == 1) && (events[0].frame == 133));
}

} // anonymous namespace


int main()
{
	test_assembler();
	test_records();
	std::printf("%s\n", failures ? "FAILED" : "all MIDI tests passed");
	return failures ? 1 : 0;
}
