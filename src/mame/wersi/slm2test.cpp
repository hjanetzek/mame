// license:BSD-3-Clause
// copyright-holders:Jeff
/***************************************************************************

    Wersi SLM-2 Standalone Test Machine

    Replays voice capture files produced by the ex20 driver.
    Set WERSI_CAPTURE env var to the capture file path:

      WERSI_CAPTURE=/path/to/capture.bin mamemuse ex20 ...    (write)
      WERSI_CAPTURE=/path/to/capture.bin mamemuse slm2test ... (replay)

    Capture format (272 bytes per record):
      Header (16 bytes): "RA", slot, bank, exsla0, exsla1, cmd, mode, cycles[8] LE
      Data (256 bytes): slave RAM snapshot

***************************************************************************/

#include "emu.h"

#include "sound/wersi_slm2.h"
#include "machine/timer.h"

#include "speaker.h"

#include "logmacro.h"


namespace {

// Capture record: 16-byte header + 256-byte data
static constexpr int RECORD_SIZE = 16 + 256;

class slm2test_state : public driver_device
{
public:
	slm2test_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_voice(*this, "voice")
	{
	}

	void slm2test(machine_config &config) ATTR_COLD;

private:
	void machine_start() override ATTR_COLD;
	void machine_reset() override ATTR_COLD;

	TIMER_CALLBACK_MEMBER(raud_trigger);
	TIMER_DEVICE_CALLBACK_MEMBER(eclk_tick);

	required_device<wersi_slm2_device> m_voice;

	emu_timer *m_raud_timer = nullptr;
	bool m_eclk_state = false;

	// Shared slave RAM — matches ex20 IC31 (8KB = 32 banks × 256 bytes)
	uint8_t m_shared_ram[8192];

	// Capture replay
	struct capture_record {
		uint8_t slot;
		uint8_t bank;
		uint8_t exsla0;
		uint8_t exsla1;
		uint8_t cmd;
		uint8_t mode;
		uint64_t cycles;      // master CPU cycles (for timing)
		uint8_t data[256];
	};
	std::vector<capture_record> m_captures;
	int m_replay_index = 0;
	uint64_t m_base_cycles = 0;  // first record's timestamp

	void schedule_next_raud();
};


//**************************************************************************
//  MACHINE CONFIGURATION
//**************************************************************************

void slm2test_state::slm2test(machine_config &config)
{
	WERSI_SLM2_VOICE(config, m_voice, 12_MHz_XTAL);

	TIMER(config, "eclk_timer").configure_periodic(
		FUNC(slm2test_state::eclk_tick),
		attotime::from_hz(400));

	SPEAKER(config, "speaker").front_center();
	m_voice->add_route(0, "speaker", 1.0);
}


//**************************************************************************
//  MACHINE DRIVER
//**************************************************************************

void slm2test_state::machine_start()
{
	save_item(NAME(m_eclk_state));
	save_item(NAME(m_shared_ram));
	m_raud_timer = timer_alloc(FUNC(slm2test_state::raud_trigger), this);
}

void slm2test_state::machine_reset()
{
	m_eclk_state = false;
	m_replay_index = 0;
	m_base_cycles = 0;
	memset(m_shared_ram, 0, sizeof(m_shared_ram));

	// Connect voice to shared RAM
	m_voice->set_shared_ram(m_shared_ram);
	m_voice->set_shared_ram_bank(0);

	// Load capture file from WERSI_CAPTURE env var
	// Optional: WERSI_SLOT=N filters to a specific voice slot
	m_captures.clear();
	const char *cap_path = getenv("WERSI_CAPTURE");
	if (!cap_path)
	{
		logerror("SLM2TEST: Set WERSI_CAPTURE=/path/to/capture.bin to replay\n");
		return;
	}

	int filter_slot = -1;  // -1 = auto-detect first SETUP slot
	const char *slot_env = getenv("WERSI_SLOT");
	if (slot_env)
		filter_slot = atoi(slot_env);

	FILE *f = fopen(cap_path, "rb");
	if (!f)
	{
		logerror("SLM2TEST: Cannot open %s\n", cap_path);
		return;
	}

	int total = 0, kept = 0;
	uint8_t buf[RECORD_SIZE];
	while (fread(buf, 1, RECORD_SIZE, f) == RECORD_SIZE)
	{
		if (buf[0] != 'R' || buf[1] != 'A')
			break;

		total++;
		if (filter_slot >= 0 && buf[2] != filter_slot)
			continue;

		capture_record rec;
		rec.slot = buf[2];
		rec.bank = buf[3];
		rec.exsla0 = buf[4];
		rec.exsla1 = buf[5];
		rec.cmd = buf[6];
		rec.mode = buf[7];
		rec.cycles = 0;
		for (int i = 0; i < 8; i++)
			rec.cycles |= (uint64_t)buf[8 + i] << (i * 8);
		memcpy(rec.data, &buf[16], 256);
		m_captures.push_back(rec);
		kept++;
	}
	fclose(f);

	// Auto-detect: if no WERSI_SLOT set, pick the first slot with a SETUP command
	if (filter_slot < 0 && !m_captures.empty())
	{
		for (auto &rec : m_captures)
		{
			if (rec.cmd & 0x01)  // SETUP command
			{
				filter_slot = rec.slot;
				break;
			}
		}
		if (filter_slot >= 0)
		{
			// Re-filter to keep only this slot
			std::vector<capture_record> filtered;
			for (auto &rec : m_captures)
				if (rec.slot == filter_slot)
					filtered.push_back(rec);
			m_captures = std::move(filtered);
			kept = m_captures.size();
		}
	}

	if (filter_slot >= 0)
		logerror("SLM2TEST: Loaded %d/%d captures (slot %d) from %s\n", kept, total, filter_slot, cap_path);
	else
		logerror("SLM2TEST: Loaded %d captures from %s\n", total, cap_path);

	if (!m_captures.empty())
	{
		m_base_cycles = m_captures[0].cycles;

		// Don't pre-load — each capture is written at RAUD time.
		// Multiple captures may use the same bank with different data.

		// Schedule first RAUD after boot (100ms)
		m_raud_timer->adjust(attotime::from_msec(100));
	}
}

void slm2test_state::schedule_next_raud()
{
	if (m_replay_index >= (int)m_captures.size())
		return;

	if (m_replay_index == 0)
	{
		// First record already scheduled from reset
		return;
	}

	// Compute delay from timestamp difference
	// Master CPU is 68B09 @ 2 MHz (ex20 crystal)
	uint64_t prev_cycles = m_captures[m_replay_index - 1].cycles;
	uint64_t next_cycles = m_captures[m_replay_index].cycles;
	uint64_t delta = next_cycles - prev_cycles;

	// Convert master cycles to time: 2 MHz = 500ns per cycle
	attotime delay = attotime::from_nsec(delta * 500);

	// Clamp minimum 1ms, maximum 2s
	if (delay < attotime::from_msec(1))
		delay = attotime::from_msec(1);
	if (delay > attotime::from_seconds(2))
		delay = attotime::from_seconds(2);

	m_raud_timer->adjust(delay);
}


//**************************************************************************
//  TIMERS
//**************************************************************************

TIMER_CALLBACK_MEMBER(slm2test_state::raud_trigger)
{
	if (m_replay_index >= (int)m_captures.size())
		return;

	auto &rec = m_captures[m_replay_index];

	// Write snapshot into the correct bank
	if (rec.bank < 32)
		memcpy(&m_shared_ram[rec.bank * 256], rec.data, 256);

	// Set bank and EXSLA for all commands
	// Firmware cmd byte: bit 0=full_setup, bit 1=stop, bit 2=voice_param_update
	// Bit 2 is NOT idle — it triggers voice_param_update which writes TMR
	// (enables T_OUT for DAC latch). All slot-filtered records get RAUD.
	m_voice->set_shared_ram_bank(rec.bank);
	m_voice->set_exsla(rec.exsla0, rec.exsla1);

	const char *cmd_str = "?";
	if (rec.cmd & 0x02) cmd_str = "STOP";
	else if (rec.cmd & 0x01) cmd_str = "SETUP";
	else if (rec.cmd & 0x04) cmd_str = "PARAM_UPDATE";
	else cmd_str = "UPDATE";

	uint64_t rel_cycles = rec.cycles - m_base_cycles;
	logerror("SLM2TEST: === RAUD #%d @%lld ===\n", m_replay_index + 1, (long long)rel_cycles);
	logerror("  slot=%d bank=%d EXSLA=%d%d cmd=$%02X(%s) mode=$%02X\n",
		rec.slot, rec.bank, rec.exsla0, rec.exsla1, rec.cmd, cmd_str, rec.mode);
	logerror("  $F4=%02X $F5=%02X $F6=%02X $F7=%02X\n",
		rec.data[0xF4], rec.data[0xF5], rec.data[0xF6], rec.data[0xF7]);
	logerror("  $F8=%02X $F9=%02X $FA=%02X $FB=%02X $FC=%02X $FD=%02X $FE=%02X $FF=%02X\n",
		rec.data[0xF8], rec.data[0xF9], rec.data[0xFA], rec.data[0xFB],
		rec.data[0xFC], rec.data[0xFD], rec.data[0xFE], rec.data[0xFF]);
	// Micro-program data region
	int mp_start = rec.data[0xF5];
	int mp_count = std::min(32, (int)rec.data[0xF4] + 1);
	logerror("  micro-prog @$%02X (%d bytes):", mp_start, mp_count);
	for (int i = 0; i < mp_count; i++)
		logerror(" %02X", rec.data[(mp_start + i) & 0xFF]);
	logerror("\n");
	// Non-zero harmonic bytes summary
	int nz = 0;
	for (int i = 0; i < 0xA0; i++)
		if (rec.data[i]) nz++;
	logerror("  harmonics: %d/160 non-zero\n", nz);

	m_voice->raud_w(1);
	m_voice->raud_w(0);

	m_replay_index++;
	schedule_next_raud();
}

TIMER_DEVICE_CALLBACK_MEMBER(slm2test_state::eclk_tick)
{
	m_eclk_state = !m_eclk_state;
	m_voice->eclk_w(m_eclk_state ? 1 : 0);
}


//**************************************************************************
//  ROM DEFINITIONS
//**************************************************************************

ROM_START( slm2test )
	// No ROM needed — capture file loaded via WERSI_CAPTURE env var
ROM_END

} // anonymous namespace


//**************************************************************************
//  GAME DRIVERS
//**************************************************************************

//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT  CLASS            INIT        COMPANY  FULLNAME                    FLAGS
SYST( 2026, slm2test, 0,      0,      slm2test, 0,     slm2test_state, empty_init, "Wersi", "SLM-2 Voice Module Test",  MACHINE_NO_SOUND )
