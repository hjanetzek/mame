// license:BSD-3-Clause
// copyright-holders:Jeff
/***************************************************************************

    wersi_slm2.h

    Wersi SLM-2 slave voice card — single Z8611 voice module.

    Each SLM-2 board contains 2 identical voice modules. Each module is a
    Zilog Z8611 (custom-mask SR0106) running at 12 MHz, computing additive
    Fourier synthesis (up to 32 harmonics) and outputting 8-bit samples
    through a DAC 0832. The DAC reference voltage is the envelope signal
    from the co-processor, so amplitude modulation happens in analog.

    In the digital domain this device emulates:
    - Z8611 CPU with internal 4KB ROM (the SR0106 firmware)
    - 256 bytes of external slave RAM (shared with master via IC31)
    - 8-bit waveform output (Port 0 → DAC)
    - RAUD input (pin 5 = P3.0 → triggers IRQ3 from master)
    - ECLK input (pin 30 = P3.3 → per-voice envelope clock, 200 Hz)
    - EXSLA0/EXSLA1 inputs (pin 39 = P3.1, pin 12 = P3.2)
    - T_OUT output (pin 40 = P3.6 → DAC 0832 ILE latch strobe)
    - Bus arbitration: P2.0 = DS/AS enable, P2.2 = RARC
    - P3.4 (pin 29) = NC, P3.5 (pin 10) = NC (verified on PCB)

***************************************************************************/

#ifndef MAME_SOUND_WERSI_SLM2_H
#define MAME_SOUND_WERSI_SLM2_H

#include "cpu/z8/z8.h"
#include "sound/dac.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class wersi_slm2_device : public device_t,
                          public device_sound_interface
{
public:
	// construction/destruction
	wersi_slm2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// external interface (accent from master/co-processor)
	void raud_w(int state);                    // RAM Access Update → pin 5 (P3.0) → IRQ3
	void eclk_w(int state);                    // Envelope Clock → pin 30 (P3.3) → port input
	// void write_ram(offs_t offset, uint8_t data); // write to voice local RAM
	// uint8_t read_ram(offs_t offset);             // read from voice local RAM

	// Shared slave RAM (IC31) — the voice accesses this directly
	// Set by the driver before RAUD; the Z8 LDE/LDEI reads/writes go here
	void set_shared_ram(uint8_t *ptr) { m_shared_ram = ptr; }
	void set_shared_ram_bank(int bank) { m_shared_ram_bank = bank; }
	void set_exsla(uint8_t exsla0, uint8_t exsla1); // pitch exponent bits
	void set_rarc_input(int state) { m_rarc_input = state; } // master bus claim (0=claimed, 1=free)
	void set_envelope(uint16_t value) {
		// 12-bit amplitude envelope target from co-proc DAC1230 via MUX.
		// Don't call m_stream->update() here — let the per-sample RC
		// smoothing in sound_stream_update handle the transition naturally
		// on the next T_OUT-triggered update (avoids attack spikes).
		m_envelope = value;
	}

	// output callbacks
	auto rarc_cb() { return m_rarc_cb.bind(); }       // bus release → SLIRQ
	auto bright_cb() { return m_bright_cb.bind(); }    // P2.3 bright filter select

protected:
	// device-level overrides
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_sound_interface overrides
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	// address maps
	void z8_data_map(address_map &map) ATTR_COLD;

	// Z8 port callbacks
	uint8_t port0_r();
	void port0_w(uint8_t data);
	uint8_t port1_r();
	void port1_w(uint8_t data);
	uint8_t port2_r();
	void port2_w(offs_t offset, uint8_t data, uint8_t mem_mask);
	uint8_t port3_r();
	void port3_w(uint8_t data);

	// external memory access (from Z8 LDE/LDEI instructions via Port 1 bus)
	uint8_t ext_ram_r(offs_t offset);
	void ext_ram_w(offs_t offset, uint8_t data);

	// DAC trace recording
	void trace_event(uint8_t event_type, uint8_t value);

	// internal state
	required_device<z8611_device> m_cpu;
	sound_stream *m_stream;

	// output callbacks
	devcb_write_line m_rarc_cb;
	devcb_write_line m_bright_cb;

	// Local slave RAM (256 bytes) — used when no shared RAM is connected
	uint8_t m_slave_ram[256];

	// Shared slave RAM pointer (IC31: 6264, 8KB) — set by driver
	// When set, Z8 LDE/LDEI accesses go through shared RAM instead of local
	uint8_t *m_shared_ram = nullptr;
	int m_shared_ram_bank = 0;

	// port state
	uint8_t m_port0_data;     // waveform output latch
	uint8_t m_port2_data;     // P2.0=DS/AS enable, P2.2=RARC, P2.3=bright, P2.4-7=routing
	uint8_t m_port2_mask;     // output mask from Z8 core (~P2M): 1=output, 0=input
	uint8_t m_exsla0;         // pitch exponent bit 0
	uint8_t m_exsla1;         // pitch exponent bit 1
	uint8_t m_port3_out;      // last port 3 output (for T_OUT edge detection)
	uint8_t m_rarc_input;     // RARC bus state from master (0=master claimed, 1=free)
	uint8_t m_raud;           // RAUD state (unused now — RAUD triggers IRQ3 directly)
	uint8_t m_eclk;           // ECLK input state on P3.3

	// audio output
	uint8_t m_dac_data;       // latched DAC data
	int16_t m_dac_output;     // current DAC value (waveform sample)
	uint16_t m_envelope;      // 12-bit envelope target (from co-proc DAC1230 via MUX)
	float m_envelope_smooth;  // smoothed envelope (RC filter: R24=1k, C3=2.2µF, τ=2.2ms)

	// DAC trace capture — env WERSI_DAC_TRACE sets path
	FILE *m_dac_trace = nullptr;
	uint32_t m_trace_seq = 0;    // global sequence counter
};

// device type definition
DECLARE_DEVICE_TYPE(WERSI_SLM2_VOICE, wersi_slm2_device)

#endif // MAME_SOUND_WERSI_SLM2_H
