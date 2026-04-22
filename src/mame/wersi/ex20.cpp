// license:BSD-3-Clause
// copyright-holders:Jeff
/***************************************************************************

    Wersi EX-20 (and DX-10/DX-5) Digital Organ

    Architecture:
    - Master CPU: 68B09 @ 8 MHz
    - Co-Processor: 68B09 @ 8 MHz (envelope/ECLK generation)
    - 6840 PTM: Timer for master
    - 6850 ACIA: MIDI I/O
    - Up to 20 slave voice modules (Z8611 @ 12 MHz on SLM-2 boards)
    - Slave RAM: 256 bytes shared between master and each slave
    - Voice Bank RAM/ROM: 16 KB

    Memory map (Master CPU, IC3: MC68B09):
    $0000-$00FF  Slave RAM (256 bytes, selected by SLRAMB bits 0-4)
    $0100-$01FF  I/O (see below)
    $0200-$1FFF  Co-Processor shared RAM (IC26: 6264, dual-ported)
    $2000-$3FFF  Work RAM (IC4: 6264SUP)
    $4000-$5FFF  Voice Bank: Preset & CV RAM (IC1: 6264SUP, NVRAM)
    $6000-$7FFF  Voice Bank: Rhythm & Seq Voice RAM (IC2: 6264SUP, NVRAM)
    $4000-$7FFF  Voice Bank: Voice ROM (IC5: 27128, when VRAMB bit 2 set)
    $8000-$FFFF  Program ROM (IC3: 27256, 32 KB)

    Memory map (Co-Processor CPU, IC42: MC68B09E):
    $0000-$3FFF  Not used
    $4000-$5FFF  MUX Latch (envelope multiplexer select)
    $6000-$7FFF  ECLK Strobes (IC41: 74154, CA0-CA3 → ECLK01-ECLK12)
    $8000-$9FFF  Envelope DAC (IC27: DAC1230, 12-bit)
    $A000-$DFFF  Not used
    $E000-$FFFF  Shared RAM with master (= master $0200-$1FFF)
                 Program loaded by master during boot from ROM $E200-$EB00
                 Reset vector at $FFFE → $E200

    SLRAMB latch (IC17: 74HCT273N, clocked by write to $0121):
    Bits 0-4: SA8-SA12 — slave select (0-19)
    Bit 5:    EXSLA0   — pitch exponent bit 0 to selected slave
    Bit 6:    EXSLA1   — pitch exponent bit 1 to selected slave
    Bit 7:    CRES     — co-processor reset (0=reset, 1=run)

    PANOUT latch (IC10: 74HCT273N, clocked by write to $0120):
    Bits 0-5: PA0-PA5  — peripheral address bus
    Bit 6:    DCLK     — VFD display serial clock
    Bit 7:    DDAT     — VFD display serial data

    I/O registers ($0100-$01FF):
    $0100  VRAMB     Voice RAM Bank Latch (IC11: 74LS174N)
    $0101  AROUT     Routing byte (AF 20)
    $0102  AMOD      Modulation byte (AF 20)
    $0103  VCS       Volume Control Select (free)
    RAUD addresses are linear: slave N = $0103 + N
    $0104-$0107  RAUD01-04  Slaves 1-4  (MM1 $00 base decoder)
    $0108-$010F  RAUD05-12  Slaves 5-12 (IC50 74HCT138, $08 base)
    $0110-$0117  RAUD13-20  Slaves 13-20 (MME1 IC4, $10 base)
    $0118  EXT       External Select (not used)
    $0119  PANRES    Panel Reset (CB30 button/LED shift registers)
    $011A  DCS       Drum Select (DX 10 only)
    $011B  PANCLK    Panel Clock
    $011C  POTSEL    Pot Select Latch
    $011D  ADCWR     ADC Write Select
    $011E  CRAR      Clear RAM Access (clears SLIRQ flip-flop IC34A)
    $0120  PANOUT    Panel Out Latch (see above)
    $0121  SLRAMB    Slave RAM Bank + EXSLA + CRES (see above)
    $0122  PANIN     Panel In Port Select
    $0123  ADCRD     ADC Read Select
    $0128  QVECT     Interrupt vector source (read: MD0=SLIRQ, MD1=TIRQ, MD2=KBIRQ, MD3=DRDY)
    $0131  KEYB0     Keyboard Select 0 (note number)
    $0132  KEYB1     Keyboard Select 1 (dynamics)
    $0133  KEYB2     Keyboard Select 2 (free)
    $0140-$017F      6850 ACIA (MIDI)
    $0180-$01FF      6840 PTM

***************************************************************************/

#include "emu.h"

#include "cpu/m6809/m6809.h"
#include "machine/6840ptm.h"
#include "machine/6850acia.h"
#include "machine/clock.h"
#include "machine/nvram.h"
#include "bus/midi/midi.h"
#include "sound/flt_biquad.h"
#include "machine/rescap.h"
#include "sound/wersi_slm2.h"
#include "video/roc10937.h"

#include "speaker.h"

#include "wersiex20.lh"

#define LOG_DBG     (1U << 1)
#define LOG_COP     (1U << 2)
#define LOG_CAPTURE (1U << 3)  // Slave RAM + RAUD capture for replay in slm2test
//#define VERBOSE (LOG_DBG)
//#define VERBOSE (LOG_COP)
//(LOG_DBG | LOG_CAPTURE)  // Enable capture logging
#define VERBOSE (0)
#include "logmacro.h"

#define NUM_VOICES 6

#define AUTO_BUTTON_PRESS 1  // Simulate button press to exit blink mode after boot
#define NO_VOICE_RAM_COPY    // Don't copy Voice ROM → RAM; let firmware use VRAMB switching

namespace {

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class wersi_ex20_state : public driver_device
{
public:
	wersi_ex20_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_copcpu(*this, "copcpu")
		, m_ptm(*this, "ptm")
		, m_acia(*this, "acia")
		, m_voice(*this, "voice%u", 0U)
		, m_voice_filt_lp(*this, "vflp%u", 0U)
		, m_voice_filt_norm(*this, "vfn%u", 0U)
		, m_voice_filt_bright(*this, "vfb%u", 0U)
		, m_vfd(*this, "vfd")
		, m_voicebank_view(*this, "voicebank_view")
		, m_panel(*this, "K%u", 0U)
		, m_panel_led(*this, "K%u_%02u", 0U, 0U)
		, m_drawbars(*this, "DB%u", 0U)
		, m_wheel1(*this, "WHEEL1")
		, m_wheel2(*this, "WHEEL2")
		, m_main_vol(*this, "MAINVOL")
	{
	}

	void ex20(machine_config &config) ATTR_COLD;

private:
	void machine_start() override ATTR_COLD;
	void machine_reset() override ATTR_COLD;

	// address maps
	void master_map(address_map &map) ATTR_COLD;
	void cop_map(address_map &map) ATTR_COLD;

	// co-processor I/O handlers
	void cop_eclk_w(offs_t offset, uint8_t data);
	void cop_envelope_dac_w(offs_t offset, uint8_t data);
	void cop_mux_latch_w(offs_t offset, uint8_t data);

	// I/O handlers
	uint8_t io_r(offs_t offset);
	void vramb_w(uint8_t data);
	void arout_w(uint8_t data);
	void amod_w(uint8_t data);
	void raud_w(offs_t offset, uint8_t data);
	void slramb_w(uint8_t data);
	void panout_w(uint8_t data);
	void panclk_w(uint8_t data);
	void panres_w(uint8_t data);
	void potsel_w(uint8_t data);
	void crar_w(uint8_t data);
	void keyb_w(offs_t offset, uint8_t data);
	uint8_t panin_r();
	uint8_t adcrd_r();
	void adcwr_w(uint8_t data);

	// slave RAM access
	uint8_t slave_ram_r(offs_t offset);
	void slave_ram_w(offs_t offset, uint8_t data);

	// slave ready handshake (RARC → IC34A flip-flop → SLIRQ)
	void voice_rarc_w(int voice, int state);
	// bright filter switch (P2.3 → IC28 4053 analog switch)
	void voice_bright_w(int voice, int state);

	// devices
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_copcpu;
	required_device<ptm6840_device> m_ptm;
	required_device<acia6850_device> m_acia;
	optional_device_array<wersi_slm2_device, NUM_VOICES> m_voice;
	optional_device_array<filter_biquad_device, NUM_VOICES> m_voice_filt_lp;
	optional_device_array<filter_biquad_device, NUM_VOICES> m_voice_filt_norm;  // X0: flat/normal
	optional_device_array<filter_biquad_device, NUM_VOICES> m_voice_filt_bright; // X1: 80Hz highpass
	required_device<mic10937_device> m_vfd;

	// voice bank view ($4000-$7FFF) — switches between ROM and RAM
	memory_view m_voicebank_view;

	// Panel — 4 PA lines × 2 chained 74164s = 8 shift registers, 64 buttons
	// SENSE0/PA0: CB31-IC4 → CB31-IC5
	// SENSE1/PA1: CB31-IC2 → CB31-IC3
	// SENSE2/PA2: CB31-IC8 → CB31-IC9
	// SENSE3/PA3: CB31-IC6 → CB30-IC3
	// Each chain: first IC = positions 0-7, second IC = positions 8-15
	required_ioport_array<8> m_panel;    // K0_0..K3_1: [pa*2+half]
	output_finder<8, 8>      m_panel_led;   // 8 ports × 8 LEDs (matching K0-K7 input ports)

	// Potentiometers / analog controls
	required_ioport_array<16> m_drawbars;   // DB0-DB15: drawbar sliders 1-16
	required_ioport m_wheel1;               // modulation wheel 1
	required_ioport m_wheel2;               // modulation wheel 2
	required_ioport m_main_vol;             // main volume

	// state
	uint8_t m_slave_ram_bank = 0;
	uint8_t m_vramb_latch = 0;    // IC11 voice bank latch (MD2-MD7)

	// Shared slave RAM (IC31: 6264, 8KB = 32 banks × 256 bytes)
	// All Z8 voices and the master share this single RAM chip.
	// IC17 SLRAMB latch selects the 256-byte bank (bits 0-4 = SA8-SA12).
	uint8_t m_shared_slave_ram[8192];
	//uint8_t m_voice_bank[NUM_VOICES];  // which bank was last assigned to each voice
	bool m_slirq = false;         // IC34A flip-flop Q output (slave ready → SLIRQ)
	bool m_tirq = false;          // PTM IRQ output (active high from PTM, active low on bus)

	// RARC wired-OR bus state tracking
	// RARC is active-low with pull-up. Low if master OR any slave claims it.
	bool m_rarc_master = false;   // master claims bus (slave_ram_r/w active)
	bool m_rarc_slave[NUM_VOICES] = {}; // per-slave P2.2 output (true = claiming bus)

	// Voice capture — dumps 256-byte slave RAM snapshot at each RAUD
	FILE *m_capture_file = nullptr;
	int m_capture_count = 0;

	void update_mirq();           // recalculate master IRQ from SLIRQ/TIRQ/KBIRQ/DRDY
	void update_rarc(bool update_master);           // recompute wired-OR RARC and broadcast to all slaves

	// MIDI autoplay — inject notes with precise timing
	// Env WERSI_AUTOPLAY=1 enables. Injects into MIDI ring buffer at $2B10.
	TIMER_CALLBACK_MEMBER(midi_autoplay);
	emu_timer *m_autoplay_timer = nullptr;
	int m_autoplay_step = 0;

	void midi_inject(uint8_t byte);

#if AUTO_BUTTON_PRESS
	TIMER_CALLBACK_MEMBER(auto_button_press);
	emu_timer *m_auto_button_timer = nullptr;
	bool m_auto_button_active = false;
#endif
	void tirq_w(int state);       // PTM IRQ callback
	uint8_t qvect_r(offs_t offset); // Read QVECT ($0128) — interrupt source identification

	// ADC / potentiometer state
	uint8_t m_potsel = 0;         // IC2 poti-latch (POTSEL $011C): bits 0-2=channel, 3-4=mux, 5=touch
	uint8_t m_adc_value = 128;    // last ADC conversion result

	// panel shift register state
	uint8_t m_panout_data = 0;    // latched PA0-PA5 from PANOUT write
	uint32_t m_pan_row = 0;       // current scan position
	uint32_t m_pa[4] = {};        // shift register contents (4 PA lines, 16 bits each)

	uint8_t m_vfd_clk = 0xff;

	// Cartridge state
	bool m_has_cartridge = false;  // true if cartridge ROM region is present

	// Envelope MUX state (co-processor → analog → voice DAC reference)
	int m_env_mux_voice = -1;       // currently selected voice for envelope update
	uint16_t m_env_dac_value = 0;   // 12-bit envelope DAC value (DAC1230)
};


//**************************************************************************
//  ADDRESS MAPS
//**************************************************************************

void wersi_ex20_state::master_map(address_map &map)
{
	map.global_mask(0xffff);

	// Slave RAM — directly accessible by master, active slave selected by SLRAMB
	map(0x0000, 0x00ff).rw(FUNC(wersi_ex20_state::slave_ram_r), FUNC(wersi_ex20_state::slave_ram_w));

	// I/O area ($0100-$013F) — directly decoded registers
	// Many are active-low select strobes (directly accent hardware latches),
	// reading them returns open bus or latch contents
	map(0x0100, 0x0127).r(FUNC(wersi_ex20_state::io_r));
	map(0x0130, 0x013f).r(FUNC(wersi_ex20_state::io_r));
	map(0x0139, 0x013a).nopw();                                  // ADAC — Analog DAC (AF 20 audio filter)
	map(0x0100, 0x0100).w(FUNC(wersi_ex20_state::vramb_w));   // VRAMB
	map(0x0101, 0x0101).w(FUNC(wersi_ex20_state::arout_w));   // AROUT
	map(0x0102, 0x0102).w(FUNC(wersi_ex20_state::amod_w));    // AMOD
	// $0103 VCS (free)
	map(0x0104, 0x0117).w(FUNC(wersi_ex20_state::raud_w));    // RAUD01-20: slave N = $0103+N
	map(0x0119, 0x0119).w(FUNC(wersi_ex20_state::panres_w));  // PANRES
	map(0x011b, 0x011b).w(FUNC(wersi_ex20_state::panclk_w));  // PANCLK
	map(0x011c, 0x011c).w(FUNC(wersi_ex20_state::potsel_w));  // POTSEL
	map(0x011d, 0x011d).w(FUNC(wersi_ex20_state::adcwr_w));   // ADCWR
	map(0x011e, 0x011e).w(FUNC(wersi_ex20_state::crar_w));    // CRAR
	map(0x0120, 0x0120).w(FUNC(wersi_ex20_state::panout_w));  // PANOUT
	map(0x0121, 0x0121).w(FUNC(wersi_ex20_state::slramb_w));  // SLRAMB
	map(0x0122, 0x0122).r(FUNC(wersi_ex20_state::panin_r));   // PANIN
	map(0x0123, 0x0123).r(FUNC(wersi_ex20_state::adcrd_r));   // ADCRD
	map(0x0128, 0x012f).r(FUNC(wersi_ex20_state::qvect_r));   // QVECT — interrupt source ID
	map(0x0131, 0x0133).w(FUNC(wersi_ex20_state::keyb_w));    // KEYB0-2

	// 6850 ACIA (MIDI)
	map(0x0140, 0x0141).rw(m_acia, FUNC(acia6850_device::read), FUNC(acia6850_device::write));

	// 6840 PTM
	map(0x0180, 0x0187).rw(m_ptm, FUNC(ptm6840_device::read), FUNC(ptm6840_device::write));

	// Co-Processor shared RAM (IC26: 6264, 8KB, dual-ported via IC20-IC23)
	// IC20-IC22 mux MA0-MA11/CA0-CA11 → RA0-RA11, IC23 muxes MA12/CA12 → RA12
	// Physical RAM address = RA12:RA11-RA0 (13-bit = 8KB)
	//
	// Master view:
	//   $0200-$0FFF → RA12=0, RA0-11=$200-$FFF → physical $0200-$0FFF (not shared)
	//   $1000-$1FFF → RA12=1, RA0-11=$000-$FFF → physical $1000-$1FFF (SHARED)
	//
	// Co-proc view:
	//   $E000-$EFFF → CA12=1, CA0-11=$000-$FFF → physical $1000-$1FFF (SHARED)
	//   $F000-$FFFF → CA12=1, CA0-11=$000-$FFF → mirrors $E000-$EFFF
	//
	// Master init copies ROM $E200-$EAF0 to master $0200-$0AF0 (physical $0200+)
	// But co-proc at $E200 sees physical $1200! So master must ALSO write to $1000+.
	// Actually: the master copies to $0200, then the co-proc vectors at $E000 point
	// to code at $E200+ which maps to physical $1200+. If master doesn't write to
	// $1200+, the co-proc sees zeros. Let's map the full 8KB as shared for now
	// and figure out the exact overlap from firmware behavior.
	map(0x0200, 0x1fff).ram().share("cop_ram");

	// Work RAM (IC4: 6264SUP)
	map(0x2000, 0x3fff).ram().share("main_ram");

	// Voice Bank area ($4000-$7FFF) — banked between ROM, RAM, and cartridge
	// Banking controlled by VRAMB latch (IC11, written via $0100)
	// Priority: CRTDE (bit 6) > VROME (bit 5) > RAM (default)
	// View 0: RAM mode — IC1 Preset & CV RAM + IC2 Voice RAM
	// View 1: ROM mode — IC5 Voice ROM (27128, 16KB)
	// View 2: Cartridge mode — external 27C128 on CT4/B board (16KB)
	map(0x4000, 0x7fff).view(m_voicebank_view);
	m_voicebank_view[0](0x4000, 0x5fff).ram().share("preset_cv");     // IC1: battery-backed, write-protectable
	m_voicebank_view[0](0x6000, 0x7fff).ram().share("voice_ram");     // IC2: battery-backed, write-protectable
	m_voicebank_view[1](0x4000, 0x7fff).rom().region("voicebank", 0); // IC5: 27128 Voice ROM
	m_voicebank_view[2](0x4000, 0x7fff).bankr("cartbank"); // CT4/B: 27C128 cartridge ROM (banked)

	// Program ROM (IC3: 27256, 32KB)
	map(0x8000, 0xffff).rom().region("maincpu", 0);
}


//**************************************************************************
//  CO-PROCESSOR ADDRESS MAP
//**************************************************************************

void wersi_ex20_state::cop_map(address_map &map)
{
	map.global_mask(0xffff);

	// Co-processor memory map (IC42: MC68B09E)
	// Program and data lives entirely in the shared RAM loaded by master.
	// $0000-$3FFF: Not used (reads as open bus)
	// $4000-$5FFF: MUX Latch — selects which slave's envelope is updated
	map(0x4000, 0x5fff).w(FUNC(wersi_ex20_state::cop_mux_latch_w));
	// $6000-$7FFF: ECLK Strobes — decoded by IC41 (74154) to ECLK01-ECLK12
	map(0x6004, 0x6018).w(FUNC(wersi_ex20_state::cop_eclk_w));  // slots 0-20 (offset 0 = voice 1)
	//map(0x6000, 0x7fff).w(FUNC(wersi_ex20_state::cop_eclk_w));
	// $8000-$9FFF: Envelope DAC (IC27: DAC1230, 12-bit)
	map(0x8000, 0x9fff).w(FUNC(wersi_ex20_state::cop_envelope_dac_w));
	// $A000-$DFFF: Not used
	// $E000-$FFFF: Shared RAM with master (IC26: 6264, 8KB)
	// Co-proc $E200 = master $0200 (start of shared region)
	// Co-proc $E000-$E1FF overlaps master's slave RAM / I/O region
	// but the physical 6264 is addressed by RA0-RA12 from the mux.
	// For simplicity: map the full 8KB and share the $0200-$1FFF portion.
	map(0xe000, 0xe1ff).ram();  // Co-proc-only low 512 bytes
	map(0xe200, 0xffff).ram().share("cop_ram");
}


//**************************************************************************
//  CO-PROCESSOR I/O HANDLERS
//**************************************************************************

void wersi_ex20_state::cop_eclk_w(offs_t offset, uint8_t data)
{
	// Two combinational decoders — ALL outputs update on every write.
	// Selected output goes LOW (active), all others HIGH.
	//
	// IC41 (74154, 4-to-16) on MM1:
	//   Inputs: A-D = CA0-CA3.  G1 = NG, G2 = CA4 (active low).
	//   Enabled when CA4=0.
	//   Output 4 = ECLK01, output 5 = ECLK02, ..., output 15 = ECLK12.
	//   ECLK_N = output (N + 3) for voices 1-12.
	//
	// IC5 (74138, 3-to-8) on MME1:
	//   Inputs: A=CA0, B=CA1, C=CA2.  G1=CA4, G2=ECLSEL(/NG).
	//   Enabled when CA4=1.
	//   Y0 = ECLK13, Y1 = ECLK14, ..., Y7 = ECLK20.
	//
	// Co-proc FIRQ slot counter starts at 20 (DEC; LBPL would allow 0):
	//   Address = $6004 + slot_counter.
	//   Slot 20: addr $6018 → no valid output (dummy, skipped by firmware)
	//   Slot 19: ECLK20, Slot 18: ECLK19, ..., Slot 12: ECLK13 (IC5)
	//   Slot 11: ECLK12, Slot 10: ECLK11, ..., Slot 1: ECLK02 (IC41)
	//   Slot  0: ECLK01 (IC41 output 4) — voice 1
	//
	// Slot 0 (voice 1 / ECLK01) IS written — confirmed by trace.
	// The FIRQ loop processes slots 20→0 (21 iterations).

	// Map base is $6004, so offset = slot counter directly (0-19).
	// Slot 0 = voice 0 (m_voice[0]), slot 1 = voice 1, etc.
	// Combinational decoder: selected slot active, all others inactive.
	for (int i = 0; i < NUM_VOICES; i++)
	{
		int active = (i == (int)offset) ? 1 : 0;

		if (active)
			LOGMASKED(LOG_DBG, "COP ECLK voice: %d,  %02X (offset %04X)\n", i + 1, data, offset);

		m_voice[i]->eclk_w(active);
	}
}

void wersi_ex20_state::cop_envelope_dac_w(offs_t offset, uint8_t data)
{
	// IC27 (DAC1230, 12-bit DAC) generates AMPL_ENV voltage.
	// Double-buffered: BYTE1/2 pin = CA0 (offset bit 0).
	//   Write $8000 (CA0=0): loads high byte → bits 4-11 of 12-bit value
	//   Write $8001 (CA0=1): loads low nibble → bits 0-3, transfers to output
	//
	// TODO: The real DAC + sample-and-hold (100nF cap + TL084 buffer) has
	// analog settling time. Consider adding a slew-rate filter if the output
	// sounds too steppy compared to real hardware.
	if ((offset & 1))
	{
		// High byte: data → bits 4-11
		m_env_dac_value = (m_env_dac_value & 0x00f) | (uint16_t(data) << 4);
	}
	else
	{
		// Low nibble: data high nibble → bits 0-3, then transfer
		m_env_dac_value = (m_env_dac_value & 0xff0) | (data >> 4);
	}
}

void wersi_ex20_state::cop_mux_latch_w(offs_t offset, uint8_t data)
{
	// IC37 (74LS174N) latches CD0-CD5 on write to $4000-$5FFF.
	// Selects which voice receives the next envelope DAC value.
	//
	// IC37 outputs:
	//   Q1-Q3 (CD0-CD2) → A,B,C on 4051 MUX → channel select (0-7)
	//   Q4-Q5 (CD3-CD4) → IC39B (74LS139) → chip select S0/S1/S2
	//
	// IC39B decodes CD3:CD4 into chip enables:
	//   00 → S0: voices 1-4  (4051 on MM1 board, 4 of 8 channels used)
	//   01 → S1: voices 5-12 (4051 on SLM-2 board S1)
	//   10 → S2: voices 13-20 (4051 on SLM-2 board S2)
	//
	// Voice number = chip_base + channel:
	//   S0: voice = channel (0-3)
	//   S1: voice = 4 + channel (4-11)
	//   S2: voice = 12 + channel (12-19)
	int channel = data & 0x07;          // CD0-CD2: 4051 channel (0-7)
	int chip = (data >> 3) & 0x03;      // CD3-CD4: IC39B select
	int enable = !BIT(data, 5);
	int voice = -1;
	if (enable) {
	switch (chip)
	{
	case 0:  voice = channel - 4; break;        // S0: voices 0-3 (4 used)
	case 1:  voice = 4 + channel; break;    // S1: voices 4-11
	case 2:  voice = 12 + channel; break;   // S2: voices 12-19
	default: voice = -1; break;             // S3: unused
	}

	m_env_mux_voice = voice;

	// Route the settled DAC voltage to the target voice's sample-and-hold.
	// In hardware: 4051 connects AMPL_ENV to voice's ENV line → cap charges.
	// ENV is the DAC 0832 reference voltage: audio = waveform × envelope.
	if (voice >= 0 && voice < NUM_VOICES && m_voice[voice])
		m_voice[voice]->set_envelope(m_env_dac_value);
	}

	LOGMASKED(LOG_COP, "COP MUX latch = %02X → voice %d env=%02X, chp: %d chn: %d enable:%d\n",
		data, voice, m_env_dac_value, chip, channel, enable);
}


//**************************************************************************
//  I/O HANDLERS
//**************************************************************************

void wersi_ex20_state::vramb_w(uint8_t data)
{
	// IC11 (74LS174N) "Voice Bank Latch" — latches MD2-MD7 on write to $0100
	//   bit 2 (MD2) → Q1: unused
	//   bit 3 (MD3) → Q2: unused
	//   bit 4 (MD4) → Q3: ~VRAME (Voice RAM enable, active low)
	//   bit 5 (MD5) → Q4: VROME (Voice ROM enable) → IC9C NAND → ROM CS
	//   bit 6 (MD6) → Q5: CRTDE (Cartridge enable)
	//   bit 7 (MD7) → Q6: ENKBQ (Keyboard enable)
	m_vramb_latch = data;

	// Voice bank view selection — priority: CRTDE > VROME > RAM
	// CRTDE (bit 6): cartridge ROM at $4000-$7FFF (if cartridge present)
	// VROME (bit 5): on-board Voice ROM (IC5) at $4000-$7FFF
	// else: RAM mode (IC1 preset + IC2 voice)
	int view;
	const char *mode;
	if (BIT(data, 6) && m_has_cartridge)
	{
		view = 2;
		mode = "CART";
	}
	else if (BIT(data, 5))
	{
		view = 1;
		mode = "ROM";
	}
	else
	{
		view = 0;
		mode = "RAM";
	}
	m_voicebank_view.select(view);
	LOGMASKED(LOG_DBG, "VRAMB = %02X → %s (VROME=%d VRAME=%d CRTDE=%d ENKBQ=%d)\n",
		data, mode, BIT(data, 5), !BIT(data, 4), BIT(data, 6), BIT(data, 7));

	//LOGMASKED(LOG_DBG, "VRAMB = %02X (VROME=%d VRAME=%d)\n", data, BIT(data, 2), BIT(data, 5));
}

void wersi_ex20_state::arout_w(uint8_t data)
{
	//LOGMASKED(LOG_DBG, "AROUT = %02X\n", data);
}

void wersi_ex20_state::amod_w(uint8_t data)
{
	//LOGMASKED(LOG_DBG, "AMOD = %02X\n", data);
}

void wersi_ex20_state::midi_inject(uint8_t byte)
{
	// Inject a byte into the polled MIDI data buffer.
	// This is the buffer written by firq_acia_poll_handler ($FCA0)
	// for normal MIDI bytes (<= $F7).
	//
	// Buffer layout (in shared cop_ram, main CPU addresses):
	//   $1D9B+:  midi_poll_data_buffer (data bytes)
	//   $1D1A:   midi_poll_write_index (write position)
	//   $1D18:   midi_poll_byte_count
	address_space &mem = m_maincpu->space(AS_PROGRAM);
	uint8_t count = mem.read_byte(0x1D18);
	uint8_t wi = mem.read_byte(0x1D1A);
	mem.write_byte(0x1D9B + wi, byte);
	mem.write_byte(0x1D1A, wi + 1);
	mem.write_byte(0x1D18, count + 1);
}

TIMER_CALLBACK_MEMBER(wersi_ex20_state::midi_autoplay)
{
	// Programmable MIDI sequence with precise timing.
	// Each step: inject bytes, schedule next step.
	// MIDI note-on: $90 note velocity, note-off: $80 note $00

	// WERSI_NOTE env var selects which MIDI note to play (default: 72 = C5)
	const char *note_env = getenv("WERSI_NOTE");
	static uint8_t note = note_env ? (uint8_t)atoi(note_env) : 72;
#if 0
	struct step { int delay_ms; const char *desc; uint8_t bytes[4]; int nbytes; };
	const step sequence[] = {
		{ 500, "note-on",      {0x90, note, 127, 0}, 3 },
		{ 500, "note-off",    {0x80, note,   0, 0}, 3 },
		{ 500, "note-on",      {0x90, uint8_t(note +12), 127, 0}, 3 },
		{ 500, "note-off",    {0x80, uint8_t(note +12),   0, 0}, 3 },
		{ 500, "note-on",      {0x90, uint8_t(note+24), 127, 0}, 3 },
		{ 500, "note-off",    {0x80, uint8_t(note+24),   0, 0}, 3 },
		{    0, nullptr,       {},                 0 },
	};

	if (sequence[m_autoplay_step].desc == nullptr)
		return;

	auto &s = sequence[m_autoplay_step];
	logerror("EX20: AUTOPLAY [%d] %s\n", m_autoplay_step, s.desc);
	for (int i = 0; i < s.nbytes; i++)
		midi_inject(s.bytes[i]);

	if (sequence[m_autoplay_step].desc)
		m_autoplay_timer->adjust(attotime::from_msec(sequence[m_autoplay_step].delay_ms));
#else
	if (note > 127) return;
	// if (note >= 56 && note < 80) {
	// 	note = 80;
	// }
	logerror("EX20: AUTOPLAY [%d] %d\n", m_autoplay_step, note);
	if (m_autoplay_step % 2 == 0) {
		midi_inject(0x90);
		midi_inject(note);
		midi_inject(127);
	} else {
		midi_inject(0x80);
		midi_inject(note);
		midi_inject(0);
		note++;
	}
	m_autoplay_timer->adjust(attotime::from_msec(50));
#endif
	m_autoplay_step++;
}

void wersi_ex20_state::raud_w(offs_t offset, uint8_t data)
{
	// RAUD (RAM Access Update) — triggers a specific slave to read new
	// parameters from slave RAM and start processing.
	//
	// CORRECTED: RAUD connects to Z8 pin 5 = P3.0 = external interrupt input.
	// RAUD directly triggers IRQ3 on the Z8! This is the "start" signal
	// that causes the Z8 to read the command byte from slave RAM and process
	// new voice parameters.
	//
	// RAUD addresses are linear: slave N = $0103 + N
	// Decoded by three 74HCT138s:
	//   $00 base (offset 0-3):  MM1 board, slaves 1-4
	//   $08 base (offset 4-11): IC50, slaves 5-12
	//   $10 base (offset 12-19): MME1 IC4, slaves 13-20
	int slave = offset;  // 0-based voice index
	//LOGMASKED(LOG_DBG, "RAUD%02d (slave %d) = %02X\n", slave + 1, slave + 1, data);
	logerror("RAUD%02d (slave %d) = %02X\n", slave + 1, slave + 1, data);

	LOGMASKED(LOG_CAPTURE, "CAP_RAUD,%lld,%d\n",
		m_maincpu->total_cycles(), slave);

	// Point the Z8 voice directly at the shared IC31 RAM bank.
	// The real hardware: all Z8s share one 6264 SRAM, with SA8-SA12
	// from SLRAMB selecting the 256-byte window.
	if (slave < NUM_VOICES && m_voice[slave])
	{
		//int bank = m_slave_ram_bank & 0x1f;  // IC17 bits 0-4 = SA8-SA12

		// Z8 LDE/LDEI now reads/writes directly from shared RAM at this bank
		//m_voice[slave]->set_shared_ram_bank(bank);
		//m_voice_bank[slave] = bank;

		// Dump RAUD event to capture file
		if (m_capture_file)
		{
			int bank = m_slave_ram_bank & 0x1f;
			uint64_t cycles = m_maincpu->total_cycles();
			// 16-byte header + 256-byte snapshot = 272 bytes per record
			// Header: "RA", slot, bank, exsla0, exsla1, cmd, mode, cycles(8 bytes LE)
			uint8_t hdr[16];
			hdr[0] = 'R';
			hdr[1] = 'A';
			hdr[2] = (uint8_t)slave;
			hdr[3] = (uint8_t)bank;
			hdr[4] = BIT(m_slave_ram_bank, 5);  // EXSLA0
			hdr[5] = BIT(m_slave_ram_bank, 6);  // EXSLA1
			hdr[6] = m_shared_slave_ram[bank * 256 + 0xF8];  // cmd
			hdr[7] = m_shared_slave_ram[bank * 256 + 0xFA];  // mode
			for (int i = 0; i < 8; i++)
				hdr[8 + i] = (uint8_t)(cycles >> (i * 8));
			fwrite(hdr, 1, 16, m_capture_file);
			fwrite(&m_shared_slave_ram[bank * 256], 1, 256, m_capture_file);
			fflush(m_capture_file);
			m_capture_count++;

			const char *cmd_str = "?";
			if (hdr[6] & 0x04) cmd_str = "IDLE";
			else if (hdr[6] & 0x02) cmd_str = "STOP";
			else if (hdr[6] & 0x01) cmd_str = "SETUP";
			else cmd_str = "UPDATE";
			logerror("CAP #%d: slot=%d bank=%d EXSLA=%d%d cmd=$%02X(%s) mode=$%02X cyc=%lld\n",
				m_capture_count, slave, bank, hdr[4], hdr[5], hdr[6], cmd_str, hdr[7], (long long)cycles);
		}

		// Latch EXSLA for THIS voice at RAUD time.
		// In real hardware, EXSLA is valid on the shared bus only when
		// RARC=high (no other bus access). The master sets SLRAMB with
		// the correct bank+EXSLA before pulsing RAUD for this voice.
		// Other SLRAMB writes for other voices must not affect this one.
		// for (int i = 0; i < NUM_VOICES; i++) {
		// {
		// 	int exsla0 = BIT(m_slave_ram_bank, 5);
		// 	int exsla1 = BIT(m_slave_ram_bank, 6);
		// 	m_voice[slave]->set_exsla(exsla0, exsla1);
		// }

		// Trigger IRQ3 (RAUD → pin 5 → P3.0)
		m_voice[slave]->raud_w(1);
		m_voice[slave]->raud_w(0);
	}
}

void wersi_ex20_state::slramb_w(uint8_t data)
{
	// IC17 (74HCT273N) "Slave RAM Bank Address" latch
	// Bits 0-4 (SA8-SA12): Slave RAM high address = slave select (0-19)
	// Bit 5: EXSLA0 (pitch exponent bit 0)
	// Bit 6: EXSLA1 (pitch exponent bit 1)
	// Bit 7: CRES (co-processor reset, active-low: 0=reset, 1=run)
	uint8_t old = m_slave_ram_bank;
	m_slave_ram_bank = data;

	// Capture logging: timestamp, SLRAMB value (slave select + EXSLA + CRES)
	LOGMASKED(LOG_CAPTURE, "CAP_SLRAMB,%lld,%02x\n",
		m_maincpu->total_cycles(), data);

	// CRES — co-processor reset control
	if (BIT(data, 7) && !BIT(old, 7))
	{
		// Rising edge: release co-processor from reset
		LOGMASKED(LOG_DBG, "SLRAMB = %02X → CRES released, co-proc starts\n", data);
		m_copcpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
	}
	else if (!BIT(data, 7) && BIT(old, 7))
	{
		// Falling edge: assert co-processor reset
		LOGMASKED(LOG_DBG, "SLRAMB = %02X → CRES asserted, co-proc held in reset\n", data);
		m_copcpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	}

	int bank = m_slave_ram_bank & 0x1f;  // IC17 bits 0-4 = SA8-SA12

	int exsla0 = BIT(m_slave_ram_bank, 5);
	int exsla1 = BIT(m_slave_ram_bank, 6);

	for (int i = 0; i < NUM_VOICES; i++) {
		m_voice[i]->set_shared_ram_bank(bank);
		m_voice[i]->set_exsla(exsla0, exsla1);
	}
}

void wersi_ex20_state::panout_w(uint8_t data)
{
	// IC10 (74HCT273) PANOUT latch — latches MD0-MD7 on write
	// Bits 0-5: PA0-PA5 peripheral address (to panel shift registers)
	//           These bits are latched and shifted out on the NEXT PANCLK write
	// Bit 6: DCLK (display VFD serial clock — directly accent MIC10937)
	// Bit 7: DDAT (display VFD serial data — directly accent MIC10937)
	//
	// PANCLK ($011B) is a SEPARATE signal that clocks the 74164 shift registers
	// for LED panel scanning. It has nothing to do with DCLK.
	m_panout_data = data;

	// VFD display serial output (directly accent by PANOUT bits 6-7)
	if (m_vfd_clk != BIT(data, 6)) {
		m_vfd_clk = BIT(data, 6);
		m_vfd->data(BIT(data, 7));   // DDAT
		m_vfd->sclk(m_vfd_clk);   // DCLK
	}
}

void wersi_ex20_state::panclk_w(uint8_t data)
{
	// PANCLK clocks all 74164 shift registers simultaneously.
	// 4 PA lines × 2 chained ICs = 16 bits per PA line.
	m_pan_row++;

	for (int pa = 0; pa < 4; pa++)
	{
		m_pa[pa] = (m_pa[pa] << 1) | BIT(m_panout_data, pa);

		// Update LEDs — first IC (bits 0-7) → K{pa*2}_xx
		//                chained IC (bits 8-15) → K{pa*2+1}_xx
		for (int i = 0; i < 8; i++)
		{
			m_panel_led[pa * 2][i]     = BIT(m_pa[pa], i);
			m_panel_led[pa * 2 + 1][i] = BIT(m_pa[pa], i + 8);
		}
	}
}

uint8_t wersi_ex20_state::panin_r()
{
	// PANIN ($0122) reads IC7 (74LS244N):
	//   Bit 0: SENSE0 — PA0 chain: CB31-IC4 → CB31-IC5
	//   Bit 1: SENSE1 — PA1 chain: CB31-IC2 → CB31-IC3
	//   Bit 2: SENSE2 — PA2 chain: CB31-IC8 → CB31-IC9
	//   Bit 3: SENSE3 — PA3 chain: CB31-IC6 → CB30-IC3
	//   Bit 4: SENSE4
	//   Bit 5: SENSE5
	//   Bit 6: tied to GND
	//   Bit 7: KD (any button pressed, active low, via D2+R16)
	//
	// Scan position 0-7 reads first IC in chain, 8-15 reads second IC.
	uint8_t data = 0;

#if AUTO_BUTTON_PRESS
	if (m_auto_button_active)
	{
		// Simulate T24 "Stage" button press — selects Marimba (3rd ROM voice)
		// K2 bit 4 = PA1 group, scanned at m_pan_row == 5
		data |= 0x80;  // KD = any key down
		// DRAWB preset
		// if (m_pan_row == 3)
		// 	data |= 0x02;  // SENSE1 (PA1 group = K2)

		// MARIMBA preset
		// if (m_pan_row == 5)
		// 	data |= 0x02;

		// FRETLS preset
		// if (m_pan_row == 9)
		//  	data |= 0x02;

		// ACGUIT preset
		// if (m_pan_row == 11)
		//   	data |= 0x02;

		// HUMAN preset
		// if (m_pan_row == 2)
		// 	data |= 0x04;

		// 	STRINGS
		// if (m_pan_row == 16)
		// 	data |= 0x02;

		//	ENSEMBLE
		//if (m_pan_row == 1)
		// 	data |= 0x04;

		// CV1 (row 7)
		if (m_pan_row == 7)
		  	data |= 0x04;

		return ~data;
	}
#endif

	// Read SENSE bits at current scan position
	//auto pos = (m_pan_row > 0) ? (m_pan_row - 1) : 0;
	if (m_pan_row > 0 && m_pan_row <= 16) {
		auto pos = m_pan_row - 1;
		for (int pa = 0; pa < 4; pa++)
		{
			int half = (pos < 8) ? 0 : 1;
			int bit_in_half = pos & 7;
			if (BIT(m_panel[pa * 2 + half]->read(), bit_in_half))
				data |= (1 << pa);
		}
	}

	// KD — any button pressed in any group (bit 7)
	bool kd = false;
	for (int i = 0; i < 8 && !kd; i++)
		kd = (m_panel[i]->read() != 0);
	if (kd)
		data |= 0x80;

	return ~data;
}

void wersi_ex20_state::panres_w(uint8_t data)
{
	// PANRES resets the panel shift registers (74164 CLR pin), NOT the VFD
	// Clears all PA shift register outputs → all LEDs off (side-effect)
	// NB: Starts scanning for pressed button
	m_pan_row = 0;
	for (auto &pa : m_pa)
		pa = 0;
	LOGMASKED(LOG_DBG, "PANRES = %02X (panel shift registers cleared)\n", data);
}

void wersi_ex20_state::potsel_w(uint8_t data)
{
	// IC2 (74174) "Poti-Latch" — latches MD0-MD5 on POTSEL write
	//   bits 0-2 (Q0-Q2): A,B,C → 4051 MUX channel select (0-7)
	//   bits 3-4 (Q3-Q4): MUX inhibit → selects which 4051 is active
	//     Q3=0,Q4=0 → IC5 (drawbar group 1: P1-P8)
	//     Q3=1,Q4=0 → IC4 (drawbar group 2: P9-P16)
	//     Q3=0,Q4=1 → IC7 (controls: main vol, L/R, balance, wheels)
	//     Q3=1,Q4=1 → none
	//   bit 5 (Q5): Touch enable (aftertouch)
	m_potsel = data & 0x3f;
}

void wersi_ex20_state::crar_w(uint8_t data)
{
	// CRAR (Clear RAM Access) — clears IC34B flip-flop
	// Releases master's bus claim. RARC goes high only if no slave holds it.
	m_rarc_master = false;
	update_rarc(true);

	m_slirq = false;
	update_mirq();
	LOGMASKED(LOG_DBG, "CRAR = %02X (bus released, SLIRQ cleared) PC=%04X\n", data, m_maincpu->pc());
}

// RARC callback from slave — called on BOTH edges of P2.2:
//   state=0: slave claims bus (P2.2 driven low via P2M=$00)
//   state=1: slave releases bus (P2.2 goes high-Z via P2M=$04)
//            Rising edge also clocks IC34A → SLIRQ to master.
void wersi_ex20_state::voice_rarc_w(int slave, int state)
{
	// LOGMASKED(LOG_DBG, "RARC from slave=%d state=%d bank=%d\n",
	// 		  slave, state, m_slave_ram_bank & 0x1f);
	logerror("RARC from slave=%d state=%d bank=%d\n",
			  slave, state, m_slave_ram_bank & 0x1f);

	if (slave >= 0 && slave < NUM_VOICES)
	{
		m_rarc_slave[slave] = !state;  // state=0 → claiming, state=1 → released
		update_rarc(false);
	}

	if (state)
	{
		// Slave released bus
		// Rising edge: IC34A flip-flop CLK → Q=1 → SLIRQ asserted
		m_slirq = true;
		update_mirq();
	}
}

void wersi_ex20_state::voice_bright_w(int voice, int state)
{
	// P2.3 from Z8 → IC26 (4053 analog switch) on SLM-2 board
	// state=1 (bright): IC26 disconnects C27 → R23=100k flat feedback
	// state=0 (normal): R22=47k + C27=4.7nF in feedback → lowpass ~720 Hz
	if (voice >= 0 && voice < NUM_VOICES)
	{
		// Switch output gain between the two parallel filters.
		// Both always process audio; only one contributes to output.
		// TODO: smooth crossfade with a timer to avoid clicks.
		//logerror("voice: %d filter: %d\n", voice, state);

		if (m_voice_filt_norm[voice])
			m_voice_filt_norm[voice]->set_output_gain(0, state ? 0.0f : 1.0f);
		if (m_voice_filt_bright[voice])
			m_voice_filt_bright[voice]->set_output_gain(0, state ? 1.0f : 0.0f);
	}
}

void wersi_ex20_state::keyb_w(offs_t offset, uint8_t data)
{
	LOGMASKED(LOG_DBG, "KEYB%d = %02X\n", offset, data);
}

uint8_t wersi_ex20_state::adcrd_r()
{
	// IC8 (ADC 0806) — returns 8-bit result of last conversion
	//adcwr_w(0);
	return m_adc_value;
	//return
}

void wersi_ex20_state::adcwr_w(uint8_t data)
{
	// IC8 (ADC 0806) — start conversion. The selected pot is determined
	// by the POTSEL latch (IC2). Result available on next read.
	//
	// Pot mapping — bits 0-2 select channel, bits 3-5 select MUX:
	//   MUX 0 (IC5): drawbar sliders 1-8
	//   MUX 1 (IC4): drawbar sliders 9-16
	//   MUX 2 (IC7): controls
	//     ch0: footswell (max $FF if not connected)
	//     ch1: wheel 1 (PITCH UP)
	//     ch2: wheel 2 (PITCH DOWN)
	//     ch3: right
	//     ch4: balance
	//     ch5: main volume
	//     ch6: left
	//     ch7: keyboard aftertouch (0 on EX-20, no keyboard)
	//
	// Wheel ADC and pitch modulation (from Ghidra analysis of mk1_ic3):
	//
	// The MK1/EX-20 firmware has NO mode-specific wheel disable. The only
	// difference between MK1 and EX-20 is the display string selected at
	// $CE41 via IO_KEYB1 bit 7 ("WERSI MK1 V1.21" vs "WERSI EX20 V1.21").
	//
	// The IRQ handler ($C216) reads ADCRD every tick. The POTSEL table at
	// $B109 interleaves control pots (footswell/wheel1/wheel2/aftertouch)
	// with every main channel read, so wheels are sampled every ~5 IRQ ticks.
	// Results: wheel1 → RAM[$2E39], wheel2 → RAM[$2E3A].
	//
	// compute_voice_modulation ($B131) processes the wheel values:
	//   Wheel 1 ($B25E): scaled ×~1.06, slew-limited → $2E71 → $2D52
	//   Wheel 2 ($B28E): deadzone $76-$8A, scaled ×20 outside → $2E72
	//
	// At $B2F6 the wheel pitch offset ($2D52) is ADDED to the MIDI pitch
	// bend result — they share the same accumulator. The combined value
	// is clamped to [0,$3FF] → $2D50 → written to slave Z8 via
	// voice_write_pitch ($AC37).
	//
	// If the wheel ADC values are not at center (~$7F), the wheel offset
	// permanently biases the pitch. MIDI pitch bend then only works in
	// one direction (or not at all if the bias saturates the range).
	//
	// On real EX-20 hardware (no physical wheels), the unconnected MUX
	// inputs on IC7 ch1/ch2 are presumably tied to Vref/2 on the PCB,
	// naturally reading ~$7F-$80. The firmware doesn't need a software
	// guard because the hardware provides the neutral center value.
	int channel = m_potsel & 0x07;

	// Bits 3-5 are individual INH lines to each 4051 (active HIGH = disabled).
	// The LOW bit selects the active MUX:
	//   bit 3 = 0 → IC5 (drawbar 1-8)
	//   bit 4 = 0 → IC4 (drawbar 9-16)
	//   bit 5 = 0 → IC1 (controls)
	m_adc_value = 0xff;  // default max
	static int first_scan = 16;
	if (!BIT(m_potsel, 3))
	{
		// IC5: drawbar sliders 1-8
		//m_adc_value = m_drawbars[channel]->read();
		m_adc_value = 0;
		if (first_scan > 0) {
			first_scan--;
		} else {
			if (channel == 0)
				m_adc_value = 0xff;
		}
	}
	else if (!BIT(m_potsel, 4))
	{
		m_adc_value = 0;
		// IC4: drawbar sliders 9-16
		// m_adc_value = m_drawbars[8 + channel]->read();
		// m_adc_value = m_drawbars[8 + channel]->read();
	}
	else if (!BIT(m_potsel, 5))
	{
		// IC1: controls
		switch (channel)
		{
		case 0: m_adc_value = 0xff; break;              // footswell (max = not connected)
		// Wheel 1/2: must read ~$7F (center) for EX-20 — see wheel/pitch analysis above.
		// For MK1 with physical wheels: m_wheel1->read() / m_wheel2->read().
		// Wheel 2 firmware has deadzone $76-$8A around center.
		case 1: m_adc_value = 0x0; break;              // wheel 2
		case 2: m_adc_value = 0x7f; break;              // wheel 1
		case 3: m_adc_value = 0xff; break;              // right
		case 4: m_adc_value = 0xff; break;              // balance
		case 5: m_adc_value = 0xff; break; //m_main_vol->read(); break; // main volume
		case 6: m_adc_value = 0xff; break;              // left
		case 7: m_adc_value = 0x1; break;              // aftertouch (none on EX-20)
		}
	}
	//printf("pot %d %d\n", mux, channel);
}

uint8_t wersi_ex20_state::qvect_r(offs_t offset)
{
	// QVECT ($0128-$012F) — interrupt vector source identification
	// IC10A (74LS244N) drives MD0-MD3 when MIRQ active:
	//   bit 0 = SLIRQ (slave ready)      — active low, pull-up R3 4k7
	//   bit 1 = TIRQ  (PTM timer)        — active low, pull-up R1 4k7
	//   bit 2 = KBIRQ (keyboard)         — active low, pull-up R5 4k7
	//   bit 3 = DRDY  (drum ready)       — active low, pull-up R12 2k2
	// IC10B drives MD4-MD7 when QVECT active:
	//   bit 4 = CRTD0 (cartridge code 0) — active low, pull-up R11 2k2
	//   bit 5 = CRTD1 (cartridge code 1)
	//   bit 6 = SW0   (foot switch 0)
	//   bit 7 = SW1   (foot switch 1)
	uint8_t data = 0xff;
	if (m_slirq) data &= ~0x01;  // SLIRQ active
	if (m_tirq)  data &= ~0x02;  // TIRQ active (from PTM)
	// KBIRQ, DRDY not yet emulated
	// CRTD0/CRTD1: cartridge identification pins (active low)
	// No cartridge: both high (pull-ups).
	// 20-sound ROM cartridge: both low (CRTD0=0, CRTD1=0).
	if (m_has_cartridge)
		data &= ~0x30;  // CRTD0 + CRTD1 low = 20-sound ROM cartridge
	// SW0/1 not yet emulated
	return data;
}

uint8_t wersi_ex20_state::io_r(offs_t offset)
{
	// Most I/O registers are active-low select strobes — reading returns 0xFF
	return 0xff;
}

void wersi_ex20_state::tirq_w(int state)
{
	// PTM IRQ output (pin 9) → TIRQ signal → IC10A pin 4 (QVECT bit 1)
	// Also drives MIRQ via IC14D AND gate → master CPU IRQ line
	// bool old = m_tirq;
	m_tirq = (state != 0);
	// if (m_tirq != old)
	// 	LOGMASKED(LOG_DBG, "TIRQ %s\n", m_tirq ? "asserted (PTM IRQ)" : "cleared");
	update_mirq();
}

void wersi_ex20_state::update_mirq()
{
	// IC14D (74HCT08N AND gate) combines interrupt sources into MIRQ
	// MIRQ is active when ANY source is active (active-low OR = AND of active-high)
	bool mirq = m_slirq || m_tirq;  // TODO: add KBIRQ, DRDY
	m_maincpu->set_input_line(M6809_IRQ_LINE, mirq ? ASSERT_LINE : CLEAR_LINE);
}

void wersi_ex20_state::update_rarc(bool update_master)
{
	// RARC is a wired-OR active-low bus with pull-up.
	// Low if master claims it OR any slave claims it (P2.2 output low).
	// Each slave reads the global state via P2.2 input (P2M=$04).
	bool bus_claimed = m_rarc_master;
	int slave_claims = 0;
	for (int i = 0; i < NUM_VOICES; i++) {
		if (m_rarc_slave[i]) slave_claims++;

		bus_claimed |= m_rarc_slave[i];
	}
	int rarc_state = bus_claimed ? 0 : 1;
	printf("slave bus claimed %d - by master %d/%d, by slaves %d\n", bus_claimed, m_rarc_master, update_master, slave_claims);
	for (int i = 0; i < NUM_VOICES; i++)
		m_voice[i]->set_rarc_input(rarc_state);

	if (slave_claims > 1) {
		abort();
	}

	if (!bus_claimed) {
		auto d = m_slave_ram_bank;
		auto b = (m_slave_ram_bank & 0x1f) * 256 + 0xff;
		static int exsla0 = -1;
		static int exsla1 = -1;
		static int pitch = -1;
		if (BIT(d, 5) != exsla0 ||
			 BIT(d, 6) != exsla1 ||
			 //(m_shared_slave_ram[0xff] & 0xfe) != pitch) && !bus_claimed) {
			 (m_shared_slave_ram[b]) != pitch) {
			 exsla0 = BIT(d, 5);
			 exsla1 = BIT(d, 6);
			 pitch = m_shared_slave_ram[b]; // & 0xfe; // drop bit0 - modulation direction
			 logerror("PITCH %04X %d (slave=%d EXSLA=%d,%d)\n",
					  ((BIT(d, 6) ? 0 : 1) << 9) | ((BIT(d, 5) ? 0 : 1) << 8) | (m_shared_slave_ram[b]),
					  ((m_shared_slave_ram[b] & 0x80) ? 1 : -1) * (((BIT(d, 6) ? 0 : 1) << 9) | ((BIT(d, 5) ? 0 : 1) << 8) |
																	  (m_shared_slave_ram[b] & 0xfe)),
					  d & 0x1f, BIT(d, 5), BIT(d, 6));
		}
		//LOGMASKED(LOG_DBG, "RARC from slave=%d → SLIRQ asserted, PC=%04X\n", voice, m_copcpu->pc());

	}

}

// Slave RAM (IC31: 6264, 8KB shared)
// Master reads/writes go to the shared RAM at bank * 256 + offset.
// IC17 SLRAMB latch bits 0-4 = SA8-SA12 = bank select.
// The Z8 voices access this same RAM via RAUD-triggered copy.
uint8_t wersi_ex20_state::slave_ram_r(offs_t offset)
{
	int bank = m_slave_ram_bank & 0x1f;
	// SLRAM active → IC34B sets RARC low (master claims bus)
	m_rarc_master = true;
	update_rarc(true);
	return m_shared_slave_ram[bank * 256 + (offset & 0xff)];
}

void wersi_ex20_state::slave_ram_w(offs_t offset, uint8_t data)
{
	int bank = m_slave_ram_bank & 0x1f;
	// SLRAM active → IC34B sets RARC low (master claims bus)
	m_rarc_master = true;
	update_rarc(true);

	LOGMASKED(LOG_CAPTURE, "CAP_RAM,%lld,%d,%02x,%02x, PC=%04X\n",
		m_maincpu->total_cycles(), bank, offset, data, m_maincpu->pc());

	m_shared_slave_ram[bank * 256 + (offset & 0xff)] = data;
}

//**************************************************************************
//  MACHINE CONFIGURATION
//**************************************************************************

void wersi_ex20_state::ex20(machine_config &config)
{
	// Master CPU — 68B09 @ 8 MHz (IC3)
	MC6809(config, m_maincpu, 8_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &wersi_ex20_state::master_map);

	// Co-Processor CPU — 68B09E @ 8 MHz (IC42)
	// Runs from shared RAM at $E000-$FFFF (loaded by master during boot)
	// Handles: ECLK generation, envelope DAC, MUX routing
	// Held in reset by CRES (SLRAMB bit 7) until master loads program
	MC6809E(config, m_copcpu, 8_MHz_XTAL);
	m_copcpu->set_addrmap(AS_PROGRAM, &wersi_ex20_state::cop_map);

	// 6840 PTM (IC6) — timer
	// Internal clock = E = 2 MHz (8 MHz / 4)
	// CLK3 = 500 kHz external
	// IRQ (pin 9) → TIRQ → IC10A (QVECT bit 1) → MIRQ via IC14D
	// OUT1 (pin 27) → not connected (VCC via R7 pull-up)
	// OUT2 (pin 3) → 500 kHz output (label on schematic)
	// OUT3 (pin 6) → R8 10k → Q1 BC237 base → CIRQ (INVERTED) → co-proc FIRQ
	PTM6840(config, m_ptm, 8_MHz_XTAL / 4);
	m_ptm->set_external_clocks(0, 0, 500000);
	m_ptm->irq_callback().set(FUNC(wersi_ex20_state::tirq_w));
	// OUT3 (pin 6) → R8 10k → Q1 BC237 base → CIRQ → co-proc FIRQ
	// BC237 inverts: OUT3 high → FIRQ asserted, OUT3 low → FIRQ deasserted
	m_ptm->o3_callback().set([this](int state) {
		m_copcpu->set_input_line(M6809_FIRQ_LINE, state ? ASSERT_LINE : CLEAR_LINE);
	});

	// Battery-backed RAMs
	NVRAM(config, "main_ram", nvram_device::DEFAULT_ALL_0);
	// Voice bank RAM defaults: pre-populate from Voice ROM (mk1_ic5_s3.bin)
	// On real hardware, these are battery-backed and factory-programmed.
	// First 8KB ($4000-$5FFF) = preset/CV data, second 8KB ($6000-$7FFF) = voice/rhythm.
	NVRAM(config, "preset_cv", nvram_device::DEFAULT_ALL_0);  // IC1: will be filled in machine_reset
	NVRAM(config, "voice_ram", nvram_device::DEFAULT_ALL_0);  // IC2: will be filled in machine_reset

	// 6850 ACIA — MIDI
	ACIA6850(config, m_acia);
	m_acia->irq_handler().set_inputline(m_maincpu, M6809_FIRQ_LINE);
	m_acia->txd_handler().set("mdout", FUNC(midi_port_device::write_txd));

	// MIDI clock (31.25 kbaud = 500 kHz / 16)
	clock_device &midiclock(CLOCK(config, "midiclock", 500_kHz_XTAL));
	midiclock.signal_handler().set(m_acia, FUNC(acia6850_device::write_txc));
	midiclock.signal_handler().append(m_acia, FUNC(acia6850_device::write_rxc));

	// MIDI ports
	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(m_acia, FUNC(acia6850_device::write_rxd));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");

	// VFD display (MIC10937, same as Wersi DX series)
	// Serial data via PANOUT latch bits 6-7 (DCLK, DDAT)
	MIC10937(config, m_vfd).set_port_value(0);
	config.set_default_layout(layout_wersiex20);

	// Voice modules — Z8611 @ 12 MHz, each with output filter
	for (int i = 0; i < NUM_VOICES; i++) {
		WERSI_SLM2_VOICE(config, m_voice[i], 12_MHz_XTAL);
		m_voice[i]->rarc_cb().set([this, i](auto data){ voice_rarc_w(i, data); });
		m_voice[i]->bright_cb().set([this, i](int state){ voice_bright_w(i, state); });
		m_voice[i]->set_shared_ram(m_shared_slave_ram);

		// Per-voice output filter — IC25 (TL084) + IC26 (4053 bright switch)
		// IC25 (TL084) + IC26 (4053 bright switch) per voice module.
		// Components: R21=6.8k, R22=47k, R23=100k, C20=47nF, C27=4.7nF
		// Both paths: R22(47k) || C27(4.7nF) in feedback
		// X0 normal: R21(6.8k), R23(100k)||C20(47nF) — flat/unity gain
		// X1 bright: C20(47nF) only — highpass ~80 Hz (bass cut, brighter tone)
		//   (documented as "80 Hz Tiefpaß" in Wersi schematic description,
		//    but highpass makes musical sense; circuit topology needs verification)
		// C2=2.2µF is DC blocking. D3/D4 (1N4148) usually not soldered.
		// Two parallel filters — voice feeds both, gains crossfade via P2.3:
		// Normal filter: flat (1 Hz highpass = pass-through)
		FILTER_BIQUAD(config, m_voice_filt_norm[i]);
		m_voice_filt_norm[i]->setup(filter_biquad_device::biquad_type::HIGHPASS, 1.0, 0.707, 1.0);
		// Bright filter: 80 Hz highpass (bass cut)
		FILTER_BIQUAD(config, m_voice_filt_bright[i]);
		m_voice_filt_bright[i]->setup(filter_biquad_device::biquad_type::HIGHPASS, 80.0, 0.707, 1.0);

		FILTER_BIQUAD(config, m_voice_filt_lp[i]);
		m_voice_filt_lp[i]->opamp_mfb_lowpass_setup(RES_K(6.8), 0, RES_K(6.8), 0, CAP_N(2.2));
	}

	// Audio output: voice → both filters → speaker
	// P2.3 switches output gain: normal=1/bright=0 or normal=0/bright=1
	SPEAKER(config, "speaker").front_center();
	for (int i = 0; i < NUM_VOICES; i++) {
		m_voice[i]->add_route(0, m_voice_filt_lp[i], 1.0);
		m_voice_filt_lp[i]->add_route(0, m_voice_filt_norm[i], 1.0);
		m_voice_filt_lp[i]->add_route(0, m_voice_filt_bright[i], 1.0);
		m_voice_filt_norm[i]->add_route(0, "speaker", 0.5);
		m_voice_filt_bright[i]->add_route(0, "speaker", 0.5);
	}
}


//**************************************************************************
//  MACHINE DRIVER
//**************************************************************************

void wersi_ex20_state::machine_start()
{
	m_panel_led.resolve();

	save_item(NAME(m_slave_ram_bank));
	save_item(NAME(m_vramb_latch));
	save_item(NAME(m_shared_slave_ram));
	//save_item(NAME(m_voice_bank));
	save_item(NAME(m_slirq));
	save_item(NAME(m_tirq));
	save_item(NAME(m_rarc_master));
	save_item(NAME(m_rarc_slave));
	save_item(NAME(m_panout_data));
	save_item(NAME(m_pan_row));
	save_item(NAME(m_pa));

	// ACIA modem control — DCD and CTS must be asserted (active low = 0)
	// for the 6850 to operate. Without this, the ACIA init at $FD3F hangs
	// because the ACIA status register never shows "ready".
	m_acia->write_dcd(0);
	m_acia->write_cts(0);

	// Cartridge bank setup — WERSI_BANK=1..12 selects ROM (default: 9)
	m_has_cartridge = false;
	memory_region *cart = memregion("cartridge");
	if (cart)
	{
		int bank = 9;  // default
		const char *bank_env = getenv("WERSI_BANK");
		if (bank_env)
			bank = std::clamp(atoi(bank_env), 1, 12);

		membank("cartbank")->configure_entries(0, 12, cart->base(), 0x4000);
		membank("cartbank")->set_entry(bank - 1);  // 0-indexed
		m_has_cartridge = true;
		logerror("Cartridge: bank %d (rom%d.bin) selected\n", bank, bank);
	}

	// Voice capture file — env WERSI_CAPTURE sets path, default off
	m_capture_count = 0;
	const char *cap_path = getenv("WERSI_CAPTURE");
	if (cap_path)
	{
		m_capture_file = fopen(cap_path, "wb");
		if (m_capture_file)
			logerror("EX20: Voice capture → %s\n", cap_path);
		else
			logerror("EX20: WARNING: Could not open %s\n", cap_path);
	}

#if AUTO_BUTTON_PRESS
	m_auto_button_timer = timer_alloc(FUNC(wersi_ex20_state::auto_button_press), this);
#endif

	// MIDI autoplay
	m_autoplay_timer = timer_alloc(FUNC(wersi_ex20_state::midi_autoplay), this);
}

void wersi_ex20_state::machine_reset()
{
	m_slave_ram_bank = 0;
	m_vramb_latch = 0;
	m_slirq = false;
	m_tirq = false;

	// Default filter state: normal mode (bright off)
	for (int i = 0; i < NUM_VOICES; i++)
	{
		if (m_voice_filt_norm[i])
			m_voice_filt_norm[i]->set_output_gain(0, 1.0f);
		if (m_voice_filt_bright[i])
			m_voice_filt_bright[i]->set_output_gain(0, 0.0f);
	}
	m_rarc_master = false;
	memset(m_rarc_slave, 0, sizeof(m_rarc_slave));
	memset(m_shared_slave_ram, 0, sizeof(m_shared_slave_ram));
	//memset(m_voice_bank, 0, sizeof(m_voice_bank));
	m_voicebank_view.select(0);  // Start in RAM mode

	// CRES = bit 7 of SLRAMB latch (IC17). On reset, latch is cleared
	// (CLR=VCC means it's NOT cleared by reset — but m_slave_ram_bank=0
	// means CRES=0=held in reset). Master writes $80 to SLRAMB after
	// loading the co-proc program to release it.
	m_copcpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);

	// System RESET (active-low) is connected to both:
	// - IC10 PANOUT latch CLR (clears all outputs to 0)
	// - MIC10937 VFD POR input directly
	// On reset: POR=0 (active), then POR=1 (run). PANRES ($0119) is unrelated.
	m_vfd->por(0);  // active-low reset
	m_vfd->por(1);  // release — VFD ready to accept data

	// PTM 6840 gate inputs: GATE1-3 (pins 26,2,5) are NOT CONNECTED
	// on the schematic (open circles). The PTM timers operate in one-shot
	// mode where gate input is ignored. Do NOT set gates high.

#ifndef NO_VOICE_RAM_COPY
	// Pre-populate voice bank RAM from Voice ROM if empty.
	// The firmware reads presets from ROM via VRAMB bit 5 (VROME) switching,
	// so this copy should not be necessary. Disable with NO_VOICE_RAM_COPY
	// to test firmware-driven ROM access on fresh NVRAM.
	{
		memory_region *vrom = memregion("voicebank");
		memory_share *preset = memshare("preset_cv");
		memory_share *voice = memshare("voice_ram");
		if (vrom && preset && voice)
		{
			bool empty = true;
			uint8_t *pdata = reinterpret_cast<uint8_t *>(preset->ptr());
			for (int i = 0; i < 16 && empty; i++)
				if (pdata[i] != 0) empty = false;

			if (empty)
			{
				uint8_t *vdata = reinterpret_cast<uint8_t *>(voice->ptr());
				int psize = std::min(int(preset->bytes()), 0x2000);
				int vsize = std::min(int(voice->bytes()), 0x2000);
				memcpy(pdata, vrom->base(), psize);
				memcpy(vdata, vrom->base() + 0x2000, std::min(int(vrom->bytes()) - 0x2000, vsize));
				logerror("Voice bank RAM populated from Voice ROM (%d + %d bytes)\n", psize, vsize);
			}
		}
	}
#endif

#if AUTO_BUTTON_PRESS
	// Simulate a button press after blink mode starts (~2 sec into boot).
	// The firmware loops in blink mode at $B8E5 until a button is detected
	// via panin_r. This auto-press exits blink mode so the system enters
	// normal operation. The press is held for 100ms then released.
	m_auto_button_active = false;
	m_auto_button_timer->adjust(attotime::from_msec(500));
#endif

	m_autoplay_step = 0;
	if (getenv("WERSI_AUTOPLAY"))
	{
		logerror("EX20: MIDI autoplay enabled\n");
		m_autoplay_timer->adjust(attotime::from_msec(1000));  // wait for boot
	}
}

#if AUTO_BUTTON_PRESS
TIMER_CALLBACK_MEMBER(wersi_ex20_state::auto_button_press)
{
	if (!m_auto_button_active)
	{
		// Press: panin_r will return key-down for ~100ms
		m_auto_button_active = true;
		m_auto_button_timer->adjust(attotime::from_msec(100));
		logerror("AUTO: button press (exiting blink mode)\n");
	}
	else
	{
		// Release
		m_auto_button_active = false;
		logerror("AUTO: button release\n");
	}
}
#endif

//**************************************************************************
//  INPUT PORTS — Panel buttons (CB30 board, 6 shift register groups)
//**************************************************************************

static INPUT_PORTS_START( ex20 )
	// PA0 first IC: CB31-IC4 (A-H / Presets 1-8)
	PORT_START("K0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T1 A")            PORT_CODE(KEYCODE_1)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T2 B")            PORT_CODE(KEYCODE_2)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T3 C")            PORT_CODE(KEYCODE_3)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T4 D")            PORT_CODE(KEYCODE_4)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T5 E")            PORT_CODE(KEYCODE_5)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T6 F")            PORT_CODE(KEYCODE_6)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T7 G")            PORT_CODE(KEYCODE_7)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T8 H")            PORT_CODE(KEYCODE_8)

	// PA0 chained IC: CB31-IC5 (Function Matrix columns 1-8)
	PORT_START("K1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T9  Matrix 1")    PORT_CODE(KEYCODE_Q)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T10 Matrix 2")    PORT_CODE(KEYCODE_W)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T11 Matrix 3")    PORT_CODE(KEYCODE_E)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T12 Matrix 4")    PORT_CODE(KEYCODE_R)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T13 Matrix 5")    PORT_CODE(KEYCODE_T)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T14 Matrix 6")    PORT_CODE(KEYCODE_Y)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T15 Matrix 7")    PORT_CODE(KEYCODE_U)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T16 Matrix 8")    PORT_CODE(KEYCODE_I)

	// PA1 first IC: CB31-IC2 (Voice presets)
	PORT_START("K2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T40 Voice Cart")  PORT_CODE(KEYCODE_A)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T41 Range Limit") PORT_CODE(KEYCODE_S)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T22 Drawbars")    PORT_CODE(KEYCODE_D)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T23 Piano")       PORT_CODE(KEYCODE_F)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T24 Stage")       PORT_CODE(KEYCODE_G)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T25 Clarinet")    PORT_CODE(KEYCODE_H)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T26 Vibes")       PORT_CODE(KEYCODE_J)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T27 Glockensp.")  PORT_CODE(KEYCODE_K)

	// PA1 chained IC: CB31-IC3 (Bass/guitar/brass)
	PORT_START("K3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T28 Bass 1")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T29 Bass 2")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T30 Guitar 1")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T31 Guitar 2")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T32 Brass 1")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T33 Brass 2")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T34 Synbrass")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T35 Horn")

	// PA2 first IC: CB31-IC8 (Orchestral + CV1-CV2)
	PORT_START("K4")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T46 Trumpet")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T47 String 1")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T48 String 2")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T49 Church")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T50 Bells")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T51 Lead")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T52 CV1")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T53 CV2")

	// PA2 chained IC: CB31-IC9 (CV3-CV10)
	PORT_START("K5")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("CV3")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("CV4")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("CV5")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("CV6")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("CV7")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("CV8")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("CV9")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("CV10")

	// PA3 first IC: CB31-IC6 (Control / voice select)
	PORT_START("K6")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T17 Left")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T18 Right")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T19 2.Voice")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T20 3.Voice")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T21 4.Voice")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T37 Preset Bank")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T38 Preset Cart")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("T39 Voice Bank")

	// PA3 chained IC: CB30-IC3 (Preset buttons on CB30)
	PORT_START("K7")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Preset 0/1")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Preset 0/2")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Preset 0/3")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Preset 0/4")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Preset 0/5")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Preset 0/6")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Preset 0/7")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Preset 0/8")

	// Drawbar sliders 1-16 (MUX 0 = IC5: DB0-DB7, MUX 1 = IC4: DB8-DB15)
	// Each is a 5k linear pot, 0-255 range, default max ($FF)
	PORT_START("DB0")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 1")  PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB1")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 2")  PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB2")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 3")  PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB3")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 4")  PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB4")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 5")  PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB5")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 6")  PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB6")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 7")  PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB7")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 8")  PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB8")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 9")  PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB9")  PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 10") PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB10") PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 11") PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB11") PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 12") PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB12") PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 13") PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB13") PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 14") PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB14") PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 15") PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
	PORT_START("DB15") PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Drawbar 16") PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)

	// Controls (MUX 2 = IC7)
	PORT_START("WHEEL1")
	PORT_BIT(0xff, 0x80, IPT_PADDLE) PORT_NAME("Wheel 1") PORT_SENSITIVITY(50) PORT_KEYDELTA(8) PORT_CENTERDELTA(20)

	PORT_START("WHEEL2")
	PORT_BIT(0xff, 0x80, IPT_PADDLE_V) PORT_NAME("Wheel 2") PORT_SENSITIVITY(50) PORT_KEYDELTA(8) PORT_CENTERDELTA(20)

	PORT_START("MAINVOL")
	PORT_BIT(0xff, 0xff, IPT_POSITIONAL) PORT_PLAYER(1) PORT_NAME("Main Volume") PORT_SENSITIVITY(100) PORT_KEYDELTA(8) PORT_CENTERDELTA(0)
INPUT_PORTS_END


//**************************************************************************
//  ROM DEFINITIONS
//**************************************************************************

ROM_START( ex20 )
	ROM_REGION( 0x8000, "maincpu", 0 )
	ROM_LOAD( "mk1_ic3_630469_w_s3_fixed.bin", 0x0000, 0x8000, CRC(b6a14562) SHA1(1d11984ac35e29c7bec29c512b646a6d0dd47235) )

	ROM_REGION( 0x4000, "voicebank", 0 )
	ROM_LOAD( "mk1_ic5_s3.bin", 0x0000, 0x4000, CRC(598b7457) SHA1(f24042ac311bf205d6a2ce95e8cfee37ceb460b8) )

	// Cartridge ROMs (CT4/B board, 27C128, 16KB each)
	// 12 banks × 16KB = 192KB contiguous region.
	// Select bank with WERSI_BANK=1..12 env var (default: 9).
	// CRTD0/CRTD1 both low = 20-sound ROM cartridge.
	ROM_REGION( 0x4000 * 12, "cartridge", ROMREGION_ERASEFF )
	ROM_LOAD( "rom1.bin",  0x00000, 0x4000, CRC(9c91705d) SHA1(d452ab8dd526dac8f0f2739e82056541e322720f) )
	ROM_LOAD( "rom2.bin",  0x04000, 0x4000, CRC(f02112d2) SHA1(e5f3d32b394727a1737cca6bba7e87924604bfb8) )
	ROM_LOAD( "rom3.bin",  0x08000, 0x4000, NO_DUMP )
	ROM_LOAD( "rom4.bin",  0x0c000, 0x4000, CRC(66ea47e4) SHA1(30811bf7bd207db2fbe2010b7e1d09539fc9dd22) )
	ROM_LOAD( "rom5.bin",  0x10000, 0x4000, CRC(6236bfed) SHA1(7c399b56d873a73fc84ffdb525c16fd4039471de) )
	ROM_LOAD( "rom6.bin",  0x14000, 0x4000, CRC(8657d0b9) SHA1(a55c9764446f230a61ec94fc7e222ce61ee422db) )
	ROM_LOAD( "rom7.bin",  0x18000, 0x4000, CRC(83031850) SHA1(bb56dc8c4c1a3f75a8499146a9ce9ce14ae40ac3) )
	ROM_LOAD( "rom8.bin",  0x1c000, 0x4000, CRC(3348fe7c) SHA1(f0a264f122079be64b654ef4cf78fe4df1e5e230) )
	ROM_LOAD( "rom9.bin",  0x20000, 0x4000, CRC(1ea8718d) SHA1(a5536bda0255546281d9b0b81979d9028a5c0d71) )
	ROM_LOAD( "rom10.bin", 0x24000, 0x4000, CRC(ed75889d) SHA1(9b933b5d2184d03d8f32ff061c97ab06e5f47fae) )
	ROM_LOAD( "rom11.bin", 0x28000, 0x4000, CRC(850fa112) SHA1(8dddcda52121ac56d84b17debc8865d3914cc092) )
	ROM_LOAD( "rom12.bin", 0x2c000, 0x4000, CRC(34d2f93c) SHA1(ec4e79b8d2876de92f701e7e60b348c68663fdd3) )
ROM_END

} // anonymous namespace


//**************************************************************************
//  GAME DRIVERS
//**************************************************************************

//    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS              INIT        COMPANY  FULLNAME                FLAGS
SYST( 1986, ex20, 0,      0,      ex20,    ex20,  wersi_ex20_state,  empty_init, "Wersi", "EX-20 Digital Organ",  MACHINE_IMPERFECT_SOUND )
