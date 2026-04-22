// license:BSD-3-Clause
// copyright-holders:Jeff
/***************************************************************************

    wersi_slm2.cpp

    Wersi SLM-2 slave voice card — single Z8611 voice module.

    Hardware per voice module:
    - Z8611 (SR0106) at 12 MHz, 4KB internal ROM
    - 256 bytes external slave RAM (address $F800+ from Z8)
    - DAC 0832 (8-bit) on Port 0, reference = envelope from co-processor
    - RAUD (pin 5 = P3.0) triggers IRQ3 — "start processing" from master
    - ECLK (pin 30 = P3.3) polled by firmware — envelope timing
    - Port 2: P2.0 = DS/AS enable (active low), P2.2 = RARC output
    - Port 2 bits 4-7: audio routing, bit 3: bright filter select
    - Port 3: P3.1 = EXSLA0 input, P3.2 = EXSLA1 input

***************************************************************************/

#include "emu.h"
#include "wersi_slm2.h"


// device type definition
DEFINE_DEVICE_TYPE(WERSI_SLM2_VOICE, wersi_slm2_device, "wersi_slm2_voice", "Wersi SLM-2 Voice Module")

//#define NO_AMP_ENV

//**************************************************************************
//  ROM DEFINITION
//**************************************************************************

ROM_START( wersi_slm2_voice )
	ROM_REGION( 0x1000, "z8cpu", 0 )
	ROM_LOAD( "sr0106.bin", 0x0000, 0x1000, CRC(fc6170d3) SHA1(874088ea44c1fc773919737beb49a55a8c3f74b9) )
ROM_END


//**************************************************************************
//  ADDRESS MAPS
//**************************************************************************

// External data memory map (accessed by Z8 via LDE/LDEI through Port 1)
// The Z8611 uses Port 0/1 in address/data bus mode (P01M=$1C)
// External RAM is at the high end of the 16-bit address space
void wersi_slm2_device::z8_data_map(address_map &map)
{
	// External RAM — the Z8 accesses it via LDE/LDEI through Port 1
	// in address/data bus mode. The actual address seen depends on the
	// P01M configuration and what the firmware puts on the bus.
	// Map the full 64K space to the 256-byte slave RAM (mirrored).
	map(0x0000, 0xffff).r(FUNC(wersi_slm2_device::ext_ram_r));
}


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  constructor
//-------------------------------------------------

wersi_slm2_device::wersi_slm2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, WERSI_SLM2_VOICE, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_cpu(*this, "z8cpu")
	, m_stream(nullptr)
	, m_rarc_cb(*this)
	, m_bright_cb(*this)
	, m_port0_data(0x80)
	, m_port2_data(0)
	, m_port2_mask(0)
	, m_exsla0(0)
	, m_exsla1(0)
	, m_port3_out(0)
	, m_rarc_input(1)
	, m_eclk(0)
	, m_dac_data(0)
	, m_dac_output(0)
	, m_envelope(0xfff)
	, m_envelope_smooth(0.0f) {}


//-------------------------------------------------
//  device_rom_region
//-------------------------------------------------

const tiny_rom_entry *wersi_slm2_device::device_rom_region() const
{
	return ROM_NAME( wersi_slm2_voice );
}


//-------------------------------------------------
//  device_add_mconfig — instantiate the Z8 CPU
//-------------------------------------------------

void wersi_slm2_device::device_add_mconfig(machine_config &config)
{
	// Z8611 with internal 4KB ROM, running at 12 MHz
	// Z8601 only has 2KB ROM — must use Z8611 for 4KB
	Z8611(config, m_cpu, DERIVED_CLOCK(1, 1));
	m_cpu->set_addrmap(AS_DATA, &wersi_slm2_device::z8_data_map);

	// Port callbacks
	m_cpu->p0_in_cb().set(FUNC(wersi_slm2_device::port0_r));
	m_cpu->p0_out_cb().set(FUNC(wersi_slm2_device::port0_w));
	m_cpu->p1_in_cb().set(FUNC(wersi_slm2_device::port1_r));
	m_cpu->p1_out_cb().set(FUNC(wersi_slm2_device::port1_w));
	m_cpu->p2_in_cb().set(FUNC(wersi_slm2_device::port2_r));
	m_cpu->p2_out_cb().set(FUNC(wersi_slm2_device::port2_w));
	m_cpu->p3_in_cb().set(FUNC(wersi_slm2_device::port3_r));
	m_cpu->p3_out_cb().set(FUNC(wersi_slm2_device::port3_w));
}


//-------------------------------------------------
//  device_start
//-------------------------------------------------

void wersi_slm2_device::device_start()
{
	// Create sound stream — mono output
	// Max sample rate: 50 kHz (sub 5+, prescaler=1, TIMER_VAL=$1E=30)
	m_stream = stream_alloc(0, 1, clock() / 240);
	//m_stream = stream_alloc(0, 1, clock() / 1024);

	// Register for save states
	//save_item(NAME(m_slave_ram));
	save_item(NAME(m_port0_data));
	save_item(NAME(m_port2_data));
	save_item(NAME(m_port2_mask));
	save_item(NAME(m_exsla0));
	save_item(NAME(m_exsla1));
	save_item(NAME(m_dac_output));

	// DAC trace capture — env WERSI_DAC_TRACE sets path
	// Format: binary records, each 16 bytes:
	//   u32 seq, u16 pc, u8 event_type, u8 value, u8 port2, u8 port3_out,
	//   u8 bank, u8 exsla0, u8 exsla1, u8 pad, u16 pad2
	// Event types: 'D'=DAC write, 'L'=latch (T_OUT edge), 'R'=RAUD, 'I'=IRQ
	m_trace_seq = 0;
	const char *trace_path = getenv("WERSI_DAC_TRACE");
	if (trace_path)
	{
		// Tag filter: WERSI_TRACE_TAG=voice4 → only trace that voice
		const char *trace_tag = getenv("WERSI_TRACE_TAG");
		bool tag_match = !trace_tag || std::string(tag()).find(trace_tag) != std::string::npos;
		if (tag_match)
		{
			m_dac_trace = fopen(trace_path, "wb");
			if (m_dac_trace)
			{
				const char hdr[8] = {'W','D','A','C', 1,0,0,0};
				fwrite(hdr, 1, 8, m_dac_trace);
				logerror("%s: DAC trace → %s (tag filter: %s)\n", tag(), trace_path, trace_tag ? trace_tag : "none");
			}
		}
		else
		{
			logerror("%s: DAC trace skipped (tag filter '%s' doesn't match '%s')\n", tag(), trace_tag, tag());
		}
	}
}


void wersi_slm2_device::trace_event(uint8_t event_type, uint8_t value)
{
	if (!m_dac_trace) return;
	// 16-byte record:
	//   u64 cycles (Z8 total_cycles, little-endian)
	//   u16 pc
	//   u8  event_type: 'D'=DAC write, 'L'=latch (T_OUT), 'R'=RAUD, 'S'=synth
	//   u8  value (DAC byte for D/L, cmd byte for R, reg[$1D] for S)
	//   u8  port0 (current DAC output)
	//   u8  port2
	//   u8  bank (for S: reg[$1B] = micro-program counter)
	//   u8  flags (for S: current opcode at reg[$1B])
	uint8_t rec[16] = {};
	uint64_t cycles = m_cpu->total_cycles();
	uint16_t pc = m_cpu->pc();
	for (int i = 0; i < 8; i++)
		rec[i] = (uint8_t)(cycles >> (i * 8));
	rec[8] = pc & 0xFF;
	rec[9] = (pc >> 8) & 0xFF;
	rec[10] = event_type;
	rec[11] = value;
	rec[12] = m_port0_data;
	rec[13] = m_port2_data;
	rec[14] = m_shared_ram_bank;
	rec[15] = m_exsla0 | (m_exsla1 << 1) | (m_eclk << 2);
	fwrite(rec, 1, 16, m_dac_trace);
}

//-------------------------------------------------
//  device_reset
//-------------------------------------------------

void wersi_slm2_device::device_reset()
{
	m_port0_data = 0x80;  // center value (silence for unsigned DAC)
	m_port2_data = 0;
	m_port2_mask = 0;     // P2M resets to $FF (all input) → mask = ~$FF = $00
	m_dac_output = 0;

	// Z8 trace callback — fires for DAC binary trace (WERSI_DAC_TRACE) and/or
	// CSV instruction trace (WERSI_Z8_TRACE)
	FILE *trace_fp = nullptr;
	if (getenv("WERSI_Z8_TRACE"))
	{
		const char *trace_tag = getenv("WERSI_TRACE_TAG");
		bool tag_match = !trace_tag || std::string(tag()).find(trace_tag) != std::string::npos;
		if (tag_match)
		{
		trace_fp = fopen(getenv("WERSI_Z8_TRACE"), "w");
		}
		if (trace_fp)
		{
			fprintf(trace_fp, "# Z8 instruction trace\n");
			fprintf(trace_fp, "# pc,R0,R4:R5,R6,R7,R8:R9,R14:R15,reg1B,reg1D,reg1E,FLAGS\n");
			logerror("%s: Z8 trace → %s\n", tag(), getenv("WERSI_Z8_TRACE"));
		}
	}
	if (trace_fp || m_dac_trace)
	{
		auto prev_r14 = std::make_shared<uint8_t>(uint8_t(0xFF));
		auto prev_r40 = std::make_shared<uint8_t>(uint8_t(0x00));
		auto prev_r1e = std::make_shared<uint8_t>(uint8_t(0x00));
		auto prev_r13 = std::make_shared<uint8_t>(uint8_t(0x00));
		auto prev_r1b = std::make_shared<uint8_t>(uint8_t(0x00));
		auto prev_r1c = std::make_shared<uint8_t>(uint8_t(0x00));
		auto prev_r1d = std::make_shared<uint8_t>(uint8_t(0x00));

		m_cpu->set_trace_callback([this, trace_fp, prev_r14, prev_r40, prev_r1e, prev_r13, prev_r1b, prev_r1c, prev_r1d](uint16_t pc, uint16_t next_pc) {
				// Synthesis trace to DAC binary trace (event 'S')
				// Fire when PC hits synthesis_output_common ($0E94) where
				// reg[$1D] and R6 are written — this is the pitch update point
				if (m_dac_trace && (pc == 0x0E9E || pc == 0x058E
					|| pc == 0x00CF || pc == 0x00DA || pc == 0x00E9 || pc == 0x00F2))
				{
					// 32-byte synthesis event with all key registers
					uint8_t rec[32] = {};
					uint64_t cycles = m_cpu->total_cycles();
					for (int i = 0; i < 8; i++)
						rec[i] = (uint8_t)(cycles >> (i * 8));
					rec[8] = pc & 0xFF;
					rec[9] = (pc >> 8) & 0xFF;
					rec[10] = 'S';
					rec[11] = m_cpu->read_register(0x1D);  // reg[$1D] = TIMER_VAL
					rec[12] = m_cpu->read_register(0x1B);  // reg[$1B] = prog counter
					rec[13] = m_cpu->read_register(0x1E);  // reg[$1E] = micro-op chain
					rec[14] = m_cpu->read_register(0x08);  // R8 = ACC high
					rec[15] = m_cpu->read_register(0x09);  // R9 = ACC low
					rec[16] = m_cpu->read_register(0x0D);  // R13 = table pointer
					rec[17] = m_cpu->read_register(0x0E);  // R14 = phase step / freq step
					rec[18] = m_cpu->read_register(0x0A);  // R10 = finalize result
					rec[19] = (m_cpu->read_register(0x1B) >= 0x1D && m_cpu->read_register(0x1B) <= 0x3C)
						? m_cpu->read_register(m_cpu->read_register(0x1B)) : 0;  // opcode
					rec[20] = m_cpu->read_register(0x05);  // R5 = current micro-op dispatch
					rec[21] = m_cpu->read_register(0x07);  // R7 = phase / waveform index
					rec[22] = m_cpu->read_register(0x10);  // reg[$10] = mode byte
					rec[23] = m_cpu->read_register(0x0F);  // R15
					rec[24] = m_cpu->read_register(0xFE);  // SPH = dither pattern
					rec[25] = m_cpu->read_prescaler(0);      // PRE0 (write-only SFR, read from CPU shadow)
					rec[26] = m_cpu->read_register(0x06);  // R6 = TIMER_LOAD
					rec[27] = m_cpu->read_register(0x40);  // reg[$40] = mul_hi (saved R10 from stub at $0FAC)
					rec[28] = m_cpu->read_register(0x41);  // reg[$41] = mul_lo (saved R11 from stub at $0FAC)
					rec[29] = m_cpu->read_register(0x12);  // reg[$12] = COEFF_HI (PITCH_HI)
					rec[30] = m_cpu->read_register(0x13);  // reg[$13] = COEFF_LO (PITCH_LO)
					rec[31] = 0;                            // padding
					fwrite(rec, 1, 32, m_dac_trace);
				}

				// Unmapped register read detection — LD Rn, 40h(R7) with R7 >= 0x40
				// addresses 0x80-0xBF are unmapped on Z8611 (returns 0xFF)
				// $00C4: micro_dds_waveform  $00C0: micro_chain_a_step2
				// $00A8: micro_interp_dds_step4  $0E79: synthesis_output.interp_prime
				if (pc == 0x00C4 || pc == 0x00C0 || pc == 0x00A8 || pc == 0x0E79)
				{
					uint8_t r7 = m_cpu->read_register(0x07);
					if (r7 >= 0x40)
					{
						uint8_t r5 = m_cpu->read_register(0x05);
						uint8_t r1e = m_cpu->read_register(0x1E);
						uint8_t r10_mode = m_cpu->read_register(0x10);
						logerror("UNMAP_REG: pc=%04X R7=%02X addr=%02X R5=%02X reg1E=%02X mode=%02X\n",
							pc, r7, 0x40 + r7, r5, r1e, r10_mode);
						if (m_dac_trace)
						{
							// 'U' event: unmapped register read
							uint8_t rec[16] = {};
							uint64_t cycles = m_cpu->total_cycles();
							for (int i = 0; i < 8; i++)
								rec[i] = (uint8_t)(cycles >> (i * 8));
							rec[8] = pc & 0xFF;
							rec[9] = (pc >> 8) & 0xFF;
							rec[10] = 'U';           // event type
							rec[11] = 0x40 + r7;     // unmapped address
							rec[12] = r5;             // current micro-op (R5)
							rec[13] = r1e;            // reg[$1E] micro-op chain
							rec[14] = r7;             // R7 raw value
							rec[15] = r10_mode;       // reg[$10] mode byte
							fwrite(rec, 1, 16, m_dac_trace);
						}
					}
				}

				// reg[$1E] change detection → 'E' event in binary trace
				{
					uint8_t r1e = m_cpu->read_register(0x1E);
					if (r1e != *prev_r1e)
					{
						if (m_dac_trace)
						{
							// 'E' event: reg[$1E] changed — who wrote it?
							// 24 bytes: header + new/old + R5/R7 + 4 previous PCs (path to writer)
							uint8_t rec[24] = {};
							uint64_t cycles = m_cpu->total_cycles();
							for (int i = 0; i < 8; i++)
								rec[i] = (uint8_t)(cycles >> (i * 8));
							rec[8] = pc & 0xFF;
							rec[9] = (pc >> 8) & 0xFF;
							rec[10] = 'E';            // event type
							rec[11] = r1e;             // new reg[$1E] value
							rec[12] = *prev_r1e;       // old reg[$1E] value
							rec[13] = m_cpu->read_register(0x05);  // R5
							rec[14] = m_cpu->read_register(0x07);  // R7
							rec[15] = m_cpu->read_register(0x10);  // reg[$10] mode
							// Previous PCs from ring buffer: age 1-4 (age 0 = pc itself)
							for (int i = 0; i < 4; i++) {
								uint16_t prev = m_cpu->read_pc_trace(i + 1);
								rec[16 + i*2] = prev & 0xFF;
								rec[17 + i*2] = (prev >> 8) & 0xFF;
							}
							fwrite(rec, 1, 24, m_dac_trace);
						}
						*prev_r1e = r1e;
					}
				}

				// === CSV trace: register change logging ===
				if (trace_fp)
				{
					uint8_t r14 = m_cpu->read_register(0x0E);
					uint8_t r40 = m_cpu->read_register(0x40);

					if (r14 != *prev_r14)
					{
						uint8_t r0  = m_cpu->read_register(0x00);
						uint8_t r7  = m_cpu->read_register(0x07);
						uint8_t r1e = m_cpu->read_register(0x1E);
						fprintf(trace_fp, "R14_CHG: pc=%04X prev=%02X now=%02X R0=%02X R7=%02X reg1E=%02X\n",
							pc, *prev_r14, r14, r0, r7, r1e);
						*prev_r14 = r14;
					}

					if (r40 != *prev_r40)
					{
						fprintf(trace_fp, "REG40_CHG: pc=%04X prev=%02X now=%02X\n", pc, *prev_r40, r40);
						*prev_r40 = r40;
					}

					uint8_t r1e_now = m_cpu->read_register(0x1E);
					if (r1e_now != *prev_r1e)
					{
						uint8_t r7  = m_cpu->read_register(0x07);
						uint8_t r5  = m_cpu->read_register(0x05);
						fprintf(trace_fp, "REG1E_CHG: pc=%04X prev=%02X now=%02X R5=%02X R7=%02X\n",
							pc, *prev_r1e, r1e_now, r5, r7);
						*prev_r1e = r1e_now;
					}

					uint8_t r13 = m_cpu->read_register(0x0D);
					if (r13 != *prev_r13)
					{
						fprintf(trace_fp, "R13_CHG: pc=%04X prev=%02X now=%02X R12=%02X\n",
							pc, *prev_r13, r13, m_cpu->read_register(0x0C));
						*prev_r13 = r13;
					}

					uint8_t r1b_cur = m_cpu->read_register(0x1B);
					if (r1b_cur != *prev_r1b)
					{
						fprintf(trace_fp, "R1B_CHG: pc=%04X prev=%02X now=%02X R13=%02X\n",
							pc, *prev_r1b, r1b_cur, r13);
						*prev_r1b = r1b_cur;
					}

					uint8_t r1c = m_cpu->read_register(0x1C);
					if (r1c != *prev_r1c)
					{
						fprintf(trace_fp, "R1C_CHG: pc=%04X prev=%02X now=%02X R1B=%02X\n",
							pc, *prev_r1c, r1c, r1b_cur);
						*prev_r1c = r1c;
					}

					uint8_t r1d_cur = m_cpu->read_register(0x1D);
					if (r1d_cur != *prev_r1d)
					{
						fprintf(trace_fp, "R1D_CHG: pc=%04X prev=%02X now=%02X R10=%02X R1B=%02X\n",
							pc, *prev_r1d, r1d_cur, m_cpu->read_register(0x0A), r1b_cur);
						*prev_r1d = r1d_cur;
					}

					// Per-instruction trace at key addresses
					bool trace = (pc >= 0x0019 && pc <= 0x00FF) ||  // micro-ops
					             (pc >= 0x058E && pc <= 0x05BA) ||  // interpreter
					             (pc >= 0x0947 && pc <= 0x09C7) ||  // branch dispatch + computed jump
					             (pc >= 0x09C8 && pc <= 0x0A55) ||  // loop control
					             (pc >= 0x0CA5 && pc <= 0x0CCC) ||  // finalize_output
					             (pc >= 0x0DED && pc <= 0x0E28) ||  // envelope/mode dispatch
					             (pc >= 0x0E2A && pc <= 0x0E61) ||  // synthesis_loop_reentry
					             (pc >= 0x0E62 && pc <= 0x0EBE) ||  // synthesis_output
					             pc == 0x03A5 || pc == 0x0330 ||    // full_setup, update_path
					             pc == 0x0374 || pc == 0x0389 ||    // voice_param_update, stop
					             pc == 0x02E0;
					if (trace)
					{
						uint8_t r0  = m_cpu->read_register(0x00);
						uint8_t r4  = m_cpu->read_register(0x04);
						uint8_t r5  = m_cpu->read_register(0x05);
						uint8_t r6  = m_cpu->read_register(0x06);
						uint8_t r7  = m_cpu->read_register(0x07);
						uint8_t r8  = m_cpu->read_register(0x08);
						uint8_t r9  = m_cpu->read_register(0x09);
						uint8_t r15 = m_cpu->read_register(0x0F);
						uint8_t r1b = m_cpu->read_register(0x1B);
						uint8_t r1d = m_cpu->read_register(0x1D);
						uint8_t r1e = m_cpu->read_register(0x1E);
						uint8_t flags = m_cpu->read_flags();
						fprintf(trace_fp, "%04X,%02X,%02X:%02X,%02X,%02X,%02X:%02X,%02X:%02X,%02X,%02X,%02X,%02X\n",
							pc, r0, r4, r5, r6, r7, r8, r9, r14, r15, r1b, r1d, r1e, flags);
					}
				}
			});
		}
}


//-------------------------------------------------
//  sound_stream_update — output the current
//  waveform sample from the Z8's Port 0
//-------------------------------------------------

void wersi_slm2_device::sound_stream_update(sound_stream &stream)
{
	// Real hardware: DAC 0832 output = waveform × envelope (VREF).
	// Envelope is smoothed by R24=1k + C3=2.2µF (τ=2.2ms) on the SLM-2 board.
	// Per-sample exponential smoothing: α = 1 - exp(-1/(fs × τ))
	constexpr sound_stream::sample_t sample_scale = 1.0 / 128.0;
	sound_stream::sample_t waveform = sound_stream::sample_t(int(m_dac_data) - 128) * sample_scale;

#ifdef NO_AMP_ENV
	stream.fill(0, waveform);
#else
	float env_target = float(m_envelope) / 4095.0f;

	// α from RC time constant: τ = R24×C3 = 1k×2.2µF = 2.2ms
	// α = 1 - exp(-1/(fs×τ)), precomputed for fs = sample_rate()
    float alpha = 1.0f - expf(-1.0f / (float(stream.sample_rate()) * 0.0022f));

	for (int i = 0; i < stream.samples(); i++)
	{
		m_envelope_smooth += (env_target - m_envelope_smooth) * alpha;
		stream.put(0, i, waveform * m_envelope_smooth);
	}
#endif
}


//**************************************************************************
//  EXTERNAL INTERFACE
//**************************************************************************

//-------------------------------------------------
//  raud_w — RAM Access Update (from master)
//  Pin 5 = P3.0 = external interrupt → triggers IRQ3
//  This is the "start processing" signal from the master
//  after writing new voice parameters to slave RAM.
//-------------------------------------------------

void wersi_slm2_device::raud_w(int state)
{
	if (state)
		trace_event('R', m_shared_ram ? m_shared_ram[m_shared_ram_bank * 256 + 0xF8] : 0);
	m_cpu->set_input_line(INPUT_LINE_IRQ3, state ? ASSERT_LINE : CLEAR_LINE);
}


//-------------------------------------------------
//  eclk_w — Envelope Clock (from co-processor)
//  Pin 30 = P3.3 = port input (NOT interrupt!)
//  Not yet found read in firmware, but wired.
//-------------------------------------------------

void wersi_slm2_device::eclk_w(int state)
{
	// FIXME line should be cleared directly when dispatching mux is deselected - how to capture this?
	// FIXME do we need to clear the line when the firmware clears it - or can a clear at the wrong time cause problems?
	// ECLK goes to P3.3 (pin 30) which is the Z8's IRQ1 external interrupt input.
	// The co-processor generates per-voice ECLK pulses at ~200 Hz to synchronize
	// envelope processing. The Z8 firmware waits for IRQ1 pending at $0E2A
	// (TCM IRQ, #$02) before outputting synthesis results. Without ECLK triggering
	// IRQ1, the synthesis loop is stuck and no sound is produced.
	m_eclk = state ? 1 : 0;
	//m_cpu->set_input_line(INPUT_LINE_IRQ1, state ? ASSERT_LINE : CLEAR_LINE);
	if (state) {
		m_cpu->set_input_line(INPUT_LINE_IRQ1, ASSERT_LINE);
		m_cpu->set_input_line(INPUT_LINE_IRQ1, CLEAR_LINE);
	}
}

//-------------------------------------------------
//  set_exsla — pitch exponent bits
//-------------------------------------------------

void wersi_slm2_device::set_exsla(uint8_t exsla0, uint8_t exsla1)
{
	m_exsla0 = exsla0;
	m_exsla1 = exsla1;
}


//**************************************************************************
//  Z8 PORT CALLBACKS
//**************************************************************************

//-------------------------------------------------
//  Port 0: waveform output to DAC
//-------------------------------------------------

uint8_t wersi_slm2_device::port0_r()
{
	return m_port0_data;
}

void wersi_slm2_device::port0_w(uint8_t data)
{
	trace_event('D', data);
	m_port0_data = data;
	m_dac_output = int(data) - 128;
}

//-------------------------------------------------
//  Port 1: multiplexed address/data bus
//  (handled by the Z8 external memory interface,
//   but we provide callbacks for non-bus mode)
//-------------------------------------------------

uint8_t wersi_slm2_device::port1_r()
{
	// In non-bus mode, Port 1 is an input port
	// Returns bus data (slave RAM data bus)
	uint8_t val = 0xff;
	if (m_shared_ram)
	 	val = m_shared_ram[m_shared_ram_bank * 256 + 0xff];

	return val;
}

void wersi_slm2_device::port1_w(uint8_t data)
{
	// Port 1 output (memory bus)
	printf("port1_w %02X\n", data);
	abort();
}

//-------------------------------------------------
//  Port 2: P2.0 = DS/AS enable (active low for IC7)
//          P2.2 = RARC output (active low = Z8 claims bus)
//          P2.3 = Bright filter select
//          P2.4-P2.7 = audio routing MUX
//
//  P2M controls direction (1=input, 0=output):
//    Init P2M=$0C: P2.2+P2.3 = input, P2.0+P2.1 = output
//    IRQ3 P2M=$00: all output (P2.2 drives RARC)
//
//  When P2.2 is input (idle, P2M=$0C), the Z8 can read
//  the RARC bus state to check if master is blocking access.
//  When P2.2 is output (IRQ3, P2M=$00), the Z8 drives RARC
//  to signal bus claim/release.
//-------------------------------------------------

uint8_t wersi_slm2_device::port2_r()
{
	// Called by Z8 core for input-configured pins (P2M bit = 1).
	// P2.2 as input: reads RARC bus state from master side (IC34B flip-flop).
	// RARC low = master has claimed slave RAM bus (SLRAM active at $0000-$00FF).
	// RARC high = bus free for slave access.
	// Set by ex20 driver: slave_ram_r/w → RARC low, crar_w → RARC high.
	uint8_t data = m_rarc_input ? 0x04 : 0x00;
	return data;
}

void wersi_slm2_device::port2_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	// Called by Z8 core for output-configured pins (P2M bit = 0).
	// mem_mask indicates which pins are outputs (~P2M).
	// Data is masked by Z8 core — only output pins have valid bits.
	//
	// P2.0: DS/AS enable (active low — 0 = IC7 OE enabled → Z8 drives bus)
	//   Firmware XOR R2, #$07 toggles between:
	//   R2=$05 (idle): P2.0=1 → IC7 disabled → DS/AS tri-stated → bus released
	//   R2=$02 (active): P2.0=0 → IC7 enabled → DS/AS driven → Z8 on bus
	//
	// P2.2: RARC — RAM Access Ready & Clear
	//   IC34A (74LS74) flip-flop: D=VCC, CLK=RARC
	//   Rising edge of RARC (P2.2: 0→1) → Q=1 → SLIRQ to master
	//   This happens when Z8 FINISHES bus access and releases the bus.
	//   Firmware: XOR R2 → $02 (RARC low, bus claimed) ... process ...
	//             XOR R2 → $05 (RARC high, bus released → SLIRQ)

	uint8_t old = m_port2_data;
	// bool old_rarc_output = BIT(m_port2_mask, 2);  // was P2.2 an output before?
	m_port2_data = data;
	m_port2_mask = mem_mask;

	// RARC (P2.2): detect effective state changes on the wired-OR bus.
	// When P2.2 is output (mem_mask bit 2 = 1): driven value = BIT(data, 2)
	// When P2.2 is input/sense (mem_mask bit 2 = 0): pin is high-Z → pull-up → 1
	if (BIT(mem_mask, 2)) {
		m_rarc_cb( BIT(data, 2));

	}
	// Bright (P2.3): output filter select via IC28 (4053 analog switch)
	// 1 = bright (wider bandwidth), 0 = normal (R28+C31 lowpass ~720 Hz)
	if (BIT(old, 3) != BIT(data, 3))
		m_bright_cb(BIT(data, 3));
}

//-------------------------------------------------
//  Port 3: input signals and timer I/O
//    Pin 5  (P3.0): RAUD — IRQ3 input (directly handled by CPU)
//    Pin 39 (P3.1): EXSLA0 — pitch exponent bit 0 (from SLRAMB latch)
//    Pin 12 (P3.2): EXSLA1 — pitch exponent bit 1 (from SLRAMB latch)
//    Pin 30 (P3.3): ECLK — envelope clock from co-processor
//    Pin 10 (P3.5): T_OUT — timer output (DAC strobe)
//-------------------------------------------------

uint8_t wersi_slm2_device::port3_r()
{
	// Return 0xFF for all bits, then clear the ones that should be 0.
	// The Z8 core ANDs this with m_input[3] (which tracks IRQ pin state).
	// Bits we drive: P3.1 (EXSLA0), P3.2 (EXSLA1), P3.3 (ECLK/IRQ1).
	// All other bits default high (pull-ups from P3M bit 0 = active pull-ups).
	uint8_t data = 0x00;
	if (m_exsla0)
		data |= 0x02;  // P3.1 = bit 1
	if (m_exsla1)
		data |= 0x04;  // P3.2 = bit 2

	return data;
}

void wersi_slm2_device::port3_w(uint8_t data)
{
	// P3.6 (bit 6): Timer T0 output (T_OUT) → IC5 XOR gate → DAC 0832 strobe.
	// The XOR gate shapes T_OUT edges into constant-width DAC latch pulses.
	// Detect T_OUT edge to capture the DAC sample at the correct time.
	if (BIT(data ^ m_port3_out, 6)) {
		m_dac_data = m_port0_data;
		m_stream->update();
		trace_event('L', m_port0_data);  // L=latch: DAC captures this sample
	}
	m_port3_out = data;
}


//**************************************************************************
//  EXTERNAL RAM ACCESS (from Z8 LDE/LDEI)
//**************************************************************************

uint8_t wersi_slm2_device::ext_ram_r(offs_t offset)
{
	if (!BIT(m_port2_mask, 2) || BIT(m_port2_data, 2)) {
		// Not owning the bus
		abort();
	}

	uint8_t val = 0xff;
	if (m_shared_ram)
		val = m_shared_ram[m_shared_ram_bank * 256 + (offset & 0xff)];

	return val;
}

void wersi_slm2_device::ext_ram_w(offs_t offset, uint8_t data)
{
	abort();
}
