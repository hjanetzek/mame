// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  pluginmain.cpp - C API of the MAME audio plugin library
//
//  Each instance owns an emulation thread that keeps the
//  machine a fixed number of frames ahead of the host.  The
//  host thread only exchanges data with it through lock-free
//  queues (see mame_plugin.h).
//
//============================================================

#include "emu.h"

#include "mame_plugin.h"
#include "pluginosd.h"
#include "pluginmidi.h"

#include "imagedev/midiin.h"
#include "imagedev/midiout.h"

#include "dinvram.h"
#include "drivenum.h"
#include "emuopts.h"
#include "main.h"
#include "ui/uimain.h"
#include "video.h"

#include "ioprocs.h"
#include "ioprocsvec.h"

#include "modules/input/input_module.h"
#include "modules/midi/midi_module.h"
#include "modules/monitor/monitor_module.h"
#include "modules/netdev/netdev_module.h"
#include "modules/output/output_module.h"
#include "modules/render/render_module.h"
#include "modules/sound/sound_module.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


extern const char bare_build_version[];
extern const char build_version[];


//**************************************************************************
//  EMULATOR INFO
//**************************************************************************

// the plugin has no frontend, UI or Lua engine

const char * emulator_info::get_bare_build_version() { return bare_build_version; }
const char * emulator_info::get_build_version() { return build_version; }
void emulator_info::display_ui_chooser(running_machine &machine) { }
int emulator_info::start_frontend(emu_options &options, osd_interface &osd, std::vector<std::string> &args) { return EMU_ERR_INVALID_CONFIG; }
int emulator_info::start_frontend(emu_options &options, osd_interface &osd, int argc, char *argv[]) { return EMU_ERR_INVALID_CONFIG; }
bool emulator_info::draw_user_interface(running_machine &machine) { return false; }
void emulator_info::periodic_check() { }
bool emulator_info::frame_hook() { return false; }
void emulator_info::sound_hook(const std::map<std::string, std::vector<std::pair<const float *, int>>> &sound) { }
void emulator_info::layout_script_cb(layout_file &file, const char *script) { plugin_layout_controls::add_script(script); }
bool emulator_info::standalone() { return true; }


namespace {

//**************************************************************************
//  NVRAM
//**************************************************************************

// NVRAM of all devices as one blob: "MPNV", u32 version, u32 count, then
// for each device u32 tag length, tag, u32 data length, data (little endian)

constexpr char NVRAM_MAGIC[4] = { 'M', 'P', 'N', 'V' };
constexpr u32 NVRAM_VERSION = 1;

// entries of adjusters' values: the prefix, the port's tag, ':' and the
// field's mask in hex, with the value as data
constexpr char const NVRAM_FIELD_PREFIX[] = "ioport:";

void put_u32(std::vector<u8> &out, u32 value)
{
	for (int i = 0; i < 4; i++)
		out.push_back(u8(value >> (i * 8)));
}

bool get_u32(u8 const *&ptr, u8 const *end, u32 &value)
{
	if ((end - ptr) < 4)
		return false;
	value = u32(ptr[0]) | (u32(ptr[1]) << 8) | (u32(ptr[2]) << 16) | (u32(ptr[3]) << 24);
	ptr += 4;
	return true;
}

std::vector<u8> collect_nvram(running_machine &machine)
{
	std::vector<u8> result(std::begin(NVRAM_MAGIC), std::end(NVRAM_MAGIC));
	put_u32(result, NVRAM_VERSION);
	std::size_t const count_pos = result.size();
	put_u32(result, 0);

	u32 count = 0;
	for (device_nvram_interface &nvram : nvram_interface_enumerator(machine.root_device()))
	{
		if (!nvram.nvram_can_save())
			continue;

		std::vector<u8> data;
		util::vector_read_write_adapter<u8> stream(data);
		if (!nvram.nvram_save(stream))
		{
			osd_printf_error("Error saving NVRAM of %s\n", nvram.device().tag());
			continue;
		}

		std::string_view const tag(nvram.device().tag());
		put_u32(result, u32(tag.size()));
		result.insert(result.end(), tag.begin(), tag.end());
		put_u32(result, u32(data.size()));
		result.insert(result.end(), data.begin(), data.end());
		count++;
	}

	// the adjusters' values, which MAME otherwise keeps in its cfg files
	for (auto const &port : machine.ioport().ports())
	{
		for (ioport_field &field : port.second->fields())
		{
			if (field.type() != IPT_ADJUSTER)
				continue;
			ioport_field::user_settings settings;
			field.get_user_settings(settings);
			std::string const tag = util::string_format("%s%s:%x", NVRAM_FIELD_PREFIX, port.first, field.mask());
			put_u32(result, u32(tag.size()));
			result.insert(result.end(), tag.begin(), tag.end());
			put_u32(result, 4);
			put_u32(result, u32(settings.value));
			count++;
		}
	}

	if (!count)
		return std::vector<u8>();
	for (int i = 0; i < 4; i++)
		result[count_pos + i] = u8(count >> (i * 8));
	return result;
}

void apply_nvram(running_machine &machine, std::vector<u8> const &blob)
{
	u8 const *ptr = blob.data();
	u8 const *const end = ptr + blob.size();
	u32 version, count;
	if ((blob.size() < sizeof(NVRAM_MAGIC)) || std::memcmp(ptr, NVRAM_MAGIC, sizeof(NVRAM_MAGIC)))
	{
		osd_printf_error("Ignoring NVRAM data in unknown format\n");
		return;
	}
	ptr += sizeof(NVRAM_MAGIC);
	if (!get_u32(ptr, end, version) || (version != NVRAM_VERSION) || !get_u32(ptr, end, count))
	{
		osd_printf_error("Ignoring NVRAM data of unsupported version\n");
		return;
	}

	for (u32 i = 0; i < count; i++)
	{
		u32 tag_size, data_size;
		if (!get_u32(ptr, end, tag_size) || (u32(end - ptr) < tag_size))
			break;
		std::string const tag(reinterpret_cast<char const *>(ptr), tag_size);
		ptr += tag_size;
		if (!get_u32(ptr, end, data_size) || (u32(end - ptr) < data_size))
			break;

		// an adjuster's value
		if (tag.starts_with(NVRAM_FIELD_PREFIX))
		{
			std::string::size_type const colon = tag.rfind(':');
			std::string const port_tag = tag.substr(std::strlen(NVRAM_FIELD_PREFIX), colon - std::strlen(NVRAM_FIELD_PREFIX));
			auto const found = machine.ioport().ports().find(port_tag);
			ioport_port *const port = (found != machine.ioport().ports().end()) ? found->second.get() : nullptr;
			ioport_field *const field = port ? port->field(ioport_value(std::strtoul(tag.c_str() + colon + 1, nullptr, 16))) : nullptr;
			u8 const *value_ptr = ptr;
			u32 value;
			if (field && (field->type() == IPT_ADJUSTER) && (data_size == 4) && get_u32(value_ptr, end, value))
			{
				ioport_field::user_settings settings;
				field->get_user_settings(settings);
				settings.value = ioport_value(value);
				field->set_user_settings(settings);
			}
			ptr += data_size;
			continue;
		}

		device_t *const device = machine.root_device().subdevice(tag);
		device_nvram_interface *nvram = nullptr;
		if (!device || !device->interface(nvram))
		{
			osd_printf_warning("Ignoring NVRAM data for unknown device %s\n", tag);
		}
		else
		{
			util::random_read::ptr const stream = util::ram_read(ptr, data_size);
			if (!stream || !nvram->nvram_load(*stream))
				osd_printf_error("Error loading NVRAM of %s\n", tag);
		}
		ptr += data_size;
	}
}


//**************************************************************************
//  MACHINE MANAGER
//**************************************************************************

class plugin_machine_manager : public machine_manager
{
public:
	plugin_machine_manager(emu_options &options, osd_interface &osd, std::vector<u8> const &nvram)
		: machine_manager(options, osd)
		, m_nvram(nvram)
	{
	}

	// called after NVRAM was loaded from files and before the first reset
	virtual void ui_initialize(running_machine &machine) override
	{
		// the throttle option is normally applied by the frontend UI;
		// the host drives the emulation, so never throttle
		machine.video().set_throttled(false);

		// NVRAM provided by the host replaces what was loaded from files
		if (!m_nvram.empty())
			apply_nvram(machine, m_nvram);
	}

	virtual ui_manager *create_ui(running_machine &machine) override
	{
		m_ui = std::make_unique<ui_manager>(machine);
		return m_ui.get();
	}

private:
	std::vector<u8> const &m_nvram;
	std::unique_ptr<ui_manager> m_ui;
};


//**************************************************************************
//  MACHINE LIST
//**************************************************************************

struct machine_entry
{
	char const *name;
	char const *description;
};

std::vector<machine_entry> const &midi_machines()
{
	static std::vector<machine_entry> const result =
		[] ()
		{
			std::vector<machine_entry> list;
			emu_options options;
			for (std::size_t i = 0; i < driver_list::total(); i++)
			{
				game_driver const &driver = driver_list::driver(i);
				if (&driver == &GAME_NAME(___empty))
					continue;
				machine_config const config(driver, options);
				if (midiin_device_enumerator(config.root_device()).first())
					list.push_back(machine_entry{ driver.name, driver.type.fullname() });
			}
			return list;
		}();
	return result;
}

} // anonymous namespace


//**************************************************************************
//  INSTANCE
//**************************************************************************

constexpr std::uint64_t EMULATION_STOPPED = ~std::uint64_t(0);

// largest look-ahead mp_set_latency() accepts; the audio queue holds that
// plus some slack
constexpr uint32_t MAX_LATENCY = 32768;

struct mp_instance
{
	mp_instance(mp_config const &config, uint32_t latency)
		: io(config.sample_rate, std::size_t(MAX_LATENCY) + config.sample_rate / 10, 65536)
		, latency(latency)
		, driver(config.driver)
		, rompath(config.rompath ? config.rompath : "")
		, datapath(config.datapath ? config.datapath : "")
	{
		if (config.nvram && config.nvram_size)
		{
			u8 const *const data = static_cast<u8 const *>(config.nvram);
			initial_nvram.assign(data, data + config.nvram_size);
		}
		if (config.options)
		{
			for (char const *const *opt = config.options; opt[0] && opt[1]; opt += 2)
				extra_options.emplace_back(opt[0], opt[1]);
		}
		for (uint32_t i = 0; config.note_map && (i < config.note_map_size); i++)
		{
			mp_note_mapping const &mapping = config.note_map[i];
			if ((mapping.note < 128) && mapping.input)
				io.note_map.push_back(plugin_io::note_mapping{ mapping.note, mapping.min_velocity, mapping.input });
		}
		io.reload_layout = (config.flags & MP_FLAG_RELOAD_LAYOUT) != 0;
	}

	void emulation_thread(std::promise<std::string> started);
	void serve_nvram(running_machine &machine);
	void stopped();
	void notify_host();
	void set_options(osd_options &options);

	plugin_io io;
	// look-ahead of the emulation, can change while running
	std::atomic<uint32_t> latency;
	std::string const driver;
	std::string const rompath;
	std::string const datapath;
	std::vector<std::pair<std::string, std::string> > extra_options;
	std::vector<u8> initial_nvram;

	std::thread thread;

	// frames consumed by the host, written by the host thread only
	std::atomic<std::uint64_t> played{ 0 };

	// frame the emulation has been run up to, set to EMULATION_STOPPED
	// when the emulation thread exits; the host waits on it in offline mode
	std::atomic<std::uint64_t> emulated{ 0 };

	// bumped by the host after each block and on destruction to wake the
	// emulation thread
	std::atomic<std::uint32_t> wake{ 0 };
	std::atomic<bool> quit{ false };
	std::atomic<bool> running{ false };
	std::atomic<bool> offline{ false };
	std::atomic<std::uint64_t> underruns{ 0 };

	// host thread state
	std::uint64_t deficit = 0;
	std::uint64_t block_start = 0;     // first frame of the last processed block
	std::uint64_t block_end = 0;

	std::string log_copy;

	// NVRAM snapshots requested by the host thread and taken by the
	// emulation thread between time slices
	std::atomic<bool> nvram_requested{ false };
	std::mutex nvram_mutex;
	std::condition_variable nvram_ready;
	bool nvram_done = false;
	std::vector<u8> nvram_snapshot;    // guarded by nvram_mutex
	std::vector<u8> nvram_result;      // host thread
};


void mp_instance::set_options(osd_options &options)
{
	auto const set = [&options] (char const *name, std::string const &value)
	{
		if (options.exists(name))
			options.set_value(name, value, OPTION_PRIORITY_MAXIMUM);
	};

	// headless: no video, no OS input, sound and MIDI go through plugin_io
	set(OPTION_READCONFIG, "0");
	set(OSD_MONITOR_PROVIDER, "none");
	set(OSD_RENDERER_PROVIDER, "none");
	set(OSD_SOUND_PROVIDER, "none");
	set(OSD_MIDI_PROVIDER, "none");
	set(OSD_KEYBOARDINPUT_PROVIDER, "none");
	set(OSD_MOUSEINPUT_PROVIDER, "none");
	set(OSD_LIGHTGUNINPUT_PROVIDER, "none");
	set(OSD_JOYSTICKINPUT_PROVIDER, "none");
	set(OSD_NETDEV_PROVIDER, "none");
	set(OSD_OUTPUT_PROVIDER, "none");
	set(OPTION_THROTTLE, "0");
	set(OPTION_SKIP_GAMEINFO, "1");
	set(OPTION_SAMPLERATE, std::to_string(io.sample_rate));
	set(OPTION_NVRAM_SAVE, "0");
	if (!rompath.empty())
		set(OPTION_MEDIAPATH, rompath);
	if (!datapath.empty())
	{
		set(OPTION_CFG_DIRECTORY, datapath + PATH_SEPARATOR "cfg");
		set(OPTION_NVRAM_DIRECTORY, datapath + PATH_SEPARATOR "nvram");
		set(OPTION_STATE_DIRECTORY, datapath + PATH_SEPARATOR "sta");
		set(OPTION_DIFF_DIRECTORY, datapath + PATH_SEPARATOR "diff");

		// layout files (artwork) are looked for in a folder named after
		// the machine: with <root>/<machine> as the data path, that's the
		// data path itself, e.g. <root>/tx81z/default.lay
		std::string::size_type const end = datapath.find_last_not_of(PATH_SEPARATOR);
		std::string::size_type const sep = (end == std::string::npos) ? std::string::npos : datapath.find_last_of(PATH_SEPARATOR, end);
		set(OPTION_ARTPATH, (sep == std::string::npos) ? std::string(".") : (sep ? datapath.substr(0, sep) : std::string(PATH_SEPARATOR)));
	}
	for (auto const &opt : extra_options)
		options.set_value(opt.first, opt.second, OPTION_PRIORITY_MAXIMUM);
}


void mp_instance::emulation_thread(std::promise<std::string> started)
{
	bool reported = false;
	auto const fail = [&] (std::string &&message)
	{
		if (!reported)
		{
			std::lock_guard<std::mutex> lock(io.log_mutex);
			started.set_value(message.empty() ? io.log_text : message);
			reported = true;
		}
	};

	try
	{
		osd_options options;
		plugin_osd_interface osd(options, io);
		osd.register_options();
		if (std::getenv("MAME_PLUGIN_VERBOSE"))
			osd.set_verbose(true);

		int const index = driver_list::find(driver.c_str());
		if (index < 0)
			return fail(util::string_format("Unknown machine '%s'\n", driver));
		game_driver const &system = driver_list::driver(index);

		options.set_system_name(driver);
		set_options(options);

		plugin_machine_manager manager(options, osd, initial_nvram);
		manager.start_http_server();

		machine_config const config(system, options);

		// connect every MIDI input and output of the machine to the host
		for (midiin_device &midiin : midiin_device_enumerator(config.root_device()))
			options.image_option(midiin.instance_name()).specify(PLUGIN_MIDI_PORT_NAME);
		for (midiout_device &midiout : midiout_device_enumerator(config.root_device()))
			options.image_option(midiout.instance_name()).specify(PLUGIN_MIDI_PORT_NAME);

		running_machine machine(config, manager);
		manager.set_machine(&machine);

		// the GUI must not draw lists of a machine that is gone
		struct panel_guard
		{
			plugin_panel &panel;
			~panel_guard() { panel.detach(); }
		} const guard{ io.panel };

		if (!machine.run_start(true))
		{
			io.panel.detach();
			machine.run_stop();
			manager.set_machine(nullptr);
			return fail(std::string());
		}

		osd.collect_fields();
		running = true;
		started.set_value(std::string());
		reported = true;

		while (!quit.load(std::memory_order_acquire))
		{
			std::uint32_t const seen = wake.load(std::memory_order_acquire);
			if (nvram_requested.exchange(false, std::memory_order_acq_rel))
				serve_nvram(machine);
			std::uint64_t const target_frame = played.load(std::memory_order_acquire) + latency.load(std::memory_order_relaxed);
			attotime const target = attotime::from_ticks(std::uint64_t(std::max<std::int64_t>(std::int64_t(target_frame) + io.audio_owed, 0)), io.sample_rate);
			if (machine.time() < target)
			{
				if (!machine.run_until(target))
					break;
				emulated.store(target_frame, std::memory_order_release);
				emulated.notify_one();
			}
			else
			{
				// wait for the host to consume audio
				wake.wait(seen, std::memory_order_acquire);
			}
		}

		// keep the final NVRAM contents for the host
		serve_nvram(machine);
		stopped();
		io.panel.detach();
		machine.run_stop();
		manager.set_machine(nullptr);
	}
	catch (std::exception const &ex)
	{
		stopped();
		fail(util::string_format("Exception while running machine: %s\n", ex.what()));
	}
	catch (...)
	{
		stopped();
		fail("Unknown exception while running machine\n");
	}
}


void mp_instance::serve_nvram(running_machine &machine)
{
	std::vector<u8> snapshot = collect_nvram(machine);
	std::lock_guard<std::mutex> lock(nvram_mutex);
	nvram_snapshot = std::move(snapshot);
	nvram_done = true;
	nvram_ready.notify_all();
}

void mp_instance::stopped()
{
	running = false;
	emulated.store(EMULATION_STOPPED, std::memory_order_release);
	emulated.notify_one();
	std::lock_guard<std::mutex> lock(nvram_mutex);
	nvram_ready.notify_all();
}

void mp_instance::notify_host()
{
	wake.fetch_add(1, std::memory_order_release);
	wake.notify_one();
}


//**************************************************************************
//  C API
//**************************************************************************

extern "C" {

uint32_t mp_api_version(void)
{
	return MP_API_VERSION;
}

const char *mp_build_version(void)
{
	return build_version;
}

uint32_t mp_machine_count(void)
{
	return uint32_t(midi_machines().size());
}

const char *mp_machine_name(uint32_t index)
{
	auto const &list = midi_machines();
	return (index < list.size()) ? list[index].name : nullptr;
}

const char *mp_machine_description(uint32_t index)
{
	auto const &list = midi_machines();
	return (index < list.size()) ? list[index].description : nullptr;
}

mp_instance *mp_create(const mp_config *config, char *error, size_t error_size)
{
	auto const report = [error, error_size] (std::string const &message)
	{
		if (error && error_size)
		{
			std::size_t const len = std::min(message.size(), error_size - 1);
			std::memcpy(error, message.data(), len);
			error[len] = '\0';
		}
	};

	if (!config || !config->driver || !config->sample_rate)
	{
		report("Invalid configuration\n");
		return nullptr;
	}

	// default look-ahead: one host block plus 3ms to cover the 1ms
	// sound update granularity and scheduling jitter
	uint32_t latency = config->latency_frames;
	if (!latency)
		latency = std::max<uint32_t>(config->max_block_frames, 64) + config->sample_rate * 3 / 1000;
	latency = std::min(latency, MAX_LATENCY);

	try
	{
		auto instance = std::make_unique<mp_instance>(*config, latency);
		std::promise<std::string> started;
		std::future<std::string> result = started.get_future();
		instance->thread = std::thread(&mp_instance::emulation_thread, instance.get(), std::move(started));

		std::string const message = result.get();
		if (!instance->running)
		{
			instance->thread.join();
			report(message.empty() ? std::string("Machine failed to start\n") : message);
			return nullptr;
		}
		return instance.release();
	}
	catch (std::exception const &ex)
	{
		report(util::string_format("Failed to create instance: %s\n", ex.what()));
		return nullptr;
	}
}

void mp_destroy(mp_instance *instance)
{
	if (!instance)
		return;

	instance->quit = true;
	instance->notify_host();
	if (instance->thread.joinable())
		instance->thread.join();
	delete instance;
}

uint32_t mp_latency(const mp_instance *instance)
{
	return instance->latency.load(std::memory_order_relaxed);
}

uint32_t mp_set_latency(mp_instance *instance, uint32_t frames)
{
	// output frame n is always emulated frame n, so the look-ahead only
	// decides how far the emulation runs ahead and when MIDI is due
	frames = std::min(frames, MAX_LATENCY);
	instance->latency.store(frames, std::memory_order_relaxed);
	instance->notify_host();
	return frames;
}

void mp_process(mp_instance *instance, const mp_midi_event *events, uint32_t event_count, float *const *out, uint32_t frames)
{
	plugin_io &io = instance->io;
	std::uint64_t const base = instance->played.load(std::memory_order_relaxed);
	uint32_t const latency = instance->latency.load(std::memory_order_relaxed);

	// MIDI is delayed by the look-ahead so it lines up with the audio
	for (uint32_t i = 0; i < event_count; i++)
	{
		std::uint64_t const due = base + std::min(events[i].frame, frames) + latency;
		for (uint32_t j = 0; j < events[i].size; j++)
			io.midi.push(plugin_io::midi_byte{ due, events[i].data[j] });
	}

	// in offline mode wait until the emulation has reached the look-ahead
	// for this block; since it only ever advances to played + latency, its
	// progress and therefore the output are deterministic
	if (instance->offline.load(std::memory_order_relaxed))
	{
		std::uint64_t const needed = base + latency;
		for (std::uint64_t reached = instance->emulated.load(std::memory_order_acquire); reached < needed; reached = instance->emulated.load(std::memory_order_acquire))
			instance->emulated.wait(reached, std::memory_order_acquire);
	}

	// drop audio owed from earlier underruns to keep MIDI and audio aligned
	if (instance->deficit)
		instance->deficit -= io.audio.pop(nullptr, std::size_t(instance->deficit) * 2) / 2;

	float buffer[512];
	uint32_t done = 0;
	while (done < frames)
	{
		std::size_t const want = std::min<std::size_t>(frames - done, std::size(buffer) / 2);
		std::size_t const got = io.audio.pop(buffer, want * 2) / 2;
		for (std::size_t i = 0; i < got; i++)
		{
			out[0][done + i] = buffer[i * 2];
			out[1][done + i] = buffer[i * 2 + 1];
		}
		done += got;
		if (got < want)
			break;
	}

	if (done < frames)
	{
		uint32_t const missing = frames - done;
		std::fill_n(out[0] + done, missing, 0.0f);
		std::fill_n(out[1] + done, missing, 0.0f);
		instance->deficit += missing;
		instance->underruns.fetch_add(missing, std::memory_order_relaxed);
	}

	instance->block_start = base;
	instance->block_end = base + frames;
	instance->played.store(base + frames, std::memory_order_release);
	instance->notify_host();
}

uint32_t mp_midi_output(mp_instance *instance, mp_midi_event *events, uint32_t max_events, uint8_t *buffer, size_t buffer_size)
{
	std::uint64_t dropped = 0;
	uint32_t const count = plugin_midi_record::read(
			instance->io.midi_out, instance->block_start, instance->block_end,
			events, max_events, buffer, buffer_size,
			dropped);
	if (dropped)
		instance->io.midi_out_dropped.fetch_add(dropped, std::memory_order_relaxed);
	return count;
}

void mp_set_offline(mp_instance *instance, int offline)
{
	instance->offline = offline != 0;
}

uint64_t mp_underruns(const mp_instance *instance)
{
	return instance->underruns.load(std::memory_order_relaxed);
}

const void *mp_nvram(mp_instance *instance, size_t *size)
{
	std::unique_lock<std::mutex> lock(instance->nvram_mutex);
	if (instance->running.load(std::memory_order_acquire))
	{
		// ask the emulation thread for a snapshot and wait for it
		instance->nvram_done = false;
		instance->nvram_requested.store(true, std::memory_order_release);
		instance->notify_host();
		bool const done = instance->nvram_ready.wait_for(
				lock,
				std::chrono::seconds(2),
				[instance] () { return instance->nvram_done || !instance->running.load(std::memory_order_acquire); });
		if (!done)
		{
			// this runs on the host thread, which has no OSD output
			instance->io.log(OSD_OUTPUT_CHANNEL_ERROR, "Timeout waiting for NVRAM snapshot\n");
			return nullptr;
		}
	}

	// after the machine stopped this is the final snapshot
	instance->nvram_result = instance->nvram_snapshot;
	lock.unlock();

	if (size)
		*size = instance->nvram_result.size();
	return instance->nvram_result.empty() ? nullptr : instance->nvram_result.data();
}

const char *mp_log(mp_instance *instance)
{
	std::lock_guard<std::mutex> lock(instance->io.log_mutex);
	if (instance->io.log_text.empty())
		return nullptr;
	instance->log_copy = std::move(instance->io.log_text);
	instance->io.log_text.clear();
	return instance->log_copy.c_str();
}

void mp_panel_size(mp_instance *instance, uint32_t *width, uint32_t *height)
{
	plugin_panel &panel = instance->io.panel;
	std::lock_guard<std::mutex> lock(panel.mutex);
	if (width)
		*width = panel.natural_width;
	if (height)
		*height = panel.natural_height;
}

void mp_panel_set_size(mp_instance *instance, uint32_t width, uint32_t height)
{
	plugin_panel &panel = instance->io.panel;
	std::lock_guard<std::mutex> lock(panel.mutex);
	if (!width || !height || (width > 8192) || (height > 8192))
		width = height = 0;
	panel.width = width;
	panel.height = height;
}

uint64_t mp_panel_frame(mp_instance *instance, void *pixels, uint32_t width, uint32_t height, uint64_t serial)
{
	return instance->io.panel.draw(pixels, width, height, serial);
}

static void push_pointer(mp_instance *instance, plugin_panel::pointer_event const &event)
{
	if (event.id >= 64)
		return;
	plugin_panel &panel = instance->io.panel;
	std::lock_guard<std::mutex> lock(panel.mutex);

	// a move replaces the previous one if that was a move too: an event
	// changing the buttons must stay, a drag measures from where it began
	bool const move = !event.leave && (event.buttons == panel.queued_buttons[event.id]);
	panel.queued_buttons[event.id] = event.leave ? 0 : event.buttons;
	if (move && !panel.pointers.empty())
	{
		plugin_panel::pointer_event &last = panel.pointers.back();
		if (last.move && (last.id == event.id))
		{
			last = event;
			last.move = true;
			return;
		}
	}
	if (panel.pointers.size() < plugin_panel::MAX_POINTER_EVENTS)
	{
		panel.pointers.push_back(event);
		panel.pointers.back().move = move;
	}
}

void mp_panel_pointer(mp_instance *instance, uint32_t id, int32_t x, int32_t y, uint32_t buttons)
{
	push_pointer(instance, plugin_panel::pointer_event{ id, x, y, buttons, false });
}

void mp_panel_pointer_leave(mp_instance *instance, uint32_t id)
{
	push_pointer(instance, plugin_panel::pointer_event{ id, -1, -1, 0, true });
}

uint32_t mp_field_count(const mp_instance *instance)
{
	return uint32_t(instance->io.fields.size());
}

int mp_field_info(const mp_instance *instance, uint32_t index, mp_field *field)
{
	if (index >= instance->io.fields.size() || !field)
		return 0;
	plugin_io::field_info const &info = instance->io.fields[index];
	field->name = info.name.c_str();
	field->port = info.port.c_str();
	field->mask = info.mask;
	field->min = info.min;
	field->max = info.max;
	field->default_value = info.default_value;
	return 1;
}

void mp_set_field(mp_instance *instance, uint32_t index, int32_t value)
{
	plugin_io &io = instance->io;
	std::lock_guard<std::mutex> lock(io.field_mutex);
	io.field_changes.emplace_back(index, value);
}

uint64_t mp_field_values(mp_instance *instance, uint64_t serial, int32_t *values, uint32_t count)
{
	plugin_io &io = instance->io;
	std::lock_guard<std::mutex> lock(io.field_mutex);
	if (serial == io.field_serial)
		return serial;
	if (values)
		std::copy_n(io.field_values.begin(), std::min<std::size_t>(count, io.field_values.size()), values);
	return io.field_serial;
}

uint64_t mp_layout_describe(mp_instance *instance, mp_layout_info *info, mp_layout_element *elements, uint32_t max_elements, mp_layout_item *items, uint32_t max_items)
{
	plugin_panel &panel = instance->io.panel;
	std::lock_guard<std::mutex> lock(panel.mutex);
	panel.layout_requested = true;
	if (info)
	{
		info->serial = panel.layout_serial;
		info->aspect = panel.layout_aspect;
		info->element_count = uint32_t(panel.layout_elements.size());
		info->item_count = uint32_t(panel.layout_items.size());
		info->screen_count = uint32_t(panel.layout_screens.size());
	}
	if (elements)
		std::copy_n(panel.layout_elements.begin(), std::min<std::size_t>(max_elements, panel.layout_elements.size()), elements);
	if (items)
		std::copy_n(panel.layout_items.begin(), std::min<std::size_t>(max_items, panel.layout_items.size()), items);
	return panel.layout_serial;
}

uint64_t mp_layout_states(mp_instance *instance, uint64_t serial, mp_layout_item_state *states, uint32_t max_states, uint64_t *layout_serial)
{
	plugin_panel &panel = instance->io.panel;
	std::lock_guard<std::mutex> lock(panel.mutex);
	if (layout_serial)
		*layout_serial = panel.layout_serial;
	if ((serial == panel.layout_states_serial) || panel.layout_states.empty())
		return serial;
	if (states)
		std::copy_n(panel.layout_states.begin(), std::min<std::size_t>(max_states, panel.layout_states.size()), states);
	return panel.layout_states_serial;
}

int mp_layout_element_image(mp_instance *instance, uint32_t element, uint32_t state, uint32_t orientation, uint32_t width, uint32_t height, void *rgba)
{
	return instance->io.panel.draw_element(element, state, orientation, width, height, rgba) ? 1 : 0;
}

void mp_layout_pointer(mp_instance *instance, uint32_t id, float x, float y, uint32_t buttons)
{
	plugin_panel::pointer_event event{ id, 0, 0, buttons, false };
	event.view = true;
	event.vx = x;
	event.vy = y;
	push_pointer(instance, event);
}

void mp_layout_input(mp_instance *instance, uint64_t layout_serial, uint32_t item, int pressed)
{
	plugin_panel &panel = instance->io.panel;
	std::lock_guard<std::mutex> lock(panel.mutex);
	if (panel.layout_inputs.size() < plugin_panel::MAX_POINTER_EVENTS)
		panel.layout_inputs.push_back(plugin_panel::layout_input_event{ layout_serial, item, pressed != 0 });
}

uint64_t mp_layout_screen(mp_instance *instance, uint32_t screen, uint64_t serial, uint32_t *width, uint32_t *height, void *rgba, size_t size)
{
	plugin_panel &panel = instance->io.panel;
	std::lock_guard<std::mutex> lock(panel.mutex);
	if (screen >= panel.layout_screens.size())
		return serial;
	plugin_panel::layout_screen const &shared = panel.layout_screens[screen];
	if (width)
		*width = shared.width;
	if (height)
		*height = shared.height;
	if ((serial == shared.serial) || !shared.serial || !rgba || (size < shared.rgba.size()))
		return serial;
	std::copy(shared.rgba.begin(), shared.rgba.end(), reinterpret_cast<uint8_t *>(rgba));
	return shared.serial;
}

} // extern "C"
