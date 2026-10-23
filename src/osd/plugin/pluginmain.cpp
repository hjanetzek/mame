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

#include "imagedev/midiin.h"

#include "drivenum.h"
#include "emuopts.h"
#include "main.h"
#include "ui/uimain.h"
#include "video.h"

#include "modules/input/input_module.h"
#include "modules/midi/midi_module.h"
#include "modules/monitor/monitor_module.h"
#include "modules/netdev/netdev_module.h"
#include "modules/output/output_module.h"
#include "modules/render/render_module.h"
#include "modules/sound/sound_module.h"

#include <algorithm>
#include <atomic>
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
void emulator_info::layout_script_cb(layout_file &file, const char *script) { }
bool emulator_info::standalone() { return true; }


namespace {

//**************************************************************************
//  MACHINE MANAGER
//**************************************************************************

class plugin_machine_manager : public machine_manager
{
public:
	plugin_machine_manager(emu_options &options, osd_interface &osd) : machine_manager(options, osd) { }

	virtual void ui_initialize(running_machine &machine) override
	{
		// the throttle option is normally applied by the frontend UI;
		// the host drives the emulation, so never throttle
		machine.video().set_throttled(false);
	}

	virtual ui_manager *create_ui(running_machine &machine) override
	{
		m_ui = std::make_unique<ui_manager>(machine);
		return m_ui.get();
	}

private:
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

struct mp_instance
{
	mp_instance(mp_config const &config, uint32_t latency)
		: io(config.sample_rate, std::size_t(latency) * 4 + config.sample_rate / 10, 65536)
		, latency(latency)
		, driver(config.driver)
		, rompath(config.rompath ? config.rompath : "")
		, datapath(config.datapath ? config.datapath : "")
	{
		if (config.options)
		{
			for (char const *const *opt = config.options; opt[0] && opt[1]; opt += 2)
				extra_options.emplace_back(opt[0], opt[1]);
		}
	}

	void emulation_thread(std::promise<std::string> started);
	void stopped();
	void notify_host();
	void set_options(osd_options &options);

	plugin_io io;
	uint32_t const latency;
	std::string const driver;
	std::string const rompath;
	std::string const datapath;
	std::vector<std::pair<std::string, std::string> > extra_options;

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

	std::string log_copy;
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
	if (!rompath.empty())
		set(OPTION_MEDIAPATH, rompath);
	if (!datapath.empty())
	{
		set(OPTION_CFG_DIRECTORY, datapath + PATH_SEPARATOR "cfg");
		set(OPTION_NVRAM_DIRECTORY, datapath + PATH_SEPARATOR "nvram");
		set(OPTION_STATE_DIRECTORY, datapath + PATH_SEPARATOR "sta");
		set(OPTION_DIFF_DIRECTORY, datapath + PATH_SEPARATOR "diff");
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

		plugin_machine_manager manager(options, osd);
		manager.start_http_server();

		machine_config const config(system, options);

		// connect every MIDI input of the machine to the host
		for (midiin_device &midiin : midiin_device_enumerator(config.root_device()))
			options.image_option(midiin.instance_name()).specify(PLUGIN_MIDI_PORT_NAME);

		running_machine machine(config, manager);
		manager.set_machine(&machine);

		if (!machine.run_start(true))
		{
			machine.run_stop();
			manager.set_machine(nullptr);
			return fail(std::string());
		}

		running = true;
		started.set_value(std::string());
		reported = true;

		while (!quit.load(std::memory_order_acquire))
		{
			std::uint32_t const seen = wake.load(std::memory_order_acquire);
			std::uint64_t const target_frame = played.load(std::memory_order_acquire) + latency;
			attotime const target = attotime::from_ticks(target_frame, io.sample_rate);
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

		stopped();
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


void mp_instance::stopped()
{
	running = false;
	emulated.store(EMULATION_STOPPED, std::memory_order_release);
	emulated.notify_one();
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
	return instance->latency;
}

void mp_process(mp_instance *instance, const mp_midi_event *events, uint32_t event_count, float *const *out, uint32_t frames)
{
	plugin_io &io = instance->io;
	std::uint64_t const base = instance->played.load(std::memory_order_relaxed);

	// MIDI is delayed by the look-ahead so it lines up with the audio
	for (uint32_t i = 0; i < event_count; i++)
	{
		std::uint64_t const due = base + std::min(events[i].frame, frames) + instance->latency;
		for (uint32_t j = 0; j < events[i].size; j++)
			io.midi.push(plugin_io::midi_byte{ due, events[i].data[j] });
	}

	// in offline mode wait until the emulation has reached the look-ahead
	// for this block; since it only ever advances to played + latency, its
	// progress and therefore the output are deterministic
	if (instance->offline.load(std::memory_order_relaxed))
	{
		std::uint64_t const needed = base + instance->latency;
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

	instance->played.store(base + frames, std::memory_order_release);
	instance->notify_host();
}

void mp_set_offline(mp_instance *instance, int offline)
{
	instance->offline = offline != 0;
}

uint64_t mp_underruns(const mp_instance *instance)
{
	return instance->underruns.load(std::memory_order_relaxed);
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

} // extern "C"
