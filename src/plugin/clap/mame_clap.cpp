// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  mame_clap.cpp - CLAP instrument plugin running a MAME
//  machine through the mp_* API (src/osd/plugin/mame_plugin.h)
//
//  The machine is selected with an enumerated parameter;
//  changing it restarts the plugin.  ROM and data paths come
//  from the environment or a configuration file, see
//  load_settings().  The plugin state holds the machine and its
//  NVRAM (e.g. patch memory); NVRAM files in the data path only
//  provide the initial contents for new instances.
//
//============================================================

#include "mame_plugin.h"

#include <clap/clap.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>


namespace {

//**************************************************************************
//  SETTINGS
//**************************************************************************

struct settings
{
	std::string rompath;
	std::string datapath;
	std::string machine;
};

std::string trim(std::string_view text)
{
	auto const first = text.find_first_not_of(" \t\r\n");
	if (first == std::string_view::npos)
		return std::string();
	auto const last = text.find_last_not_of(" \t\r\n");
	return std::string(text.substr(first, last - first + 1));
}

std::string env_or(char const *name, std::string fallback)
{
	char const *const value = std::getenv(name);
	return (value && *value) ? std::string(value) : std::move(fallback);
}

// MAME_PLUGIN_ROMPATH, MAME_PLUGIN_DATAPATH and MAME_PLUGIN_MACHINE override
// the rompath, datapath and machine entries of mame-plugin.ini in the
// configuration directory
settings load_settings()
{
#if defined(_WIN32)
	std::string const config_dir = env_or("APPDATA", ".") + "\\mame-plugin";
	std::string const data_dir = config_dir;
	char const sep = '\\';
#elif defined(__APPLE__)
	std::string const config_dir = env_or("HOME", ".") + "/Library/Application Support/mame-plugin";
	std::string const data_dir = config_dir;
	char const sep = '/';
#else
	std::string const home = env_or("HOME", ".");
	std::string const config_dir = env_or("XDG_CONFIG_HOME", home + "/.config") + "/mame-plugin";
	std::string const data_dir = env_or("XDG_DATA_HOME", home + "/.local/share") + "/mame-plugin";
	char const sep = '/';
#endif

	settings result;
	result.datapath = data_dir;
	result.rompath = data_dir + sep + "roms";

	std::ifstream file(config_dir + sep + "mame-plugin.ini");
	std::string line;
	while (std::getline(file, line))
	{
		auto const eq = line.find('=');
		if ((eq == std::string::npos) || (line[0] == '#') || (line[0] == ';'))
			continue;
		std::string const key = trim(std::string_view(line).substr(0, eq));
		std::string value = trim(std::string_view(line).substr(eq + 1));
		if (key == "rompath")
			result.rompath = std::move(value);
		else if (key == "datapath")
			result.datapath = std::move(value);
		else if (key == "machine")
			result.machine = std::move(value);
	}

	result.rompath = env_or("MAME_PLUGIN_ROMPATH", std::move(result.rompath));
	result.datapath = env_or("MAME_PLUGIN_DATAPATH", std::move(result.datapath));
	result.machine = env_or("MAME_PLUGIN_MACHINE", std::move(result.machine));
	return result;
}


//**************************************************************************
//  DESCRIPTOR
//**************************************************************************

char const *const s_features[] = {
	CLAP_PLUGIN_FEATURE_INSTRUMENT,
	CLAP_PLUGIN_FEATURE_SYNTHESIZER,
	CLAP_PLUGIN_FEATURE_STEREO,
	nullptr };

char s_version[64];

clap_plugin_descriptor_t const s_descriptor = {
	CLAP_VERSION_INIT,
	"org.mamedev.mame.synth",
	"MAME Synth",
	"MAMEdev",
	"https://mamedev.org",
	"",
	"",
	s_version,
	"Emulated hardware synthesizers",
	s_features };

constexpr clap_id PARAM_MACHINE = 0;

// number of bytes in a MIDI channel or system common message with the
// given status byte, not counting SysEx
unsigned midi_message_length(uint8_t status)
{
	switch (status & 0xf0)
	{
	case 0x80: case 0x90: case 0xa0: case 0xb0: case 0xe0:
		return 3;
	case 0xc0: case 0xd0:
		return 2;
	case 0xf0:
		switch (status)
		{
		case 0xf1: case 0xf3: return 2;
		case 0xf2: return 3;
		case 0xf0: case 0xf7: return 0;
		default: return 1;
		}
	default:
		return 0;
	}
}


//**************************************************************************
//  PLUGIN
//**************************************************************************

class mame_clap
{
public:
	mame_clap(clap_host_t const *host);

	clap_plugin_t const *plugin() const { return &m_plugin; }

private:
	static mame_clap &self(clap_plugin_t const *plugin) { return *static_cast<mame_clap *>(plugin->plugin_data); }

	// clap_plugin
	bool init();
	void destroy();
	bool activate(double sample_rate, uint32_t min_frames, uint32_t max_frames);
	void deactivate();
	clap_process_status process(clap_process_t const *process);
	void const *get_extension(char const *id);

	// extensions
	uint32_t audio_ports_count(bool is_input) const { return is_input ? 0 : 1; }
	bool audio_ports_get(uint32_t index, bool is_input, clap_audio_port_info_t *info) const;
	uint32_t note_ports_count(bool is_input) const { return is_input ? 1 : 0; }
	bool note_ports_get(uint32_t index, bool is_input, clap_note_port_info_t *info) const;
	uint32_t latency_get() const { return m_instance ? mp_latency(m_instance) : 0; }
	bool render_set(clap_plugin_render_mode mode);
	uint32_t params_count() const { return 1; }
	bool params_get_info(uint32_t index, clap_param_info_t *info) const;
	bool params_get_value(clap_id id, double *value) const;
	bool params_value_to_text(clap_id id, double value, char *text, uint32_t size) const;
	bool params_text_to_value(clap_id id, char const *text, double *value) const;
	void params_flush(clap_input_events_t const *in, clap_output_events_t const *out);
	bool state_save(clap_ostream_t const *stream);
	bool state_load(clap_istream_t const *stream);

	void handle_param(clap_event_param_value_t const &event);
	void update_nvram();
	int machine_index(std::string_view name) const;
	void log(clap_log_severity severity, std::string const &message) const;

	static clap_plugin_audio_ports_t const s_audio_ports;
	static clap_plugin_note_ports_t const s_note_ports;
	static clap_plugin_latency_t const s_latency;
	static clap_plugin_render_t const s_render;
	static clap_plugin_params_t const s_params;
	static clap_plugin_state_t const s_state;

	clap_plugin_t m_plugin;
	clap_host_t const *const m_host;
	clap_host_log_t const *m_host_log = nullptr;
	clap_host_params_t const *m_host_params = nullptr;
	clap_host_state_t const *m_host_state = nullptr;

	settings m_settings;
	int m_default_machine = 0;
	std::string m_machine;           // main thread
	std::vector<uint8_t> m_nvram;    // main thread, last known NVRAM of m_nvram_machine
	std::string m_nvram_machine;
	std::atomic<int> m_requested{ -1 };  // machine index requested from any thread
	bool m_offline = false;
	mp_instance *m_instance = nullptr;

	// MIDI collected for one block; SysEx points into the host's buffers
	static constexpr uint32_t MAX_EVENTS = 1024;
	mp_midi_event m_events[MAX_EVENTS];
	uint8_t m_event_bytes[MAX_EVENTS][3];
};


mame_clap::mame_clap(clap_host_t const *host) : m_host(host)
{
	m_plugin.desc = &s_descriptor;
	m_plugin.plugin_data = this;
	m_plugin.init = [] (clap_plugin_t const *p) { return self(p).init(); };
	m_plugin.destroy = [] (clap_plugin_t const *p) { self(p).destroy(); };
	m_plugin.activate = [] (clap_plugin_t const *p, double rate, uint32_t min, uint32_t max) { return self(p).activate(rate, min, max); };
	m_plugin.deactivate = [] (clap_plugin_t const *p) { self(p).deactivate(); };
	m_plugin.start_processing = [] (clap_plugin_t const *p) { return true; };
	m_plugin.stop_processing = [] (clap_plugin_t const *p) { };
	m_plugin.reset = [] (clap_plugin_t const *p) { };
	m_plugin.process = [] (clap_plugin_t const *p, clap_process_t const *process) { return self(p).process(process); };
	m_plugin.get_extension = [] (clap_plugin_t const *p, char const *id) { return self(p).get_extension(id); };
	m_plugin.on_main_thread = [] (clap_plugin_t const *p) { };
}


bool mame_clap::init()
{
	m_host_log = static_cast<clap_host_log_t const *>(m_host->get_extension(m_host, CLAP_EXT_LOG));
	m_host_params = static_cast<clap_host_params_t const *>(m_host->get_extension(m_host, CLAP_EXT_PARAMS));
	m_host_state = static_cast<clap_host_state_t const *>(m_host->get_extension(m_host, CLAP_EXT_STATE));

	m_settings = load_settings();
	if (!mp_machine_count())
	{
		log(CLAP_LOG_ERROR, "MAME: no machines with MIDI input in this build");
		return false;
	}

	// start with the configured machine, falling back to the first one
	m_default_machine = std::max(machine_index(m_settings.machine), 0);
	m_machine = mp_machine_name(m_default_machine);
	return true;
}


void mame_clap::destroy()
{
	deactivate();
	delete this;
}


bool mame_clap::activate(double sample_rate, uint32_t min_frames, uint32_t max_frames)
{
	// apply a machine change requested while inactive or that caused the restart
	int const requested = m_requested.exchange(-1);
	if (requested >= 0)
		m_machine = mp_machine_name(requested);

	std::string const datapath = m_settings.datapath + "/" + m_machine;
	mp_config config = {};
	config.driver = m_machine.c_str();
	config.rompath = m_settings.rompath.c_str();
	config.datapath = datapath.c_str();
	config.sample_rate = uint32_t(sample_rate);
	config.max_block_frames = max_frames;
	if ((m_nvram_machine == m_machine) && !m_nvram.empty())
	{
		config.nvram = m_nvram.data();
		config.nvram_size = m_nvram.size();
	}

	char error[1024];
	m_instance = mp_create(&config, error, sizeof(error));
	if (!m_instance)
	{
		// stay active and output silence, so another machine can be selected
		log(CLAP_LOG_ERROR, "MAME: " + m_machine + " failed to start:\n" + error);
		return true;
	}
	mp_set_offline(m_instance, m_offline);

	if (char const *const messages = mp_log(m_instance))
		log(CLAP_LOG_WARNING, "MAME: " + m_machine + ":\n" + messages);
	log(CLAP_LOG_INFO, "MAME: started " + m_machine);
	return true;
}


void mame_clap::deactivate()
{
	if (m_instance)
	{
		// keep edits across restarts, e.g. for a sample rate change
		update_nvram();
		mp_destroy(m_instance);
		m_instance = nullptr;
	}
}


// fetch the running machine's NVRAM
void mame_clap::update_nvram()
{
	size_t size = 0;
	auto const *const data = static_cast<uint8_t const *>(mp_nvram(m_instance, &size));
	if (data)
	{
		m_nvram.assign(data, data + size);
		m_nvram_machine = m_machine;
	}
	if (char const *const messages = mp_log(m_instance))
		log(CLAP_LOG_WARNING, "MAME: " + m_machine + ":\n" + messages);
}


clap_process_status mame_clap::process(clap_process_t const *process)
{
	uint32_t const frames = process->frames_count;
	uint32_t count = 0;

	clap_input_events_t const *const in = process->in_events;
	uint32_t const event_count = in->size(in);
	for (uint32_t i = 0; i < event_count; i++)
	{
		clap_event_header_t const *const header = in->get(in, i);
		if (header->space_id != CLAP_CORE_EVENT_SPACE_ID)
			continue;

		mp_midi_event &event = m_events[count];
		event.frame = std::min(header->time, frames);
		switch (header->type)
		{
		case CLAP_EVENT_MIDI:
			{
				auto const &midi = *reinterpret_cast<clap_event_midi_t const *>(header);
				uint32_t const length = midi_message_length(midi.data[0]);
				if (!length || (count >= MAX_EVENTS))
					break;
				std::memcpy(m_event_bytes[count], midi.data, 3);
				event.size = length;
				event.data = m_event_bytes[count];
				count++;
			}
			break;

		case CLAP_EVENT_MIDI_SYSEX:
			{
				auto const &sysex = *reinterpret_cast<clap_event_midi_sysex_t const *>(header);
				if (!sysex.size || (count >= MAX_EVENTS))
					break;
				event.size = sysex.size;
				event.data = sysex.buffer;
				count++;
			}
			break;

		case CLAP_EVENT_NOTE_ON:
		case CLAP_EVENT_NOTE_OFF:
			{
				auto const &note = *reinterpret_cast<clap_event_note_t const *>(header);
				if ((note.key < 0) || (note.key > 127) || (count >= MAX_EVENTS))
					break;
				uint8_t const channel = (note.channel >= 0) ? (note.channel & 0x0f) : 0;
				bool const on = header->type == CLAP_EVENT_NOTE_ON;
				uint8_t velocity = uint8_t(std::clamp(note.velocity, 0.0, 1.0) * 127.0 + 0.5);
				if (on && !velocity)
					velocity = 1;
				m_event_bytes[count][0] = (on ? 0x90 : 0x80) | channel;
				m_event_bytes[count][1] = uint8_t(note.key);
				m_event_bytes[count][2] = on ? velocity : 64;
				event.size = 3;
				event.data = m_event_bytes[count];
				count++;
			}
			break;

		case CLAP_EVENT_PARAM_VALUE:
			handle_param(*reinterpret_cast<clap_event_param_value_t const *>(header));
			break;
		}
	}

	float *const *const out = process->audio_outputs[0].data32;
	if (m_instance)
	{
		mp_process(m_instance, m_events, count, out, frames);
	}
	else
	{
		std::fill_n(out[0], frames, 0.0f);
		std::fill_n(out[1], frames, 0.0f);
	}
	return CLAP_PROCESS_CONTINUE;
}


void const *mame_clap::get_extension(char const *id)
{
	if (!std::strcmp(id, CLAP_EXT_AUDIO_PORTS))
		return &s_audio_ports;
	if (!std::strcmp(id, CLAP_EXT_NOTE_PORTS))
		return &s_note_ports;
	if (!std::strcmp(id, CLAP_EXT_LATENCY))
		return &s_latency;
	if (!std::strcmp(id, CLAP_EXT_RENDER))
		return &s_render;
	if (!std::strcmp(id, CLAP_EXT_PARAMS))
		return &s_params;
	if (!std::strcmp(id, CLAP_EXT_STATE))
		return &s_state;
	return nullptr;
}


bool mame_clap::audio_ports_get(uint32_t index, bool is_input, clap_audio_port_info_t *info) const
{
	if (is_input || index)
		return false;
	info->id = 0;
	std::snprintf(info->name, sizeof(info->name), "%s", "Output");
	info->flags = CLAP_AUDIO_PORT_IS_MAIN;
	info->channel_count = 2;
	info->port_type = CLAP_PORT_STEREO;
	info->in_place_pair = CLAP_INVALID_ID;
	return true;
}


bool mame_clap::note_ports_get(uint32_t index, bool is_input, clap_note_port_info_t *info) const
{
	if (!is_input || index)
		return false;
	info->id = 0;
	info->supported_dialects = CLAP_NOTE_DIALECT_MIDI | CLAP_NOTE_DIALECT_CLAP;
	info->preferred_dialect = CLAP_NOTE_DIALECT_MIDI;
	std::snprintf(info->name, sizeof(info->name), "%s", "MIDI In");
	return true;
}


bool mame_clap::render_set(clap_plugin_render_mode mode)
{
	m_offline = mode == CLAP_RENDER_OFFLINE;
	if (m_instance)
		mp_set_offline(m_instance, m_offline);
	return true;
}


bool mame_clap::params_get_info(uint32_t index, clap_param_info_t *info) const
{
	if (index != 0)
		return false;
	info->id = PARAM_MACHINE;
	// automatable so hosts show it in their generic editor; a change
	// restarts the plugin
	info->flags = CLAP_PARAM_IS_STEPPED | CLAP_PARAM_IS_ENUM | CLAP_PARAM_IS_AUTOMATABLE;
	info->cookie = nullptr;
	std::snprintf(info->name, sizeof(info->name), "%s", "Machine");
	info->module[0] = '\0';
	info->min_value = 0.0;
	info->max_value = double(mp_machine_count() - 1);
	info->default_value = double(m_default_machine);
	return true;
}


bool mame_clap::params_get_value(clap_id id, double *value) const
{
	if (id != PARAM_MACHINE)
		return false;
	int const requested = m_requested.load();
	*value = double((requested >= 0) ? requested : std::max(machine_index(m_machine), 0));
	return true;
}


bool mame_clap::params_value_to_text(clap_id id, double value, char *text, uint32_t size) const
{
	uint32_t const index = uint32_t(std::max(value, 0.0) + 0.5);
	if ((id != PARAM_MACHINE) || (index >= mp_machine_count()))
		return false;
	std::snprintf(text, size, "%s (%s)", mp_machine_description(index), mp_machine_name(index));
	return true;
}


bool mame_clap::params_text_to_value(clap_id id, char const *text, double *value) const
{
	if (id != PARAM_MACHINE)
		return false;
	for (uint32_t i = 0; i < mp_machine_count(); i++)
	{
		char buffer[256];
		params_value_to_text(id, double(i), buffer, sizeof(buffer));
		if (!std::strcmp(text, mp_machine_name(i)) || !std::strcmp(text, buffer))
		{
			*value = double(i);
			return true;
		}
	}
	return false;
}


void mame_clap::params_flush(clap_input_events_t const *in, clap_output_events_t const *out)
{
	uint32_t const count = in->size(in);
	for (uint32_t i = 0; i < count; i++)
	{
		clap_event_header_t const *const header = in->get(in, i);
		if ((header->space_id == CLAP_CORE_EVENT_SPACE_ID) && (header->type == CLAP_EVENT_PARAM_VALUE))
			handle_param(*reinterpret_cast<clap_event_param_value_t const *>(header));
	}
}


// state: a header line, key=value lines and the NVRAM blob
//   mame-plugin-state 1
//   machine=<name>
//   nvram=<size>
//   <size bytes>
constexpr std::string_view STATE_HEADER = "mame-plugin-state 1";

bool mame_clap::state_save(clap_ostream_t const *stream)
{
	int const requested = m_requested.load();
	std::string const machine = (requested >= 0) ? std::string(mp_machine_name(requested)) : m_machine;

	// NVRAM belongs to a machine, a pending change to another one starts afresh
	if (m_instance && (machine == m_machine))
		update_nvram();
	bool const with_nvram = (m_nvram_machine == machine) && !m_nvram.empty();

	std::string data = std::string(STATE_HEADER) + "\nmachine=" + machine + "\n";
	if (with_nvram)
	{
		data += "nvram=" + std::to_string(m_nvram.size()) + "\n";
		data.append(reinterpret_cast<char const *>(m_nvram.data()), m_nvram.size());
	}

	char const *ptr = data.data();
	std::size_t remaining = data.size();
	while (remaining)
	{
		int64_t const written = stream->write(stream, ptr, remaining);
		if (written <= 0)
			return false;
		ptr += written;
		remaining -= std::size_t(written);
	}
	return true;
}


bool mame_clap::state_load(clap_istream_t const *stream)
{
	std::string data;
	char buffer[4096];
	for (int64_t got; (got = stream->read(stream, buffer, sizeof(buffer))) != 0; )
	{
		if (got < 0)
			return false;
		data.append(buffer, std::size_t(got));
	}

	std::string machine;
	std::vector<uint8_t> nvram;
	std::string_view rest(data);
	bool first = true;
	while (!rest.empty())
	{
		auto const end = rest.find('\n');
		std::string_view const line = rest.substr(0, end);
		rest = (end == std::string_view::npos) ? std::string_view() : rest.substr(end + 1);
		if (first && (line != STATE_HEADER))
		{
			log(CLAP_LOG_ERROR, "MAME: state is not in a known format");
			return false;
		}
		first = false;

		if (line.substr(0, 8) == "machine=")
		{
			machine = trim(line.substr(8));
		}
		else if (line.substr(0, 6) == "nvram=")
		{
			std::size_t const size = std::strtoull(std::string(line.substr(6)).c_str(), nullptr, 10);
			if (size > rest.size())
			{
				log(CLAP_LOG_ERROR, "MAME: state is truncated");
				return false;
			}
			nvram.assign(rest.begin(), rest.begin() + size);
			rest = rest.substr(size);
		}
	}

	// a state must name a machine available in this build
	int const index = machine_index(machine);
	if (index < 0)
	{
		log(CLAP_LOG_ERROR, "MAME: state doesn't name a known machine");
		return false;
	}

	m_nvram = std::move(nvram);
	m_nvram_machine = machine;

	// a running machine is restarted, even if it is the same one, as NVRAM
	// can only be applied at startup
	if (m_instance)
	{
		m_requested = index;
		m_host->request_restart(m_host);
	}
	else
	{
		m_requested = (index == machine_index(m_machine)) ? -1 : index;
	}
	if (m_host_params)
		m_host_params->rescan(m_host, CLAP_PARAM_RESCAN_VALUES);
	return true;
}


void mame_clap::handle_param(clap_event_param_value_t const &event)
{
	if (event.param_id != PARAM_MACHINE)
		return;
	int const index = int(std::max(event.value, 0.0) + 0.5);
	if ((index >= int(mp_machine_count())) || (index == m_requested.load()))
		return;
	if ((m_requested.load() < 0) && (index == machine_index(m_machine)))
		return;

	// the machine can only be replaced while inactive
	m_requested = index;
	m_host->request_restart(m_host);
}


int mame_clap::machine_index(std::string_view name) const
{
	for (uint32_t i = 0; i < mp_machine_count(); i++)
	{
		if (name == mp_machine_name(i))
			return int(i);
	}
	return -1;
}


void mame_clap::log(clap_log_severity severity, std::string const &message) const
{
	if (m_host_log)
		m_host_log->log(m_host, severity, message.c_str());
	else
		std::fprintf(stderr, "%s\n", message.c_str());
}


clap_plugin_audio_ports_t const mame_clap::s_audio_ports = {
	[] (clap_plugin_t const *p, bool is_input) { return self(p).audio_ports_count(is_input); },
	[] (clap_plugin_t const *p, uint32_t index, bool is_input, clap_audio_port_info_t *info) { return self(p).audio_ports_get(index, is_input, info); } };

clap_plugin_note_ports_t const mame_clap::s_note_ports = {
	[] (clap_plugin_t const *p, bool is_input) { return self(p).note_ports_count(is_input); },
	[] (clap_plugin_t const *p, uint32_t index, bool is_input, clap_note_port_info_t *info) { return self(p).note_ports_get(index, is_input, info); } };

clap_plugin_latency_t const mame_clap::s_latency = {
	[] (clap_plugin_t const *p) { return self(p).latency_get(); } };

clap_plugin_render_t const mame_clap::s_render = {
	[] (clap_plugin_t const *p) { return false; },
	[] (clap_plugin_t const *p, clap_plugin_render_mode mode) { return self(p).render_set(mode); } };

clap_plugin_params_t const mame_clap::s_params = {
	[] (clap_plugin_t const *p) { return self(p).params_count(); },
	[] (clap_plugin_t const *p, uint32_t index, clap_param_info_t *info) { return self(p).params_get_info(index, info); },
	[] (clap_plugin_t const *p, clap_id id, double *value) { return self(p).params_get_value(id, value); },
	[] (clap_plugin_t const *p, clap_id id, double value, char *text, uint32_t size) { return self(p).params_value_to_text(id, value, text, size); },
	[] (clap_plugin_t const *p, clap_id id, char const *text, double *value) { return self(p).params_text_to_value(id, text, value); },
	[] (clap_plugin_t const *p, clap_input_events_t const *in, clap_output_events_t const *out) { self(p).params_flush(in, out); } };

clap_plugin_state_t const mame_clap::s_state = {
	[] (clap_plugin_t const *p, clap_ostream_t const *stream) { return self(p).state_save(stream); },
	[] (clap_plugin_t const *p, clap_istream_t const *stream) { return self(p).state_load(stream); } };


//**************************************************************************
//  FACTORY AND ENTRY
//**************************************************************************

clap_plugin_factory_t const s_factory = {
	[] (clap_plugin_factory_t const *factory) -> uint32_t { return 1; },
	[] (clap_plugin_factory_t const *factory, uint32_t index) -> clap_plugin_descriptor_t const * { return index ? nullptr : &s_descriptor; },
	[] (clap_plugin_factory_t const *factory, clap_host_t const *host, char const *plugin_id) -> clap_plugin_t const *
	{
		if (!clap_version_is_compatible(host->clap_version) || std::strcmp(plugin_id, s_descriptor.id))
			return nullptr;
		return (new mame_clap(host))->plugin();
	} };

} // anonymous namespace


extern "C" {

#if defined(_WIN32)
__declspec(dllexport)
#else
__attribute__((visibility("default")))
#endif
extern clap_plugin_entry_t const clap_entry = {
	CLAP_VERSION_INIT,
	[] (char const *plugin_path) -> bool
	{
		std::snprintf(s_version, sizeof(s_version), "%s", mp_build_version());
		return true;
	},
	[] () { },
	[] (char const *factory_id) -> void const * { return std::strcmp(factory_id, CLAP_PLUGIN_FACTORY_ID) ? nullptr : &s_factory; } };

} // extern "C"
