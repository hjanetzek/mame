/* license:BSD-3-Clause
 * copyright-holders:Hannes Janetzek
 *
 * claphost.c - minimal CLAP host exercising the MAME plugin
 *
 * Loads the plugin, selects a machine through the state extension,
 * plays a note pattern as MIDI events, switches machines through the
 * parameter (handling the restart request like a host would) and saves
 * the state again.
 *
 * CLAPHOST_STATE_IN=file loads the initial state from a file instead,
 * CLAPHOST_STATE_OUT=file writes the final state to a file.
 * CLAPHOST_SYSEX="F0 .. F7" sends a SysEx message 9 seconds in; MIDI
 * the plugin outputs is summarised and SysEx printed.
 *
 *   cc -O2 -o claphost src/plugin/clap/test/claphost.c -I3rdparty/clap/include -ldl
 *   ./claphost ./mame_synths.clap tx81z out.wav [seconds] [switch-to-machine]
 */
#include <clap/clap.h>

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RATE 48000
#define BLOCK 256

static int restart_requested;

/* host */

static const void *host_get_extension(const clap_host_t *host, const char *id) { (void)host; (void)id; return NULL; }
static void host_request_restart(const clap_host_t *host) { restart_requested = 1; printf("host: restart requested\n"); }
static void host_request_process(const clap_host_t *host) { }
static void host_request_callback(const clap_host_t *host) { }

static const clap_host_t host = {
	CLAP_VERSION_INIT, NULL, "claphost", "MAMEdev", "", "1",
	host_get_extension, host_request_restart, host_request_process, host_request_callback };

/* event lists */

typedef struct event_list
{
	uint32_t count;
	const clap_event_header_t *events[16];
} event_list;

static uint32_t in_size(const clap_input_events_t *list) { return ((const event_list *)list->ctx)->count; }
static const clap_event_header_t *in_get(const clap_input_events_t *list, uint32_t index) { return ((const event_list *)list->ctx)->events[index]; }
static unsigned out_midi, out_sysex;

static bool out_try_push(const clap_output_events_t *list, const clap_event_header_t *event)
{
	(void)list;
	if (event->space_id != CLAP_CORE_EVENT_SPACE_ID)
		return false;
	if (event->type == CLAP_EVENT_MIDI)
	{
		out_midi++;
	}
	else if (event->type == CLAP_EVENT_MIDI_SYSEX)
	{
		const clap_event_midi_sysex_t *sysex = (const clap_event_midi_sysex_t *)event;
		out_sysex++;
		printf("plugin sent SysEx (%u bytes, frame %u):", sysex->size, event->time);
		for (uint32_t i = 0; (i < sysex->size) && (i < 16); i++)
			printf(" %02X", sysex->buffer[i]);
		printf("%s\n", (sysex->size > 16) ? " ..." : "");
	}
	return true;
}

/* streams */

typedef struct buffer
{
	char data[1 << 20];
	size_t size, pos;
} buffer;

static int64_t stream_write(const clap_ostream_t *stream, const void *data, uint64_t size)
{
	buffer *const b = (buffer *)stream->ctx;
	if (b->size + size > sizeof(b->data))
		return -1;
	memcpy(b->data + b->size, data, size);
	b->size += size;
	return (int64_t)size;
}

static int64_t stream_read(const clap_istream_t *stream, void *data, uint64_t size)
{
	buffer *const b = (buffer *)stream->ctx;
	size_t const n = (b->size - b->pos < size) ? (b->size - b->pos) : size;
	memcpy(data, b->data + b->pos, n);
	b->pos += n;
	return (int64_t)n;
}

/* WAV output */

static void put32(FILE *f, uint32_t v) { fputc(v & 0xff, f); fputc((v >> 8) & 0xff, f); fputc((v >> 16) & 0xff, f); fputc(v >> 24, f); }
static void put16(FILE *f, uint16_t v) { fputc(v & 0xff, f); fputc(v >> 8, f); }

static int16_t to16(float v)
{
	if (v > 1.0f) v = 1.0f;
	if (v < -1.0f) v = -1.0f;
	return (int16_t)(v * 32767.0f);
}

static int render(const clap_plugin_t *plugin, const char *path, double seconds, const clap_event_param_value_t *param, uint32_t param_block)
{
	FILE *wav = fopen(path, "wb");
	if (!wav)
	{
		perror(path);
		return 1;
	}
	uint32_t const blocks = (uint32_t)(seconds * RATE) / BLOCK;
	uint32_t const datasize = blocks * BLOCK * 4;
	fwrite("RIFF", 1, 4, wav); put32(wav, 36 + datasize); fwrite("WAVEfmt ", 1, 8, wav);
	put32(wav, 16); put16(wav, 1); put16(wav, 2); put32(wav, RATE); put32(wav, RATE * 4); put16(wav, 4); put16(wav, 16);
	fwrite("data", 1, 4, wav); put32(wav, datasize);

	static const uint8_t notes[] = { 60, 64, 67, 72, 67, 64 };
	uint32_t const boot = 8 * RATE, step = RATE / 2, length = RATE * 2 / 5;

	/* optional SysEx to send */
	uint8_t sysex_data[256];
	uint32_t sysex_size = 0;
	const char *sysex_env = getenv("CLAPHOST_SYSEX");
	for (const char *p = sysex_env; p && *p && (sysex_size < sizeof(sysex_data)); )
	{
		char *end;
		unsigned long const v = strtoul(p, &end, 16);
		if (end == p)
			break;
		sysex_data[sysex_size++] = (uint8_t)v;
		p = end;
	}
	clap_event_midi_sysex_t sysex_event;
	memset(&sysex_event, 0, sizeof(sysex_event));
	sysex_event.header.size = sizeof(sysex_event);
	sysex_event.header.space_id = CLAP_CORE_EVENT_SPACE_ID;
	sysex_event.header.type = CLAP_EVENT_MIDI_SYSEX;
	sysex_event.buffer = sysex_data;
	sysex_event.size = sysex_size;
	uint32_t const sysex_block = 9 * RATE / BLOCK;

	float left[BLOCK], right[BLOCK];
	float *channels[2] = { left, right };
	clap_audio_buffer_t output = { channels, NULL, 2, 0, 0 };
	clap_event_midi_t midi[8];
	event_list list;
	clap_input_events_t in = { &list, in_size, in_get };
	clap_output_events_t out = { NULL, out_try_push };

	clap_process_t process;
	memset(&process, 0, sizeof(process));
	process.frames_count = BLOCK;
	process.audio_outputs = &output;
	process.audio_outputs_count = 1;
	process.in_events = &in;
	process.out_events = &out;

	for (uint32_t b = 0; b < blocks; b++)
	{
		uint32_t const start = b * BLOCK;
		list.count = 0;
		if (param && (b == param_block))
			list.events[list.count++] = &param->header;
		if (sysex_size && (b == sysex_block))
			list.events[list.count++] = &sysex_event.header;
		for (uint32_t f = start; f < start + BLOCK; f++)
		{
			if (f < boot)
				continue;
			uint32_t const pos = (f - boot) % step;
			if ((pos == 0) || (pos == length))
			{
				clap_event_midi_t *const m = &midi[list.count];
				m->header.size = sizeof(*m);
				m->header.time = f - start;
				m->header.space_id = CLAP_CORE_EVENT_SPACE_ID;
				m->header.type = CLAP_EVENT_MIDI;
				m->header.flags = 0;
				m->port_index = 0;
				m->data[0] = (pos == 0) ? 0x90 : 0x80;
				m->data[1] = notes[((f - boot) / step) % sizeof(notes)];
				m->data[2] = (pos == 0) ? 100 : 0;
				list.events[list.count++] = &m->header;
			}
		}

		process.steady_time = start;
		if (plugin->process(plugin, &process) == CLAP_PROCESS_ERROR)
		{
			fprintf(stderr, "process failed\n");
			fclose(wav);
			return 1;
		}
		for (uint32_t i = 0; i < BLOCK; i++)
		{
			put16(wav, (uint16_t)to16(left[i]));
			put16(wav, (uint16_t)to16(right[i]));
		}
	}
	fclose(wav);
	printf("plugin sent %u MIDI and %u SysEx events\n", out_midi, out_sysex);
	return 0;
}

#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "check failed: %s (line %d)\n", #cond, __LINE__); return 1; } } while (0)

int main(int argc, char **argv)
{
	if (argc < 4)
	{
		fprintf(stderr, "usage: %s <plugin.clap> <machine> <out.wav> [seconds] [switch-to-machine]\n", argv[0]);
		return 1;
	}
	const char *path = argv[1];
	const char *machine = argv[2];
	const char *outpath = argv[3];
	double const seconds = (argc > 4) ? atof(argv[4]) : 12.0;
	const char *switch_to = (argc > 5) ? argv[5] : NULL;

	void *lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
	CHECK(lib);
	const clap_plugin_entry_t *entry = (const clap_plugin_entry_t *)dlsym(lib, "clap_entry");
	CHECK(entry && clap_version_is_compatible(entry->clap_version));
	CHECK(entry->init(path));

	const clap_plugin_factory_t *factory = (const clap_plugin_factory_t *)entry->get_factory(CLAP_PLUGIN_FACTORY_ID);
	CHECK(factory && (factory->get_plugin_count(factory) == 1));
	const clap_plugin_descriptor_t *desc = factory->get_plugin_descriptor(factory, 0);
	printf("plugin: %s (%s) version %s by %s\n", desc->name, desc->id, desc->version, desc->vendor);

	const clap_plugin_t *plugin = factory->create_plugin(factory, &host, desc->id);
	CHECK(plugin && plugin->init(plugin));

	const clap_plugin_audio_ports_t *audio_ports = plugin->get_extension(plugin, CLAP_EXT_AUDIO_PORTS);
	const clap_plugin_note_ports_t *note_ports = plugin->get_extension(plugin, CLAP_EXT_NOTE_PORTS);
	const clap_plugin_latency_t *latency = plugin->get_extension(plugin, CLAP_EXT_LATENCY);
	const clap_plugin_params_t *params = plugin->get_extension(plugin, CLAP_EXT_PARAMS);
	const clap_plugin_state_t *state = plugin->get_extension(plugin, CLAP_EXT_STATE);
	const clap_plugin_render_t *render_ext = plugin->get_extension(plugin, CLAP_EXT_RENDER);
	CHECK(audio_ports && note_ports && latency && params && state && render_ext);

	clap_audio_port_info_t api;
	CHECK((audio_ports->count(plugin, false) == 1) && audio_ports->get(plugin, 0, false, &api) && (api.channel_count == 2));
	clap_note_port_info_t npi;
	CHECK((note_ports->count(plugin, true) == 1) && note_ports->get(plugin, 0, true, &npi));
	printf("ports: audio out '%s' (%u ch), note in '%s' (dialects %x)\n", api.name, api.channel_count, npi.name, npi.supported_dialects);

	/* list the machine parameter */
	clap_param_info_t pinfo;
	CHECK((params->count(plugin) == 1) && params->get_info(plugin, 0, &pinfo));
	printf("param '%s':", pinfo.name);
	for (int v = (int)pinfo.min_value; v <= (int)pinfo.max_value; v++)
	{
		char text[256];
		CHECK(params->value_to_text(plugin, pinfo.id, v, text, sizeof(text)));
		printf(" [%d] %s", v, text);
	}
	printf("\n");

	/* select the machine through the state, as a host restoring a project */
	static buffer sbuf;
	const char *state_in = getenv("CLAPHOST_STATE_IN");
	if (state_in)
	{
		FILE *f = fopen(state_in, "rb");
		CHECK(f);
		sbuf.size = fread(sbuf.data, 1, sizeof(sbuf.data), f);
		fclose(f);
	}
	else
	{
		sbuf.size = (size_t)snprintf(sbuf.data, sizeof(sbuf.data), "mame-plugin-state 1\nmachine=%s\n", machine);
	}
	clap_istream_t istream = { &sbuf, stream_read };
	CHECK(state->load(plugin, &istream));

	/* render offline */
	CHECK(render_ext->set(plugin, CLAP_RENDER_OFFLINE));
	CHECK(plugin->activate(plugin, RATE, 1, BLOCK));
	printf("latency: %u frames\n", latency->get(plugin));
	CHECK(plugin->start_processing(plugin));

	clap_event_param_value_t change;
	double target = 0.0;
	if (switch_to)
	{
		CHECK(params->text_to_value(plugin, pinfo.id, switch_to, &target));
		memset(&change, 0, sizeof(change));
		change.header.size = sizeof(change);
		change.header.space_id = CLAP_CORE_EVENT_SPACE_ID;
		change.header.type = CLAP_EVENT_PARAM_VALUE;
		change.param_id = pinfo.id;
		change.note_id = -1;
		change.port_index = change.channel = change.key = -1;
		change.value = target;
	}
	/* switch machine at the last block */
	CHECK(!render(plugin, outpath, seconds, switch_to ? &change : NULL, (uint32_t)(seconds * RATE) / BLOCK - 1));

	plugin->stop_processing(plugin);
	plugin->deactivate(plugin);

	if (switch_to)
	{
		CHECK(restart_requested);
		double value;
		CHECK(params->get_value(plugin, pinfo.id, &value) && (value == target));
		printf("restarting with %s\n", switch_to);
		CHECK(plugin->activate(plugin, RATE, 1, BLOCK));
		plugin->deactivate(plugin);
	}

	/* the saved state names the current machine */
	static buffer obuf;
	clap_ostream_t ostream = { &obuf, stream_write };
	CHECK(state->save(plugin, &ostream));
	const char *nvram = strstr(obuf.data, "nvram=");
	printf("state: %d bytes, %.*s\n", (int)obuf.size, (int)strcspn(obuf.data + 20, "\n"), obuf.data + 20);
	if (nvram)
		printf("state: %.*s\n", (int)strcspn(nvram, "\n"), nvram);
	const char *state_out = getenv("CLAPHOST_STATE_OUT");
	if (state_out)
	{
		FILE *f = fopen(state_out, "wb");
		CHECK(f && (fwrite(obuf.data, 1, obuf.size, f) == obuf.size));
		fclose(f);
	}

	plugin->destroy(plugin);
	entry->deinit();
	dlclose(lib);
	printf("ok\n");
	return 0;
}
