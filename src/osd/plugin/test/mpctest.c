/* license:BSD-3-Clause
 * copyright-holders:Hannes Janetzek
 *
 * mpctest.c - exercise the MAME plugin library C API
 *
 * Loads the library, plays a fixed note pattern into a machine and
 * writes the output to a 16-bit stereo WAV file.
 *
 * With MPCTEST_INSTANCES=n, n instances are created and rendered
 * concurrently on separate threads, writing <out>.<i>.wav and using
 * <datapath>.<i> as data directory.  With MPCTEST_NVRAM=1 the main
 * thread repeatedly takes NVRAM snapshots while rendering, as a host
 * saving its project would.
 *
 * With MPCTEST_PANEL=prefix the front panel is rendered at its natural
 * size times MPCTEST_PANEL_SCALE (default 1) and written to
 * <prefix>.<second>.ppm once per second.  MPCTEST_CLICK=x,y,t0,t1
 * holds the primary button down at panel pixel x,y from t0 to t1
 * seconds; MPCTEST_CLICK=x,y,t0,t1,x1,y1 drags from x,y to x1,y1
 * meanwhile.  Several clicks are separated by ';'.
 *
 * MPCTEST_NOTES=note=input;note=input;... maps notes to inputs of a
 * machine without MIDI input (mp_config::note_map), e.g.
 * "60=SNARE 3;64=SIDESTICK".
 *
 * MPCTEST_SYSEX=file.syx sends the SysEx messages of the file after
 * boot, 200ms apart, and starts the notes a second after the last.
 * MPCTEST_NOTE=n holds note n instead of playing the pattern.
 * MPCTEST_RATE and MPCTEST_BLOCK set the sample rate (default 48000)
 * and block size (default 256).  MPCTEST_NVRAM_OUT=dir writes the NVRAM
 * of the first instance at the end, one file per device named like
 * MAME's NVRAM files, e.g. to make a data folder's starting NVRAM.
 * MPCTEST_OPTIONS=name=value;... passes extra MAME options, and
 * MPCTEST_RELOAD_LAYOUT=1 reloads changed layout files.
 * MPCTEST_FIELDS=file writes the machine's adjusters as the parameter
 * descriptions of a gearmulator plugin product.  With
 * MPCTEST_LAYOUT=1 (and MPCTEST_PANEL) a frame composed from the layout
 * description (mp_layout_*) is written next to each panel frame as
 * <prefix>.layout.<i>.<second>.ppm, with the number of pixels that differ.
 *
 *   cc -O2 -o mpctest src/osd/plugin/test/mpctest.c -Isrc/osd/plugin -ldl -lpthread -lm
 *   ./mpctest ./libmameplugin_synths.so tx81z roms out.wav [seconds] [datapath] [--realtime]
 */
#include "mame_plugin.h"

#include <dlfcn.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* MPCTEST_RATE and MPCTEST_BLOCK override these */
#define MAX_BLOCK 4096
static uint32_t RATE = 48000;
static uint32_t BLOCK = 256;

#define MAX_INSTANCES 16

#define LOAD(name) \
	p_##name = (name##_fn)dlsym(lib, #name); \
	if (!p_##name) { fprintf(stderr, "missing symbol %s\n", #name); return 1; }

typedef uint32_t (*mp_machine_count_fn)(void);
typedef const char *(*mp_machine_name_fn)(uint32_t);
typedef const char *(*mp_build_version_fn)(void);
typedef mp_instance *(*mp_create_fn)(const mp_config *, char *, size_t);
typedef void (*mp_destroy_fn)(mp_instance *);
typedef uint32_t (*mp_latency_fn)(const mp_instance *);
typedef void (*mp_process_fn)(mp_instance *, const mp_midi_event *, uint32_t, float *const *, uint32_t);
typedef void (*mp_set_offline_fn)(mp_instance *, int);
typedef uint64_t (*mp_underruns_fn)(const mp_instance *);
typedef const char *(*mp_log_fn)(mp_instance *);
typedef const void *(*mp_nvram_fn)(mp_instance *, size_t *);
typedef void (*mp_panel_size_fn)(mp_instance *, uint32_t *, uint32_t *);
typedef void (*mp_panel_set_size_fn)(mp_instance *, uint32_t, uint32_t);
typedef uint64_t (*mp_panel_frame_fn)(mp_instance *, void *, uint32_t, uint32_t, uint64_t);
typedef void (*mp_panel_pointer_fn)(mp_instance *, uint32_t, int32_t, int32_t, uint32_t);
typedef uint32_t (*mp_field_count_fn)(const mp_instance *);
typedef int (*mp_field_info_fn)(const mp_instance *, uint32_t, mp_field *);
typedef uint64_t (*mp_layout_describe_fn)(mp_instance *, mp_layout_info *, mp_layout_element *, uint32_t, mp_layout_item *, uint32_t);
typedef uint64_t (*mp_layout_states_fn)(mp_instance *, uint64_t, mp_layout_item_state *, uint32_t, uint64_t *);
typedef int (*mp_layout_element_image_fn)(mp_instance *, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, void *);
typedef uint64_t (*mp_layout_screen_fn)(mp_instance *, uint32_t, uint64_t, uint32_t *, uint32_t *, void *, size_t);

static mp_build_version_fn p_mp_build_version;
static mp_machine_count_fn p_mp_machine_count;
static mp_machine_name_fn p_mp_machine_name;
static mp_create_fn p_mp_create;
static mp_destroy_fn p_mp_destroy;
static mp_latency_fn p_mp_latency;
static mp_process_fn p_mp_process;
static mp_set_offline_fn p_mp_set_offline;
static mp_underruns_fn p_mp_underruns;
static mp_log_fn p_mp_log;
static mp_nvram_fn p_mp_nvram;
static mp_panel_size_fn p_mp_panel_size;
static mp_panel_set_size_fn p_mp_panel_set_size;
static mp_panel_frame_fn p_mp_panel_frame;
static mp_panel_pointer_fn p_mp_panel_pointer;
static mp_field_count_fn p_mp_field_count;
static mp_field_info_fn p_mp_field_info;
static mp_layout_describe_fn p_mp_layout_describe;
static mp_layout_states_fn p_mp_layout_states;
static mp_layout_element_image_fn p_mp_layout_element_image;
static mp_layout_screen_fn p_mp_layout_screen;

/* call through the pointers resolved with dlsym */
#define mp_build_version p_mp_build_version
#define mp_machine_count p_mp_machine_count
#define mp_machine_name p_mp_machine_name
#define mp_create p_mp_create
#define mp_destroy p_mp_destroy
#define mp_latency p_mp_latency
#define mp_process p_mp_process
#define mp_set_offline p_mp_set_offline
#define mp_underruns p_mp_underruns
#define mp_log p_mp_log
#define mp_nvram p_mp_nvram
#define mp_panel_size p_mp_panel_size
#define mp_panel_set_size p_mp_panel_set_size
#define mp_panel_frame p_mp_panel_frame
#define mp_panel_pointer p_mp_panel_pointer
#define mp_field_count p_mp_field_count
#define mp_field_info p_mp_field_info
#define mp_layout_describe p_mp_layout_describe
#define mp_layout_states p_mp_layout_states
#define mp_layout_element_image p_mp_layout_element_image
#define mp_layout_screen p_mp_layout_screen

typedef struct job
{
	int index;
	mp_instance *inst;
	char outpath[1024];
	char datapath[1024];
	double seconds;
	int realtime;
	int result;
	volatile int finished;
	volatile uint32_t frames;   /* frames rendered so far */
	const char *panel;
	uint32_t pw, ph;
} job;

static void put32(FILE *f, uint32_t v) { fputc(v & 0xff, f); fputc((v >> 8) & 0xff, f); fputc((v >> 16) & 0xff, f); fputc(v >> 24, f); }
static void put16(FILE *f, uint16_t v) { fputc(v & 0xff, f); fputc(v >> 8, f); }

static int16_t to16(float v)
{
	if (v > 1.0f) v = 1.0f;
	if (v < -1.0f) v = -1.0f;
	return (int16_t)(v * 32767.0f);
}

static void write_ppm(const char *prefix, int index, uint32_t second, const uint8_t *rgba, uint32_t width, uint32_t height)
{
	char path[1100];
	snprintf(path, sizeof(path), "%s.%d.%u.ppm", prefix, index, second);
	FILE *f = fopen(path, "wb");
	if (!f)
	{
		perror(path);
		return;
	}
	fprintf(f, "P6\n%u %u\n255\n", width, height);
	for (uint32_t i = 0; i < width * height; i++)
		fwrite(rgba + i * 4, 1, 3, f);
	fclose(f);
}

/* as MAME's render_round_nearest */
static int round_nearest(float f) { return (int)floorf(f + 0.5f); }

static uint8_t clamp8(int v) { return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v); }

/* blends one pixel as MAME's software renderer does */
static void blend(uint8_t *d, const uint8_t *s, uint32_t mode, const mp_layout_item_state *st)
{
	int const r = (int)(s[0] * st->r), g = (int)(s[1] * st->g), b = (int)(s[2] * st->b);
	int const a = (int)(s[3] * st->a);
	switch (mode)
	{
	case MP_LAYOUT_BLEND_NONE:
		d[0] = clamp8(r); d[1] = clamp8(g); d[2] = clamp8(b);
		break;
	case MP_LAYOUT_BLEND_ALPHA:
		if (a)
		{
			d[0] = clamp8((r * a + d[0] * (256 - a)) >> 8);
			d[1] = clamp8((g * a + d[1] * (256 - a)) >> 8);
			d[2] = clamp8((b * a + d[2] * (256 - a)) >> 8);
		}
		break;
	case MP_LAYOUT_BLEND_MULTIPLY:
		d[0] = clamp8((r * d[0]) >> 8); d[1] = clamp8((g * d[1]) >> 8); d[2] = clamp8((b * d[2]) >> 8);
		break;
	case MP_LAYOUT_BLEND_ADD:
		d[0] = clamp8(d[0] + ((r * a) >> 8)); d[1] = clamp8(d[1] + ((g * a) >> 8)); d[2] = clamp8(d[2] + ((b * a) >> 8));
		break;
	}
}

/* composes the panel from the layout description, as a GUI drawing the
   layout itself would; returns 0 if there is no description yet */
static int compose_layout(mp_instance *inst, uint8_t *out, uint32_t width, uint32_t height)
{
	mp_layout_info info;
	mp_layout_describe(inst, &info, NULL, 0, NULL, 0);
	if (!info.serial || !info.item_count)
		return 0;
	mp_layout_item *const items = calloc(info.item_count, sizeof(*items));
	mp_layout_item_state *const states = calloc(info.item_count, sizeof(*states));
	mp_layout_describe(inst, &info, NULL, 0, items, info.item_count);
	uint64_t layout_serial = 0;
	if (!mp_layout_states(inst, 0, states, info.item_count, &layout_serial) || (layout_serial != info.serial))
	{
		free(items);
		free(states);
		return 0;
	}

	/* the view fills the frame, as MAME's panel target does; a GUI would
	   size its canvas from info.aspect */
	float const visw = (float)width, vish = (float)height, ox = 0.0f, oy = 0.0f;

	memset(out, 0, (size_t)width * height * 4);
	for (uint32_t i = 0; i < info.item_count; i++)
	{
		const mp_layout_item_state *const st = &states[i];
		int const x0 = round_nearest(ox + st->x0 * visw), y0 = round_nearest(oy + st->y0 * vish);
		int const w = round_nearest((st->x1 - st->x0) * visw), h = round_nearest((st->y1 - st->y0) * vish);
		if (w <= 0 || h <= 0 || (items[i].flags & MP_LAYOUT_SCROLLED))
			continue;
		uint8_t *image = NULL;
		uint32_t iw = (uint32_t)w, ih = (uint32_t)h;
		if (items[i].element != MP_LAYOUT_NO_ELEMENT)
		{
			image = malloc((size_t)w * h * 4);
			if (!mp_layout_element_image(inst, items[i].element, (uint32_t)st->state, items[i].orientation, iw, ih, image))
			{
				free(image);
				continue;
			}
		}
		else
		{
			uint32_t sw = 0, sh = 0;
			mp_layout_screen(inst, items[i].screen, ~0ULL, &sw, &sh, NULL, 0);
			if (!sw || !sh)
				continue;
			image = malloc((size_t)sw * sh * 4);
			mp_layout_screen(inst, items[i].screen, 0, &sw, &sh, image, (size_t)sw * sh * 4);
			iw = sw;
			ih = sh;
		}
		for (int y = 0; y < h; y++)
		{
			if (y0 + y < 0 || y0 + y >= (int)height)
				continue;
			for (int x = 0; x < w; x++)
			{
				if (x0 + x < 0 || x0 + x >= (int)width)
					continue;
				uint8_t *const d = out + ((size_t)(y0 + y) * width + x0 + x) * 4;
				if (iw == (uint32_t)w && ih == (uint32_t)h)
				{
					/* element images have the item's size */
					blend(d, image + ((size_t)y * iw + x) * 4, items[i].blend, st);
					continue;
				}

				/* screens are scaled like the panel's software renderer
				   does (rendersw.hxx): nearest texel from the first
				   pixel's centre, stepping in 16.16 fixed point by the
				   texture size over the unrounded bounds */
				int const dudx = round_nearest(65536.0f * (float)iw / ((st->x1 - st->x0) * visw));
				int const dvdy = round_nearest(65536.0f * (float)ih / ((st->y1 - st->y0) * vish));
				uint32_t u = (uint32_t)((dudx / 2 + x * dudx) >> 16), v = (uint32_t)((dvdy / 2 + y * dvdy) >> 16);
				u = u < iw ? u : iw - 1;
				v = v < ih ? v : ih - 1;
				blend(d, image + ((size_t)v * iw + u) * 4, items[i].blend, st);
			}
		}
		free(image);
	}
	for (size_t p = 0; p < (size_t)width * height; p++)
		out[p * 4 + 3] = 0xff;
	free(items);
	free(states);
	return 1;
}

/* draws the panel like a GUI would, writing a frame each second; with
   MPCTEST_LAYOUT also one composed from the layout description, and how
   much the two differ */
static void *draw_panel(void *param)
{
	job *const j = (job *)param;
	uint8_t *const pixels = calloc((size_t)j->pw * j->ph, 4);
	int const layout = getenv("MPCTEST_LAYOUT") != NULL;
	uint8_t *const composed = layout ? calloc((size_t)j->pw * j->ph, 4) : NULL;
	uint64_t serial = 0;
	uint32_t written = 0;
	struct timespec pause = { 0, 16000000 };
	while (!__atomic_load_n(&j->finished, __ATOMIC_ACQUIRE))
	{
		serial = mp_panel_frame(j->inst, pixels, j->pw, j->ph, serial);
		if (layout)
			mp_layout_describe(j->inst, NULL, NULL, 0, NULL, 0);   /* starts capturing */
		uint32_t const second = __atomic_load_n(&j->frames, __ATOMIC_ACQUIRE) / RATE;
		if (second != written)
		{
			write_ppm(j->panel, j->index, second, pixels, j->pw, j->ph);
			if (layout && compose_layout(j->inst, composed, j->pw, j->ph))
			{
				char prefix[1100];
				snprintf(prefix, sizeof(prefix), "%s.layout", j->panel);
				write_ppm(prefix, j->index, second, composed, j->pw, j->ph);
				size_t differing = 0, total = (size_t)j->pw * j->ph;
				int maxdiff = 0;
				for (size_t p = 0; p < total; p++)
				{
					int d = 0;
					for (int c = 0; c < 3; c++)
					{
						int const e = abs((int)pixels[p * 4 + c] - (int)composed[p * 4 + c]);
						d = e > d ? e : d;
					}
					differing += d > 8;
					maxdiff = d > maxdiff ? d : maxdiff;
				}
				fprintf(stderr, "[%d] layout at %us: %zu of %zu pixels differ by more than 8 (max %d)\n", j->index, second, differing, total, maxdiff);
			}
			written = second;
		}
		nanosleep(&pause, NULL);
	}
	free(composed);
	free(pixels);
	return NULL;
}

static void *render(void *param)
{
	job *const j = (job *)param;
	mp_instance *const inst = j->inst;

	FILE *wav = fopen(j->outpath, "wb");
	if (!wav)
	{
		perror(j->outpath);
		j->result = 1;
		return NULL;
	}
	uint32_t const total = (uint32_t)(j->seconds * RATE);
	uint32_t const blocks = (total + BLOCK - 1) / BLOCK;
	uint32_t const datasize = blocks * BLOCK * 4;
	fwrite("RIFF", 1, 4, wav); put32(wav, 36 + datasize); fwrite("WAVEfmt ", 1, 8, wav);
	put32(wav, 16); put16(wav, 1); put16(wav, 2); put32(wav, RATE); put32(wav, RATE * 4); put16(wav, 4); put16(wav, 16);
	fwrite("data", 1, 4, wav); put32(wav, datasize);

	/* after the machine had time to boot (MPCTEST_BOOT seconds, default 8),
	   play a note every 500ms: C major arpeggio */
	static const uint8_t notes[] = { 60, 64, 67, 72, 67, 64 };
	const char *bootenv = getenv("MPCTEST_BOOT");
	uint32_t const boot = (uint32_t)((bootenv ? atof(bootenv) : 8.0) * RATE), step = RATE / 2, length = RATE * 2 / 5;

	/* SysEx messages to send first */
	static uint8_t sysex[65536];
	size_t sysex_size = 0;
	uint32_t sysex_start[64], sysex_length[64], sysex_count = 0;
	const char *sysexenv = getenv("MPCTEST_SYSEX");
	if (sysexenv)
	{
		FILE *f = fopen(sysexenv, "rb");
		if (!f)
		{
			perror(sysexenv);
			j->result = 1;
			fclose(wav);
			return NULL;
		}
		sysex_size = fread(sysex, 1, sizeof(sysex), f);
		fclose(f);
		for (size_t i = 0; (i < sysex_size) && (sysex_count < 64); i++)
		{
			if (sysex[i] != 0xf0)
				continue;
			size_t end = i;
			while ((end < sysex_size) && (sysex[end] != 0xf7))
				end++;
			if (end == sysex_size)
				break;
			sysex_start[sysex_count] = (uint32_t)i;
			sysex_length[sysex_count++] = (uint32_t)(end + 1 - i);
			i = end;
		}
		fprintf(stderr, "[%d] %u SysEx messages\n", j->index, sysex_count);
	}
	uint32_t const sysex_step = RATE / 5;
	uint32_t const notes_start = boot + (sysex_count ? sysex_count * sysex_step + RATE : 0);
	const char *noteenv = getenv("MPCTEST_NOTE");
	int const held = noteenv ? atoi(noteenv) : -1;

	float left[MAX_BLOCK], right[MAX_BLOCK];
	float *out[2] = { left, right };
	uint8_t msgs[8][3];
	mp_midi_event events[8];
	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	j->panel = getenv("MPCTEST_PANEL");
	pthread_t panel_thread;
	double clicks[256][6];
	uint32_t click_count = 0;
	double const *click = NULL;   /* the click holding the button */
	uint32_t buttons = 0;
	if (j->panel)
	{
		const char *scaleenv = getenv("MPCTEST_PANEL_SCALE");
		uint32_t const scale = scaleenv ? (uint32_t)atoi(scaleenv) : 1;
		mp_panel_size(inst, &j->pw, &j->ph);
		j->pw *= scale;
		j->ph *= scale;
		fprintf(stderr, "[%d] panel %ux%u\n", j->index, j->pw, j->ph);
		mp_panel_set_size(inst, j->pw, j->ph);
		const char *clickenv = getenv("MPCTEST_CLICK");
		if (clickenv)
		{
			for (const char *c = clickenv; c && *c && (click_count < 256); c = strchr(c, ';') ? strchr(c, ';') + 1 : NULL)
			{
				double *const k = clicks[click_count];
				int const n = sscanf(c, "%lf,%lf,%lf,%lf,%lf,%lf", &k[0], &k[1], &k[2], &k[3], &k[4], &k[5]);
				if (n < 4)
					break;
				if (n < 6)
				{
					k[4] = k[0];
					k[5] = k[1];
				}
				click_count++;
			}
		}
		pthread_create(&panel_thread, NULL, draw_panel, j);
	}

	for (uint32_t b = 0; b < blocks; b++)
	{
		uint32_t const start = b * BLOCK;
		uint32_t count = 0;
		for (uint32_t f = start; (f < start + BLOCK) && (count < 8); f++)
		{
			if ((f >= boot) && (f < notes_start) && !((f - boot) % sysex_step) && ((f - boot) / sysex_step < sysex_count))
			{
				uint32_t const m = (f - boot) / sysex_step;
				events[count].frame = f - start;
				events[count].size = sysex_length[m];
				events[count].data = sysex + sysex_start[m];
				count++;
				continue;
			}
			if (f < notes_start)
				continue;
			if (held >= 0)
			{
				if (f == notes_start)
				{
					msgs[count][0] = 0x90;
					msgs[count][1] = (uint8_t)held;
					msgs[count][2] = 100;
					events[count].frame = f - start;
					events[count].size = 3;
					events[count].data = msgs[count];
					count++;
				}
				continue;
			}
			uint32_t const pos = (f - notes_start) % step;
			uint8_t const note = notes[((f - notes_start) / step) % sizeof(notes)];
			if ((pos == 0) || (pos == length))
			{
				msgs[count][0] = (pos == 0) ? 0x90 : 0x80;
				msgs[count][1] = note;
				msgs[count][2] = (pos == 0) ? 100 : 0;
				events[count].frame = f - start;
				events[count].size = 3;
				events[count].data = msgs[count];
				count++;
			}
		}

		mp_process(inst, events, count, out, BLOCK);

		__atomic_store_n(&j->frames, start + BLOCK, __ATOMIC_RELEASE);
		if (j->panel)
		{
			double const t = (double)start / RATE;
			double const *active = NULL;
			for (uint32_t c = 0; c < click_count; c++)
				if ((t >= clicks[c][2]) && (t < clicks[c][3]))
					active = clicks[c];
			if (active || buttons)
			{
				double const *const k = active ? active : click;
				double const f = active ? (t - k[2]) / (k[3] - k[2]) : 1.0;
				mp_panel_pointer(inst, 0, (int32_t)(k[0] + f * (k[4] - k[0])), (int32_t)(k[1] + f * (k[5] - k[1])), active != NULL);
				buttons = active != NULL;
				click = active;
			}
		}

		for (uint32_t i = 0; i < BLOCK; i++)
		{
			put16(wav, (uint16_t)to16(left[i]));
			put16(wav, (uint16_t)to16(right[i]));
		}

		const char *log = mp_log(inst);
		if (log)
			fprintf(stderr, "[%d] %s", j->index, log);

		if (j->realtime)
		{
			next.tv_nsec += (long)BLOCK * 1000000000L / RATE;
			while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
		}
	}
	fclose(wav);
	j->result = 0;
	__atomic_store_n(&j->finished, 1, __ATOMIC_RELEASE);
	if (j->panel)
		pthread_join(panel_thread, NULL);
	return NULL;
}

/* the machine's adjusters as parameter descriptions for the gearmulator
   plugin (parameterDescriptions_mame.json of a product), named as MAME
   names them */
static int write_fields(mp_instance *inst, const char *path)
{
	FILE *f = fopen(path, "w");
	if (!f)
	{
		perror(path);
		return 0;
	}
	uint32_t const count = mp_field_count(inst);
	fprintf(f, "{\n\t\"parameterdescriptiondefaults\":\n\t{\n\t\t\"page\": 0, \"class\": \"\", \"toText\": {\"format\": \"%%.0f\"}, \"isPublic\": true, \"isDiscrete\": false, \"isBool\": false, \"isBipolar\": false, \"step\": 0\n\t},\n");
	fprintf(f, "\t\"parameterdescriptions\":\n\t[\n");
	for (uint32_t i = 0; i < count; i++)
	{
		mp_field field;
		mp_field_info(inst, i, &field);
		fprintf(f, "\t\t{\"index\": %u, \"name\": \"%s\", \"min\": %d, \"max\": %d, \"default\": %d}%s\n",
				i, field.name, field.min, field.max, field.default_value, (i + 1 < count) ? "," : "");
	}
	fprintf(f, "\t],\n\t\"valuelists\":\n\t{\n\t}\n}\n");
	fclose(f);
	fprintf(stderr, "wrote %u fields to %s\n", count, path);
	return 1;
}

static uint32_t get32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }

/* splits the blob of mp_nvram() ("MPNV", version, count, then tag and
   data of each device) into files */
static int write_nvram(mp_instance *inst, const char *dir)
{
	size_t size = 0;
	const uint8_t *blob = mp_nvram(inst, &size);
	if (!blob || (size < 12) || memcmp(blob, "MPNV", 4))
	{
		fprintf(stderr, "no NVRAM\n");
		return 0;
	}
	const uint8_t *p = blob + 12, *const end = blob + size;
	for (uint32_t count = get32(blob + 8); count--; )
	{
		if (end - p < 4)
			return 0;
		uint32_t const taglen = get32(p);
		p += 4;
		if ((uint32_t)(end - p) < taglen + 4)
			return 0;
		char name[256];
		uint32_t n = 0;
		for (uint32_t i = (taglen && (p[0] == ':')) ? 1 : 0; (i < taglen) && (n < sizeof(name) - 1); i++)
			name[n++] = (p[i] == ':') ? '_' : (char)p[i];
		name[n] = 0;
		p += taglen;
		uint32_t const datalen = get32(p);
		p += 4;
		if ((uint32_t)(end - p) < datalen)
			return 0;
		char path[1400];
		snprintf(path, sizeof(path), "%s/%s", dir, name);
		FILE *f = fopen(path, "wb");
		if (!f || (fwrite(p, 1, datalen, f) != datalen))
		{
			perror(path);
			if (f)
				fclose(f);
			return 0;
		}
		fclose(f);
		fprintf(stderr, "wrote %s (%u bytes)\n", path, datalen);
		p += datalen;
	}
	return 1;
}

int main(int argc, char **argv)
{
	if (argc < 5)
	{
		fprintf(stderr, "usage: %s <library> <machine> <rompath> <out.wav> [seconds] [datapath] [--realtime]\n", argv[0]);
		return 1;
	}
	/* MPCTEST_OPTIONS=name=value;name=value passes extra MAME options */
	static char optbuf[4096];
	static const char *options[65];
	int optcount = 0;
	if (getenv("MPCTEST_OPTIONS"))
	{
		snprintf(optbuf, sizeof(optbuf), "%s", getenv("MPCTEST_OPTIONS"));
		for (char *item = strtok(optbuf, ";"); item && (optcount < 63); item = strtok(NULL, ";"))
		{
			char *const eq = strchr(item, '=');
			if (!eq)
				continue;
			*eq = 0;
			options[optcount++] = item;
			options[optcount++] = eq + 1;
		}
	}
	options[optcount] = NULL;

	if (getenv("MPCTEST_RATE"))
		RATE = (uint32_t)atoi(getenv("MPCTEST_RATE"));
	if (getenv("MPCTEST_BLOCK"))
		BLOCK = (uint32_t)atoi(getenv("MPCTEST_BLOCK"));
	if (!BLOCK || (BLOCK > MAX_BLOCK))
		BLOCK = 256;

	const char *libpath = argv[1];
	const char *machine = argv[2];
	const char *rompath = argv[3];
	const char *outpath = argv[4];
	double seconds = (argc > 5) ? atof(argv[5]) : 16.0;
	const char *datapath = (argc > 6) ? argv[6] : NULL;
	int realtime = (argc > 7) && !strcmp(argv[7], "--realtime");
	const char *instenv = getenv("MPCTEST_INSTANCES");
	int instances = instenv ? atoi(instenv) : 1;
	if ((instances < 1) || (instances > MAX_INSTANCES))
	{
		fprintf(stderr, "MPCTEST_INSTANCES must be between 1 and %d\n", MAX_INSTANCES);
		return 1;
	}

	void *lib = dlopen(libpath, RTLD_NOW | RTLD_LOCAL);
	if (!lib)
	{
		fprintf(stderr, "dlopen: %s\n", dlerror());
		return 1;
	}

	LOAD(mp_build_version)
	LOAD(mp_machine_count)
	LOAD(mp_machine_name)
	LOAD(mp_create)
	LOAD(mp_destroy)
	LOAD(mp_latency)
	LOAD(mp_process)
	LOAD(mp_set_offline)
	LOAD(mp_underruns)
	LOAD(mp_log)
	LOAD(mp_nvram)
	LOAD(mp_panel_size)
	LOAD(mp_panel_set_size)
	LOAD(mp_panel_frame)
	LOAD(mp_panel_pointer)
	LOAD(mp_layout_describe)
	LOAD(mp_layout_states)
	LOAD(mp_layout_element_image)
	LOAD(mp_layout_screen)
	LOAD(mp_field_count)
	LOAD(mp_field_info)

	printf("MAME %s, MIDI machines:", mp_build_version());
	for (uint32_t i = 0; i < mp_machine_count(); i++)
		printf(" %s", mp_machine_name(i));
	printf("\n");

	/* create all instances on the main thread, as a host would */
	static job jobs[MAX_INSTANCES];
	for (int i = 0; i < instances; i++)
	{
		job *const j = &jobs[i];
		j->index = i;
		j->seconds = seconds;
		j->realtime = realtime;
		if (instances == 1)
		{
			snprintf(j->outpath, sizeof(j->outpath), "%s", outpath);
			snprintf(j->datapath, sizeof(j->datapath), "%s", datapath ? datapath : "");
		}
		else
		{
			snprintf(j->outpath, sizeof(j->outpath), "%s.%d.wav", outpath, i);
			snprintf(j->datapath, sizeof(j->datapath), "%s.%d", datapath ? datapath : "data", i);
		}

		mp_config config;
		memset(&config, 0, sizeof(config));
		config.driver = machine;
		config.rompath = rompath;
		config.datapath = j->datapath[0] ? j->datapath : NULL;
		config.options = options[0] ? options : NULL;
		config.flags = getenv("MPCTEST_RELOAD_LAYOUT") ? MP_FLAG_RELOAD_LAYOUT : 0;
		config.sample_rate = RATE;
		config.max_block_frames = BLOCK;

		static char notes_text[1024];
		static mp_note_mapping note_map[64];
		const char *notesenv = getenv("MPCTEST_NOTES");
		if (notesenv && !config.note_map_size)
		{
			snprintf(notes_text, sizeof(notes_text), "%s", notesenv);
			for (char *entry = strtok(notes_text, ";"); entry && (config.note_map_size < 64); entry = strtok(NULL, ";"))
			{
				char *eq = strchr(entry, '=');
				if (!eq)
					continue;
				*eq = 0;
				note_map[config.note_map_size].note = (uint8_t)atoi(entry);
				note_map[config.note_map_size].min_velocity = 0;
				note_map[config.note_map_size].input = eq + 1;
				config.note_map_size++;
			}
		}
		config.note_map = config.note_map_size ? note_map : NULL;

		char error[1024];
		j->inst = mp_create(&config, error, sizeof(error));
		if (!j->inst)
		{
			fprintf(stderr, "mp_create %d failed:\n%s", i, error);
			return 1;
		}
		printf("instance %d: latency %u frames\n", i, mp_latency(j->inst));
		if (i == 0 && getenv("MPCTEST_FIELDS") && !write_fields(j->inst, getenv("MPCTEST_FIELDS")))
			return 1;
		mp_set_offline(j->inst, !realtime);
	}

	/* render concurrently, each instance on its own thread */
	pthread_t threads[MAX_INSTANCES];
	for (int i = 0; i < instances; i++)
		pthread_create(&threads[i], NULL, render, &jobs[i]);

	if (getenv("MPCTEST_NVRAM"))
	{
		unsigned snapshots = 0;
		size_t last = 0;
		for (int busy = 1; busy; )
		{
			busy = 0;
			for (int i = 0; i < instances; i++)
			{
				if (__atomic_load_n(&jobs[i].finished, __ATOMIC_ACQUIRE))
					continue;
				busy = 1;
				if (mp_nvram(jobs[i].inst, &last))
					snapshots++;
			}
			struct timespec const delay = { 0, 20 * 1000 * 1000 };
			nanosleep(&delay, NULL);
		}
		printf("nvram: %u snapshots, %zu bytes\n", snapshots, last);
	}
	int result = 0;
	for (int i = 0; i < instances; i++)
	{
		pthread_join(threads[i], NULL);
		result |= jobs[i].result;
		printf("instance %d: underruns %llu frames\n", i, (unsigned long long)mp_underruns(jobs[i].inst));
	}

	const char *nvramdir = getenv("MPCTEST_NVRAM_OUT");
	if (nvramdir && !write_nvram(jobs[0].inst, nvramdir))
		result = 1;

	for (int i = 0; i < instances; i++)
		mp_destroy(jobs[i].inst);
	dlclose(lib);
	return result;
}
