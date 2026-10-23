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
 *   cc -O2 -o mpctest src/osd/plugin/test/mpctest.c -Isrc/osd/plugin -ldl -lpthread
 *   ./mpctest ./libmameplugin_synths.so tx81z roms out.wav [seconds] [datapath] [--realtime]
 */
#include "mame_plugin.h"

#include <dlfcn.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define RATE 48000
#define BLOCK 256

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
} job;

static void put32(FILE *f, uint32_t v) { fputc(v & 0xff, f); fputc((v >> 8) & 0xff, f); fputc((v >> 16) & 0xff, f); fputc(v >> 24, f); }
static void put16(FILE *f, uint16_t v) { fputc(v & 0xff, f); fputc(v >> 8, f); }

static int16_t to16(float v)
{
	if (v > 1.0f) v = 1.0f;
	if (v < -1.0f) v = -1.0f;
	return (int16_t)(v * 32767.0f);
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

	float left[BLOCK], right[BLOCK];
	float *out[2] = { left, right };
	uint8_t msgs[8][3];
	mp_midi_event events[8];
	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	for (uint32_t b = 0; b < blocks; b++)
	{
		uint32_t const start = b * BLOCK;
		uint32_t count = 0;
		for (uint32_t f = start; f < start + BLOCK; f++)
		{
			if (f < boot)
				continue;
			uint32_t const pos = (f - boot) % step;
			uint8_t const note = notes[((f - boot) / step) % sizeof(notes)];
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
	return NULL;
}

int main(int argc, char **argv)
{
	if (argc < 5)
	{
		fprintf(stderr, "usage: %s <library> <machine> <rompath> <out.wav> [seconds] [datapath] [--realtime]\n", argv[0]);
		return 1;
	}

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
		config.sample_rate = RATE;
		config.max_block_frames = BLOCK;

		char error[1024];
		j->inst = mp_create(&config, error, sizeof(error));
		if (!j->inst)
		{
			fprintf(stderr, "mp_create %d failed:\n%s", i, error);
			return 1;
		}
		printf("instance %d: latency %u frames\n", i, mp_latency(j->inst));
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

	for (int i = 0; i < instances; i++)
		mp_destroy(jobs[i].inst);
	dlclose(lib);
	return result;
}
