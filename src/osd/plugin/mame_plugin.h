/* license:BSD-3-Clause
 * copyright-holders:Hannes Janetzek
 *
 * mame_plugin.h - C API of the MAME audio plugin library
 *
 * The library built with OSD=plugin runs one emulated machine per
 * instance on its own emulation thread.  The host feeds MIDI and pulls
 * audio through mp_process(), which is safe to call from a realtime
 * audio thread: it neither locks nor allocates.
 *
 * The emulation runs ahead of the host by mp_latency() frames; MIDI
 * events are delayed by the same amount so their timing relative to
 * the audio is preserved.  Hosts should report this latency for
 * delay compensation.
 *
 * NVRAM (e.g. a synthesizer's patch memory) is persisted by the host:
 * mp_nvram() returns its current contents, which can be passed back in
 * mp_config when the machine is created again.  NVRAM files in the data
 * path are only read, as initial contents for machines created without
 * NVRAM data, and never written.
 *
 * Layout files (MAME artwork) are looked for in the data path's parent
 * folder, in a folder or zip named after the machine: with the data path
 * <root>/<machine>, <root>/<machine>/default.lay or <machine>.lay is
 * shown instead of the layout compiled into the library.  With
 * MP_FLAG_RELOAD_LAYOUT they are loaded again whenever they change.
 */
#ifndef MAME_OSD_PLUGIN_MAME_PLUGIN_H
#define MAME_OSD_PLUGIN_MAME_PLUGIN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#define MP_API __declspec(dllexport)
#else
#define MP_API __attribute__((visibility("default")))
#endif

#define MP_API_VERSION 8

typedef struct mp_instance mp_instance;

/* for machines without a MIDI input: a MIDI note pressing one of the
   machine's inputs while it is held */
typedef struct mp_note_mapping
{
	uint8_t note;
	uint8_t min_velocity;        /* of the mappings of a note, the one with the highest min_velocity not above the velocity is used */
	const char *input;           /* name of an input of the machine, e.g. "SNARE 1" */
} mp_note_mapping;

typedef struct mp_config
{
	const char *driver;          /* short machine name, e.g. "tx81z" */
	const char *rompath;         /* ';'-separated ROM search path */
	const char *datapath;        /* directory for nvram/cfg/state files, NULL for current directory */
	uint32_t sample_rate;        /* host sample rate in Hz */
	uint32_t max_block_frames;   /* largest frame count passed to mp_process */
	uint32_t latency_frames;     /* look-ahead of the emulation, 0 selects a default */
	const char *const *options;  /* NULL-terminated name/value pairs of extra MAME options, may be NULL */
	const void *nvram;           /* NVRAM contents from mp_nvram() to start with, may be NULL */
	size_t nvram_size;
	const mp_note_mapping *note_map;   /* notes that press inputs of a machine without MIDI input, may be NULL */
	uint32_t note_map_size;
	uint32_t flags;              /* MP_FLAG_* */
} mp_config;

/* mp_config::flags */
#define MP_FLAG_RELOAD_LAYOUT 1  /* layout development: load the panel's layout files again when they change */

typedef struct mp_midi_event
{
	uint32_t frame;              /* offset into the current block */
	uint32_t size;               /* number of bytes, may be a complete SysEx message */
	const uint8_t *data;
} mp_midi_event;

/* library information */
MP_API uint32_t mp_api_version(void);
MP_API const char *mp_build_version(void);

/* machines in this library that have a MIDI input */
MP_API uint32_t mp_machine_count(void);
MP_API const char *mp_machine_name(uint32_t index);
MP_API const char *mp_machine_description(uint32_t index);

/* create a machine instance; on failure returns NULL and writes a
   message to error (if not NULL) */
MP_API mp_instance *mp_create(const mp_config *config, char *error, size_t error_size);
MP_API void mp_destroy(mp_instance *instance);

/* look-ahead in frames */
MP_API uint32_t mp_latency(const mp_instance *instance);

/* realtime: change the look-ahead, e.g. when the host's block size
   changes; returns the latency in effect, which is limited to 32768 */
MP_API uint32_t mp_set_latency(mp_instance *instance, uint32_t frames);

/* realtime: queue the block's MIDI events (sorted by frame) and fill
   frames of stereo audio into out[0] and out[1] */
MP_API void mp_process(mp_instance *instance, const mp_midi_event *events, uint32_t event_count, float *const *out, uint32_t frames);

/* realtime: after mp_process, fetch the MIDI messages the machine sent
   during that block (with frame offsets into it), storing their bytes in
   buffer; returns the number of events.  Messages that don't fit stay
   queued for the next call, a single message larger than buffer is
   dropped. */
MP_API uint32_t mp_midi_output(mp_instance *instance, mp_midi_event *events, uint32_t max_events, uint8_t *buffer, size_t buffer_size);

/* offline mode: mp_process waits for the emulation instead of
   outputting silence on underrun, producing deterministic output */
MP_API void mp_set_offline(mp_instance *instance, int offline);

/* number of frames that were output as silence because the emulation
   could not keep up */
MP_API uint64_t mp_underruns(const mp_instance *instance);

/* main thread: snapshot the NVRAM of all devices; returns NULL if the
   machine has none or the snapshot failed.  The data stays valid until
   the next call for this instance.  After the machine stopped, returns
   the contents at the time it stopped. */
MP_API const void *mp_nvram(mp_instance *instance, size_t *size);

/* front panel: the machine's layout (artwork, screens, LEDs and
   clickable controls) as MAME draws it.  These calls may be made from
   any thread but the realtime one; they take a lock shared with the
   emulation thread.  The emulation thread prepares frames while the
   host keeps calling mp_process, mp_panel_frame draws them on the
   calling thread, which should always be the same one. */

/* natural size of the layout in pixels */
MP_API void mp_panel_size(mp_instance *instance, uint32_t *width, uint32_t *height);

/* render at this size from now on, 0 stops rendering */
MP_API void mp_panel_set_size(mp_instance *instance, uint32_t width, uint32_t height);

/* draw the latest frame as R, G, B, A bytes (width * height * 4) if it
   is newer than the one with the given serial and has the given size;
   returns the serial of the drawn frame, or the passed serial if there
   was nothing new */
MP_API uint64_t mp_panel_frame(mp_instance *instance, void *pixels, uint32_t width, uint32_t height, uint64_t serial);

/* pointer input in pixels of the size passed to mp_panel_set_size;
   buttons has bit 0 set while the primary button is down.  Pointer ids
   are below 64. */
MP_API void mp_panel_pointer(mp_instance *instance, uint32_t id, int32_t x, int32_t y, uint32_t buttons);
MP_API void mp_panel_pointer_leave(mp_instance *instance, uint32_t id);

/* Layout: the machine's layout as MAME resolved it, for a host that draws
   the panel itself.  The current view is a list of items in drawing
   order; each shows an element (whose image per state the library draws)
   or a screen.  The emulation thread captures the items' states, bounds
   and colours and the screens' pixels while the host calls mp_process;
   these calls take the same lock as the panel calls and should be made
   from one thread.  Capturing starts with the first mp_layout_describe
   call.  Images are R, G, B, A bytes, not premultiplied. */

/* mp_layout_item::flags */
#define MP_LAYOUT_CLICKABLE    1   /* clicking it presses an input */
#define MP_LAYOUT_CLICKTHROUGH 2   /* clicks also reach the items below */
#define MP_LAYOUT_ANIMATED     4   /* its bounds or colour change with its state */
#define MP_LAYOUT_SCROLLED     8   /* shows a scrolled window of its element: not described here */

/* blend modes, as MAME's */
#define MP_LAYOUT_BLEND_NONE     0   /* replace */
#define MP_LAYOUT_BLEND_ALPHA    1
#define MP_LAYOUT_BLEND_MULTIPLY 2   /* multiply RGB with the image's, after applying its alpha */
#define MP_LAYOUT_BLEND_ADD      3   /* add the image's RGB times its alpha */

#define MP_LAYOUT_NO_ELEMENT 0xffffffffu

typedef struct mp_layout_info
{
	uint64_t serial;             /* changes with the view or the items shown */
	float aspect;                /* width / height of the view */
	uint32_t element_count;
	uint32_t item_count;
	uint32_t screen_count;
} mp_layout_info;

typedef struct mp_layout_element
{
	uint32_t state_count;        /* states are 0 .. state_count - 1 */
	uint32_t default_state;
} mp_layout_element;

typedef struct mp_layout_item
{
	uint32_t element;            /* index of its element, MP_LAYOUT_NO_ELEMENT for a screen */
	uint32_t screen;             /* index of its screen, if it shows one */
	uint32_t blend;              /* MP_LAYOUT_BLEND_* */
	uint32_t orientation;        /* MAME's ORIENTATION_* bits: 1 flip x, 2 flip y, 4 swap x and y */
	uint32_t flags;              /* MP_LAYOUT_* */
} mp_layout_item;

typedef struct mp_layout_item_state
{
	int32_t state;               /* the element's state to show */
	float x0, y0, x1, y1;        /* bounds, 0..1 across the view */
	float a, r, g, b;            /* colour multiplied with the image */
} mp_layout_item_state;

/* the current view: fills info and as many elements and items as fit,
   returns the serial (also in info->serial); call with NULL arrays to get
   the counts first */
MP_API uint64_t mp_layout_describe(mp_instance *instance, mp_layout_info *info, mp_layout_element *elements, uint32_t max_elements, mp_layout_item *items, uint32_t max_items);

/* the items' current states, in the order of mp_layout_describe, if they
   differ from those with the given serial; returns their serial, or the
   passed one if nothing changed.  *layout_serial receives the serial of
   the description they belong to. */
MP_API uint64_t mp_layout_states(mp_instance *instance, uint64_t serial, mp_layout_item_state *states, uint32_t max_states, uint64_t *layout_serial);

/* draws an element in a state at width x height pixels, oriented as an
   item with that orientation shows it; returns 0 if there is no such
   element */
MP_API int mp_layout_element_image(mp_instance *instance, uint32_t element, uint32_t state, uint32_t orientation, uint32_t width, uint32_t height, void *rgba);

/* a screen's current image at its own size, if it differs from the one
   with the given serial and fits size bytes; *width and *height receive
   its size either way.  Returns its serial, or the passed one. */
MP_API uint64_t mp_layout_screen(mp_instance *instance, uint32_t screen, uint64_t serial, uint32_t *width, uint32_t *height, void *rgba, size_t size);

/* pointer input in view coordinates (0..1 across the view, as the items'
   bounds), handled like mp_panel_pointer: MAME presses clickable items and
   the layout's knobs and sliders follow drags.  mp_panel_pointer_leave
   ends it. */
MP_API void mp_layout_pointer(mp_instance *instance, uint32_t id, float x, float y, uint32_t buttons);

/* presses or releases the input of a clickable item of the description
   with the given serial (ignored for another one); each state is held for
   at least 50 ms of emulated time */
MP_API void mp_layout_input(mp_instance *instance, uint64_t layout_serial, uint32_t item, int pressed);

/* drain and return messages logged by the emulation (errors and
   warnings), or NULL if there are none; the pointer is valid until the
   next call for this instance */
MP_API const char *mp_log(mp_instance *instance);

#ifdef __cplusplus
}
#endif

#endif /* MAME_OSD_PLUGIN_MAME_PLUGIN_H */
