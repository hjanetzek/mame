// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  pluginosd.cpp - headless OSD for running a machine inside
//  an audio plugin
//
//============================================================

#include "emu.h"

#include "pluginosd.h"
#include "pluginmidi.h"

#include "fileio.h"
#include "imagedev/midiin.h"
#include "interface/midiport.h"

#include "render.h"
#include "rendersw.hxx"
#include "rendlay.h"

#include "osdfile.h"
#include "strformat.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <type_traits>
#include <unordered_map>
#include <vector>
#include <cstdio>


namespace {

// the panel renderer writes R, G, B, A bytes
using panel_renderer = std::conditional_t<
		std::endian::native == std::endian::little,
		software_renderer<u32, 0,0,0, 0,8,16>,
		software_renderer<u32, 0,0,0, 24,16,8> >;
constexpr u32 PANEL_ALPHA = (std::endian::native == std::endian::little) ? 0xff000000 : 0x000000ff;

// shortest time a button state is held, in emulated time
attotime const PANEL_MIN_HOLD = attotime::from_msec(50);

// most frames the panel is rendered per second
constexpr int PANEL_MAX_FPS = 60;

// how often layout development checks the layout files
constexpr int LAYOUT_CHECKS_PER_SECOND = 2;

// how often MIDI notes are turned into input changes, and the shortest time
// an input pressed by a note keeps a state: the firmware has to see it when
// it scans its keys
attotime const NOTE_PERIOD = attotime::from_hz(2000);
attotime const NOTE_MIN_HOLD = attotime::from_msec(10);


// stereo sink node presented to the sound manager
constexpr uint32_t PLUGIN_NODE_ID = 1;
constexpr uint32_t PLUGIN_STREAM_ID = 1;


//============================================================
//  plugin_midi_input - delivers the host's MIDI bytes once
//  the emulation reaches their due time
//============================================================

class plugin_midi_input : public osd::midi_input_port
{
public:
	plugin_midi_input(running_machine &machine, plugin_io &io) : m_machine(machine), m_io(io) { }

	virtual bool poll() override
	{
		plugin_io::midi_byte const *const next = m_io.midi.front();
		return next && (next->frame <= current_frame());
	}

	virtual int read(uint8_t *out) override
	{
		// midiin_device reads into a 32K buffer
		constexpr int MAX_BYTES = 8192 * 4;

		std::uint64_t const now = current_frame();
		int count = 0;
		for (plugin_io::midi_byte const *next = m_io.midi.front(); next && (next->frame <= now) && (count < MAX_BYTES); next = m_io.midi.front())
		{
			out[count++] = next->data;
			m_io.midi.pop(nullptr, 1);
		}
		return count;
	}

private:
	std::uint64_t current_frame() const { return m_io.frame(m_machine.time()); }

	running_machine &m_machine;
	plugin_io &m_io;
};


//============================================================
//  plugin_midi_output - passes the machine's MIDI output to
//  the host as complete messages, timestamped with the frame
//  at which their last byte was sent
//============================================================

class plugin_midi_output : public osd::midi_output_port
{
public:
	plugin_midi_output(running_machine &machine, plugin_io &io) : m_machine(machine), m_io(io) { }

	virtual void write(uint8_t data) override
	{
		m_assembler.write(
				data,
				[this] (uint8_t const *message, std::size_t size)
				{
					std::uint64_t const frame = m_io.frame(m_machine.time());
					if (!plugin_midi_record::push(m_io.midi_out, frame, message, size, m_record))
						m_io.midi_out_dropped.fetch_add(1, std::memory_order_relaxed);
				},
				[this] () { m_io.midi_out_dropped.fetch_add(1, std::memory_order_relaxed); });
	}

private:
	running_machine &m_machine;
	plugin_io &m_io;
	plugin_midi_assembler m_assembler;
	std::vector<uint8_t> m_record;
};


//============================================================
//  panel_fingerprint - identifies what a primitive list
//  shows, so unchanged frames needn't be drawn again
//============================================================

class panel_fingerprint
{
public:
	template <typename T> void add(T const &value) { add(&value, sizeof(value)); }

	void add(void const *data, std::size_t size)
	{
		u8 const *bytes = reinterpret_cast<u8 const *>(data);
		for (std::size_t i = 0; i < size; i++)
			m_hash = (m_hash ^ bytes[i]) * 0x100000001b3ULL;
	}

	u64 value() const { return m_hash; }

private:
	u64 m_hash = 0xcbf29ce484222325ULL;   // FNV-1a
};

u64 fingerprint(render_primitive_list &list)
{
	panel_fingerprint result;
	list.acquire_lock();
	for (render_primitive const &prim : list)
	{
		result.add(prim.type);
		result.add(prim.bounds);
		result.add(prim.color);
		result.add(prim.flags);
		render_texinfo const &tex = prim.texture;
		if (!tex.base)
			continue;
		result.add(tex.width);
		result.add(tex.height);
		if (!PRIMFLAG_GET_SCREENTEX(prim.flags))
		{
			// layout elements keep their image and sequence ID while the
			// scaled image is reused
			result.add(tex.base);
			result.add(tex.seqid);
			continue;
		}

		// screens get a new sequence ID each time and alternate between
		// two bitmaps, but are small on the machines this is for:
		// compare their pixels instead
		u32 const bytes = (PRIMFLAG_GET_TEXFORMAT(prim.flags) == TEXFORMAT_PALETTE16 || PRIMFLAG_GET_TEXFORMAT(prim.flags) == TEXFORMAT_YUY16) ? 2 : 4;
		u8 const *row = reinterpret_cast<u8 const *>(tex.base);
		for (u32 y = 0; y < tex.height; y++, row += std::size_t(tex.rowpixels) * bytes)
			result.add(row, std::size_t(tex.width) * bytes);
		if (tex.palette)
			result.add(tex.palette, std::size_t(tex.palette_length) * sizeof(rgb_t));
	}
	list.release_lock();
	return result.value();
}

} // anonymous namespace


//============================================================
//  plugin_io
//============================================================

void plugin_io::log(osd_output_channel channel, std::string const &text)
{
	if ((channel != OSD_OUTPUT_CHANNEL_ERROR) && (channel != OSD_OUTPUT_CHANNEL_WARNING))
		return;

	std::lock_guard<std::mutex> lock(log_mutex);
	log_text.append(text);
}


//============================================================
//  plugin_panel
//============================================================

std::uint64_t plugin_panel::draw(void *pixels, std::uint32_t width, std::uint32_t height, std::uint64_t last)
{
	std::unique_lock<std::mutex> lock(mutex);
	std::uint64_t const current = serial;
	if (!list || (current == last) || (list_width != width) || (list_height != height))
		return last;
	render_primitive_list &primitives = *list;
	std::unique_lock<std::mutex> drawing(draw_mutex);
	lock.unlock();

	// the list keeps the textures it uses alive, but the screens keep
	// updating their bitmaps, so a frame drawn late may show parts of a
	// newer one
	u32 *const dest = reinterpret_cast<u32 *>(pixels);
	std::fill_n(dest, std::size_t(width) * height, 0);
	primitives.acquire_lock();
	panel_renderer::draw_primitives(primitives, dest, width, height, width);
	primitives.release_lock();
	for (std::size_t i = 0; i < std::size_t(width) * height; i++)
		dest[i] |= PANEL_ALPHA;

	drawing.unlock();
	lock.lock();
	fetched = current;
	return current;
}

bool plugin_panel::draw_element(std::uint32_t element, std::uint32_t state, std::uint32_t orientation, std::uint32_t width, std::uint32_t height, void *rgba)
{
	std::unique_lock<std::mutex> lock(mutex);
	if ((element >= layout_element_ptrs.size()) || !width || !height)
		return false;
	layout_element &elem = *layout_element_ptrs[element];
	std::unique_lock<std::mutex> drawing(draw_mutex);
	lock.unlock();

	// the element is drawn as its texture, which an item with this
	// orientation shows rotated or flipped, as render_target does
	bool const swap = orientation & ORIENTATION_SWAP_XY;
	u32 const texwidth = swap ? height : width;
	u32 const texheight = swap ? width : height;
	bitmap_argb32 bitmap(texwidth, texheight);
	elem.draw_state(bitmap, int(state));

	u8 *dest = reinterpret_cast<u8 *>(rgba);
	for (u32 y = 0; y < height; y++)
	{
		for (u32 x = 0; x < width; x++)
		{
			u32 u = swap ? y : x;
			u32 v = swap ? x : y;
			if (orientation & (swap ? ORIENTATION_FLIP_Y : ORIENTATION_FLIP_X))
				u = texwidth - 1 - u;
			if (orientation & (swap ? ORIENTATION_FLIP_X : ORIENTATION_FLIP_Y))
				v = texheight - 1 - v;
			rgb_t const pixel = bitmap.pix(v, u);
			*dest++ = pixel.r();
			*dest++ = pixel.g();
			*dest++ = pixel.b();
			*dest++ = pixel.a();
		}
	}
	return true;
}

void plugin_panel::detach()
{
	{
		std::lock_guard<std::mutex> lock(mutex);
		list = nullptr;
	}
	std::lock_guard<std::mutex> drawing(draw_mutex);
}


//============================================================
//  plugin_osd_interface
//============================================================

plugin_osd_interface::plugin_osd_interface(osd_options &options, plugin_io &io)
	: osd_common_t(options)
	, m_io(io)
{
	std::fill(std::begin(m_note_held), std::end(m_note_held), -1);
}

plugin_osd_interface::~plugin_osd_interface()
{
}

void plugin_osd_interface::init(running_machine &machine)
{
	osd_common_t::init(machine);
	osd_common_t::init_subsystems();

	// there are no windows, but the core expects a UI target; it renders
	// the machine's layout for the plugin GUI
	create_panel_target();
	if (m_io.reload_layout)
		m_layout_files = layout_files_state();

	// a machine with a MIDI input reads the host's MIDI itself, other machines
	// can have notes press their inputs
	if (!m_io.note_map.empty() && !midiin_device_enumerator(machine.root_device()).first())
	{
		m_note_timer = machine.scheduler().timer_alloc(timer_expired_delegate(FUNC(plugin_osd_interface::update_notes), this));
		m_note_timer->adjust(NOTE_PERIOD, 0, NOTE_PERIOD);
	}
}

void plugin_osd_interface::create_panel_target()
{
	// a new target loads the layout files again
	m_target = machine().render().target_alloc();
	machine().render().set_ui_target(*m_target);

	// natural size: the view's aspect at no less than one pixel per
	// screen pixel
	s32 min_width, min_height;
	m_target->compute_minimum_size(min_width, min_height);
	float const aspect = m_target->current_view().effective_aspect();
	s32 width = s32(min_height * aspect + 0.5f);
	s32 height = min_height;
	if (width < min_width)
	{
		width = min_width;
		height = s32(min_width / aspect + 0.5f);
	}

	std::lock_guard<std::mutex> lock(m_io.panel.mutex);
	m_io.panel.natural_width = std::max(width, 1);
	m_io.panel.natural_height = std::max(height, 1);
}

std::string plugin_osd_interface::layout_files_state() const
{
	// the files render_target looks for in the art path: <machine>.lay or
	// default.lay in a folder named after the machine, or in a zip of that
	// name; their sizes and times tell whether one changed
	std::string result;
	char const *const name = machine().system().name;
	path_iterator paths(machine().options().art_path());
	std::string dir;
	while (paths.next(dir))
	{
		for (std::string const &file : {
				util::string_format("%s" PATH_SEPARATOR "%s" PATH_SEPARATOR "%s.lay", dir, name, name),
				util::string_format("%s" PATH_SEPARATOR "%s" PATH_SEPARATOR "default.lay", dir, name),
				util::string_format("%s" PATH_SEPARATOR "%s.zip", dir, name) })
		{
			osd::directory::entry::ptr const entry = osd_stat(file);
			if (entry)
				result += util::string_format("%s %d %d\n", file, entry->size, std::chrono::duration_cast<std::chrono::nanoseconds>(entry->last_modified.time_since_epoch()).count());
		}
	}
	return result;
}

void plugin_osd_interface::capture_layout()
{
	plugin_panel &panel = m_io.panel;
	layout_view &view = m_target->current_view();
	auto const &visible = view.visible_items();

	// describe the view again when it or its visible items change
	bool same = (&view == m_layout_view) && (visible.size() == m_layout_item_ptrs.size());
	for (std::size_t i = 0; same && (i < visible.size()); i++)
		same = &visible[i].get() == m_layout_item_ptrs[i];
	if (!same)
	{
		m_layout_view = &view;
		m_layout_item_ptrs.clear();
		m_layout_screen_ptrs.clear();
		std::vector<layout_element *> elements;
		std::vector<mp_layout_element> element_descs;
		std::vector<mp_layout_item> items;
		std::unordered_map<layout_element *, u32> element_index;
		for (layout_view_item &item : visible)
		{
			m_layout_item_ptrs.push_back(&item);
			// screens default to the blend mode of their own quads, which
			// replace what's below
			int const blend = (item.blend_mode() < 0) ? BLENDMODE_NONE : item.blend_mode();
			mp_layout_item desc{ MP_LAYOUT_NO_ELEMENT, 0, u32(blend), u32(item.orientation()), 0 };
			if (item.screen())
			{
				desc.screen = u32(m_layout_screen_ptrs.size());
				m_layout_screen_ptrs.push_back(dynamic_cast<screen_device *>(item.screen()));
			}
			else if (item.element())
			{
				auto const found = element_index.emplace(item.element(), u32(elements.size()));
				if (found.second)
				{
					elements.push_back(item.element());
					element_descs.push_back(mp_layout_element{ u32(item.element()->state_count()), u32(std::max(item.element()->default_state(), 0)) });
				}
				desc.element = found.first->second;
			}
			if (item.has_input())
				desc.flags |= MP_LAYOUT_CLICKABLE;
			if (item.clickthrough())
				desc.flags |= MP_LAYOUT_CLICKTHROUGH;
			if (item.bounds_animated() || item.color_animated())
				desc.flags |= MP_LAYOUT_ANIMATED;
			if (item.scroll_wrap_x() || item.scroll_wrap_y() || (item.scroll_size_x() < 1.0f) || (item.scroll_size_y() < 1.0f))
				desc.flags |= MP_LAYOUT_SCROLLED;
			items.push_back(desc);
		}
		m_layout_states.clear();
		m_layout_screen_images.assign(m_layout_screen_ptrs.size(), std::vector<u8>());

		release_layout_inputs();
		m_layout_serial = ++m_layout_counter;
		std::lock_guard<std::mutex> lock(panel.mutex);
		panel.layout_serial = m_layout_serial;
		panel.layout_aspect = view.effective_aspect();
		panel.layout_elements = std::move(element_descs);
		panel.layout_element_ptrs = std::move(elements);
		panel.layout_items = std::move(items);
		panel.layout_states.clear();
		panel.layout_screens.assign(m_layout_screen_ptrs.size(), plugin_panel::layout_screen());
	}

	// the items' states, bounds and colours, as their bindings give them
	std::vector<mp_layout_item_state> states;
	states.reserve(m_layout_item_ptrs.size());
	for (layout_view_item *item : m_layout_item_ptrs)
	{
		render_bounds const bounds = item->bounds();
		render_color const color = item->color();
		states.push_back(mp_layout_item_state{
				item->screen() ? 0 : std::max(item->element_state(), 0),
				bounds.x0, bounds.y0, bounds.x1, bounds.y1,
				color.a, color.r, color.g, color.b });
	}
	if ((states.size() != m_layout_states.size()) || std::memcmp(states.data(), m_layout_states.data(), states.size() * sizeof(states[0])))
	{
		m_layout_states = states;
		std::lock_guard<std::mutex> lock(panel.mutex);
		panel.layout_states = std::move(states);
		panel.layout_states_serial = ++m_layout_counter;
	}

	// the screens' visible areas, opaque, from the last completed frame
	// (the one the screen's texture shows, not the one being drawn)
	for (std::size_t i = 0; i < m_layout_screen_ptrs.size(); i++)
	{
		screen_device *const screen = m_layout_screen_ptrs[i];
		if (!screen)
			continue;
		screen_bitmap &bitmap = screen->curbitmap();
		if (!bitmap.valid())
			continue;
		rectangle const visarea = screen->visible_area() & bitmap.cliprect();
		u32 const width = visarea.width();
		u32 const height = visarea.height();
		std::vector<u8> image(std::size_t(width) * height * 4);
		u8 *dest = image.data();
		rgb_t const *const palette = ((bitmap.format() == BITMAP_FORMAT_IND16) && screen->has_palette()) ? screen->palette().palette()->entry_list_adjusted() : nullptr;
		for (s32 y = visarea.top(); y <= visarea.bottom(); y++)
		{
			for (s32 x = visarea.left(); x <= visarea.right(); x++)
			{
				rgb_t const pixel = (bitmap.format() == BITMAP_FORMAT_IND16)
						? (palette ? palette[bitmap.as_ind16().pix(y, x)] : rgb_t::black())
						: rgb_t(bitmap.as_rgb32().pix(y, x));
				*dest++ = pixel.r();
				*dest++ = pixel.g();
				*dest++ = pixel.b();
				*dest++ = 0xff;
			}
		}
		if (image == m_layout_screen_images[i])
			continue;
		m_layout_screen_images[i] = image;
		std::lock_guard<std::mutex> lock(panel.mutex);
		plugin_panel::layout_screen &shared = panel.layout_screens[i];
		shared.width = width;
		shared.height = height;
		shared.rgba = std::move(image);
		shared.serial = ++m_layout_counter;
	}
}

void plugin_osd_interface::layout_input(plugin_panel::layout_input_event const &event)
{
	if ((event.serial != m_layout_serial) || (event.item >= m_layout_item_ptrs.size()))
		return;
	auto const [port, mask] = m_layout_item_ptrs[event.item]->input_tag_and_mask();
	ioport_field *const field = (port && mask) ? port->field(mask) : nullptr;
	if (!field)
		return;

	auto found = std::find_if(m_layout_inputs.begin(), m_layout_inputs.end(), [field] (note_input const &input) { return input.field == field; });
	if (found == m_layout_inputs.end())
	{
		m_layout_inputs.push_back(note_input{ field });
		found = m_layout_inputs.end() - 1;
	}
	bool const last = found->pending.empty() ? found->pressed : found->pending.back();
	if (event.pressed != last)
		found->pending.push_back(event.pressed);
}

void plugin_osd_interface::release_layout_inputs()
{
	// the items of another description may not reach them anymore
	for (note_input &input : m_layout_inputs)
	{
		if (input.pressed)
			input.field->clear_value();
	}
	m_layout_inputs.clear();
}

void plugin_osd_interface::reload_layout()
{
	// the GUI must not draw a list or an element of the target that goes
	// away
	{
		std::lock_guard<std::mutex> lock(m_io.panel.mutex);
		std::lock_guard<std::mutex> drawing(m_io.panel.draw_mutex);
		m_io.panel.list = nullptr;
		m_io.panel.layout_element_ptrs.clear();
	}
	m_layout_view = nullptr;
	m_layout_item_ptrs.clear();
	m_layout_screen_ptrs.clear();
	release_layout_inputs();

	m_layout_controls.detach();
	m_layout_controls_attached = false;
	render_target *const previous = m_target;
	create_panel_target();
	m_target->resolve_tags();
	machine().render().target_free(previous);
	m_panel_fingerprint = 0;
	osd_printf_info("plugin: layout files changed, panel reloaded\n");
}

void plugin_osd_interface::update_notes(s32 param)
{
	// the inputs are set up after the OSD, so the names are looked up here
	if (m_note_map.empty())
	{
		for (plugin_io::note_mapping const &mapping : m_io.note_map)
		{
			ioport_field *found = nullptr;
			for (auto const &port : machine().ioport().ports())
			{
				for (ioport_field &field : port.second->fields())
				{
					if (!field.is_analog() && (field.name() == mapping.input))
						found = &field;
				}
			}
			if (!found)
			{
				osd_printf_warning("plugin: note %d: machine has no input \"%s\"\n", mapping.note, mapping.input);
				continue;
			}

			auto const known = std::find_if(m_note_inputs.begin(), m_note_inputs.end(), [found] (note_input const &input) { return input.field == found; });
			int const index = int(known - m_note_inputs.begin());
			if (known == m_note_inputs.end())
				m_note_inputs.push_back(note_input{ found });
			m_note_map.push_back(note_mapping{ mapping.note, mapping.min_velocity, index });
		}
		if (m_note_map.empty())
		{
			m_note_timer->enable(false);
			return;
		}
	}

	// the host's MIDI that is due
	std::uint64_t const now_frame = m_io.frame(machine().time());
	for (plugin_io::midi_byte const *next = m_io.midi.front(); next && (next->frame <= now_frame); next = m_io.midi.front())
	{
		std::uint8_t const data = next->data;
		m_io.midi.pop(nullptr, 1);
		m_note_assembler.write(data, [this] (std::uint8_t const *message, std::size_t size) { note_message(message, size); }, [] () { });
	}

	// apply the next state of each input once the current one was held long enough
	attotime const now = machine().time();
	for (note_input &input : m_note_inputs)
	{
		while (!input.pending.empty() && (input.pending.front() == input.pressed))
			input.pending.pop_front();
		if (input.pending.empty() || ((now - input.changed) < NOTE_MIN_HOLD))
			continue;

		input.pressed = input.pending.front();
		input.pending.pop_front();
		input.changed = now;
		input.field->set_value(input.pressed ? 1 : 0);
		input.field->port().frame_update();
	}
}

void plugin_osd_interface::note_message(std::uint8_t const *data, std::size_t size)
{
	// notes on any channel
	if ((size != 3) || ((data[0] & 0xe0) != 0x80))
		return;
	std::uint8_t const note = data[1];
	std::uint8_t const velocity = data[2];
	bool const on = ((data[0] & 0xf0) == 0x90) && velocity;

	int &held = m_note_held[note];
	if (held >= 0)
	{
		queue_input(held, false);
		held = -1;
	}
	if (!on)
		return;

	note_mapping const *best = nullptr;
	for (note_mapping const &mapping : m_note_map)
	{
		if ((mapping.note == note) && (mapping.min_velocity <= velocity) && (!best || (mapping.min_velocity > best->min_velocity)))
			best = &mapping;
	}
	if (!best)
		return;

	queue_input(best->input, true);
	held = best->input;
}

void plugin_osd_interface::queue_input(int input, bool pressed)
{
	// a press of an input that is down, by another note, releases it first
	std::deque<bool> &pending = m_note_inputs[input].pending;
	bool const last = pending.empty() ? m_note_inputs[input].pressed : pending.back();
	if (pressed && last)
		pending.push_back(false);
	if (pressed || last)
		pending.push_back(pressed);
}

void plugin_osd_interface::collect_fields()
{
	// ports are kept by tag, so the order is the same every time
	m_fields.clear();
	m_io.fields.clear();
	for (auto const &port : machine().ioport().ports())
	{
		for (ioport_field &field : port.second->fields())
		{
			if (field.type() != IPT_ADJUSTER)
				continue;
			m_fields.push_back(&field);
			m_io.fields.push_back(plugin_io::field_info{
					field.name(), port.second->tag(), field.mask(),
					s32(field.minval()), s32(field.maxval()), s32(field.defvalue()) });
		}
	}
	m_field_values.clear();
	update_fields();
}

void plugin_osd_interface::update_fields()
{
	// the host's changes, then the values, which the panel changes too
	std::vector<std::pair<u32, s32>> changes;
	{
		std::lock_guard<std::mutex> lock(m_io.field_mutex);
		changes.swap(m_io.field_changes);
	}
	for (auto const &change : changes)
	{
		if (change.first >= m_fields.size())
			continue;
		ioport_field &field = *m_fields[change.first];
		ioport_field::user_settings settings;
		field.get_user_settings(settings);
		settings.value = ioport_value(std::clamp<s32>(change.second, s32(field.minval()), s32(field.maxval())));
		field.set_user_settings(settings);
	}

	std::vector<s32> values;
	values.reserve(m_fields.size());
	for (ioport_field *field : m_fields)
	{
		ioport_field::user_settings settings;
		field->get_user_settings(settings);
		values.push_back(s32(settings.value));
	}
	if (values == m_field_values)
		return;
	m_field_values = values;
	std::lock_guard<std::mutex> lock(m_io.field_mutex);
	m_io.field_values = std::move(values);
	m_io.field_serial++;
}

void plugin_osd_interface::update(bool skip_redraw)
{
	osd_common_t::update(skip_redraw);
	update_panel();
}

void plugin_osd_interface::update_panel()
{
	update_fields();

	// in layout development, files in the art path that change are
	// loaded again, without restarting the machine
	if (m_io.reload_layout && (machine().phase() == machine_phase::RUNNING))
	{
		osd_ticks_t const now = osd_ticks();
		if ((now - m_layout_checked) >= (osd_ticks_per_second() / LAYOUT_CHECKS_PER_SECOND))
		{
			m_layout_checked = now;
			std::string state = layout_files_state();
			if (state != m_layout_files)
			{
				m_layout_files = std::move(state);
				reload_layout();
			}
		}
	}

	plugin_panel &panel = m_io.panel;
	u32 width, height, natural_width, natural_height;
	bool consumed, layout_requested;
	{
		std::lock_guard<std::mutex> lock(panel.mutex);
		width = panel.width;
		height = panel.height;
		natural_width = panel.natural_width;
		natural_height = panel.natural_height;
		consumed = panel.fetched == panel.serial;
		layout_requested = panel.layout_requested;
		m_layout_input_events.insert(m_layout_input_events.end(), panel.layout_inputs.begin(), panel.layout_inputs.end());
		panel.layout_inputs.clear();
		m_pointer_events.insert(m_pointer_events.end(), panel.pointers.begin(), panel.pointers.end());
		panel.pointers.clear();
	}

	// the layout's knobs and sliders, once the inputs they show are
	// resolved; also for a host that draws the layout itself, as attaching
	// hides the warnings of layout scripts
	if (!m_layout_controls_attached && (machine().phase() == machine_phase::RUNNING))
	{
		m_layout_controls.attach(*m_target);
		m_layout_controls_attached = true;
	}

	// the layout for a host that draws the panel itself, at the rate
	// frames are built
	if (layout_requested && (machine().phase() == machine_phase::RUNNING))
	{
		osd_ticks_t const now = osd_ticks();
		if ((now - m_layout_captured) >= (osd_ticks_per_second() / PANEL_MAX_FPS))
		{
			m_layout_captured = now;
			capture_layout();
		}
	}

	// inputs pressed through the description, each state held a while
	for (plugin_panel::layout_input_event const &event : m_layout_input_events)
		layout_input(event);
	m_layout_input_events.clear();
	attotime const emulated = machine().time();
	for (note_input &input : m_layout_inputs)
	{
		while (!input.pending.empty() && (input.pending.front() == input.pressed))
			input.pending.pop_front();
		if (input.pending.empty() || ((emulated - input.changed) < PANEL_MIN_HOLD))
			continue;
		input.pressed = input.pending.front();
		input.pending.pop_front();
		input.changed = emulated;
		input.field->set_value(input.pressed ? 1 : 0);
		input.field->port().frame_update();
	}

	// a host drawing the layout itself gives pointers in view coordinates;
	// without a frame size the target keeps the layout's natural size to
	// map them, and builds no frames
	bool const framed = width && height;
	if (!framed)
	{
		if (!layout_requested)
		{
			m_pointer_events.clear();
			return;
		}
		width = natural_width;
		height = natural_height;
		if (!width || !height)
			return;
	}
	while (m_pointer_events.size() > plugin_panel::MAX_POINTER_EVENTS)
		m_pointer_events.pop_front();

	bool const resized = (s32(width) != m_target->width()) || (s32(height) != m_target->height());
	if (resized)
		m_target->set_bounds(width, height);

	// view coordinates go through the visible area the target maps its
	// pixels with, which can be smaller than the target
	s32 viswidth, visheight;
	m_target->compute_visible_area(width, height, m_target->pixel_aspect(), m_target->orientation(), viswidth, visheight);
	float const visx = float(s32(width) - viswidth) / 2, visy = float(s32(height) - visheight) / 2;

	// clicks on the layout's controls; the inputs are only looked at once
	// a frame and machines scan them even less often, so each button state
	// is held for a while before the next change is applied
	bool updated = false;
	attotime const now_emulated = machine().time();
	while (!m_pointer_events.empty())
	{
		plugin_panel::pointer_event const &event = m_pointer_events.front();
		if (m_pointer_buttons.size() <= event.id)
		{
			m_pointer_buttons.resize(event.id + 1, 0);
			m_pointer_changed.resize(event.id + 1, attotime::zero);
		}
		u32 const previous = m_pointer_buttons[event.id];
		u32 const buttons = event.leave ? 0 : event.buttons;
		s32 const x = event.view ? s32(std::lround(visx + event.vx * float(viswidth))) : event.x;
		s32 const y = event.view ? s32(std::lround(visy + event.vy * float(visheight))) : event.y;
		if (buttons != previous)
		{
			if ((now_emulated - m_pointer_changed[event.id]) < PANEL_MIN_HOLD)
				break;
			m_pointer_changed[event.id] = now_emulated;
		}

		if (event.leave)
		{
			m_target->pointer_left(osd::ui_event_handler::pointer::MOUSE, event.id, 0, x, y, previous, 0);
		}
		else
		{
			u32 const pressed = buttons & ~previous;
			u32 const released = previous & ~buttons;
			m_target->pointer_updated(osd::ui_event_handler::pointer::MOUSE, event.id, 0, x, y, buttons, pressed, released, BIT(pressed, 0) ? 1 : 0);
		}
		m_pointer_buttons[event.id] = buttons;
		m_pointer_events.pop_front();
		updated = true;
	}
	if (updated)
		m_target->update_pointer_fields();

	if (!framed)
		return;

	// build a new list when the GUI drew the last one, at a limited rate;
	// the target alternates between two lists, so the one rebuilt here is
	// never the one the GUI may still be drawing
	osd_ticks_t const now = osd_ticks();
	if (!resized && (!consumed || ((now - m_panel_last) < (osd_ticks_per_second() / PANEL_MAX_FPS))))
		return;
	m_panel_last = now;

	// never rebuild a list while the GUI draws one; try again next time
	// rather than hold up the emulation
	std::unique_lock<std::mutex> drawing(panel.draw_mutex, std::try_to_lock);
	if (!drawing)
		return;
	render_primitive_list &list = m_target->get_primitives();

	// drawing the panel in software is costly, so the GUI is only told
	// about a frame that shows something new; the list is passed on
	// anyway, so the one the GUI can draw is never the one rebuilt next
	u64 const shown = fingerprint(list);
	bool const changed = resized || (shown != m_panel_fingerprint);
	m_panel_fingerprint = shown;

	// draw() takes draw_mutex while holding mutex
	drawing.unlock();
	std::lock_guard<std::mutex> lock(panel.mutex);
	panel.list = &list;
	panel.list_width = width;
	panel.list_height = height;
	if (changed)
		panel.serial++;
}

osd::audio_info plugin_osd_interface::sound_get_information()
{
	osd::audio_info result;
	result.m_generation = 1;
	result.m_default_sink = PLUGIN_NODE_ID;
	result.m_default_source = 0;
	result.m_nodes.resize(1);

	osd::audio_info::node_info &node = result.m_nodes[0];
	node.m_name = "plugin";
	node.m_display_name = "Plugin output";
	node.m_id = PLUGIN_NODE_ID;
	node.m_rate = osd::audio_rate_range{ m_io.sample_rate, m_io.sample_rate, m_io.sample_rate };
	node.m_port_names = { "FL", "FR" };
	node.m_port_positions = { osd::channel_position::FL(), osd::channel_position::FR() };
	node.m_sinks = 2;
	node.m_sources = 0;
	return result;
}

uint32_t plugin_osd_interface::sound_stream_sink_open(uint32_t node, std::string name, uint32_t rate)
{
	if ((node != PLUGIN_NODE_ID) || (rate != m_io.sample_rate))
	{
		osd_printf_error("plugin: unexpected sound stream %s on node %u at %u Hz\n", name, node, rate);
		return 0;
	}
	return PLUGIN_STREAM_ID;
}

void plugin_osd_interface::sound_stream_sink_update(uint32_t id, const int16_t *buffer, int samples_this_frame)
{
	if (id != PLUGIN_STREAM_ID)
		return;

	std::size_t remaining = std::size_t(samples_this_frame) * 2;
	while (remaining)
	{
		std::size_t const chunk = std::min(remaining, std::size(m_convert));
		for (std::size_t i = 0; i < chunk; i++)
			m_convert[i] = float(buffer[i]) * (1.0f / 32768.0f);
		if (m_io.audio.push(m_convert, chunk) != chunk)
			osd_printf_verbose("plugin: audio queue overflow\n");
		buffer += chunk;
		remaining -= chunk;
	}

	// should the sound system deliver less audio than the emulated time,
	// the output frames run behind by as much, and the emulation makes up
	// for it instead of leaving the host short of audio for good
	m_io.audio_frames += samples_this_frame;
	m_io.audio_owed = s64(machine().time().as_ticks(m_io.sample_rate)) - s64(m_io.audio_frames);
}

std::unique_ptr<osd::midi_input_port> plugin_osd_interface::create_midi_input(std::string_view name)
{
	if (name != PLUGIN_MIDI_PORT_NAME)
		return nullptr;
	return std::make_unique<plugin_midi_input>(machine(), m_io);
}

std::unique_ptr<osd::midi_output_port> plugin_osd_interface::create_midi_output(std::string_view name)
{
	if (name != PLUGIN_MIDI_PORT_NAME)
		return nullptr;
	return std::make_unique<plugin_midi_output>(machine(), m_io);
}

std::vector<osd::midi_port_info> plugin_osd_interface::list_midi_ports()
{
	return { osd::midi_port_info{ PLUGIN_MIDI_PORT_NAME, true, true, true, true } };
}

void plugin_osd_interface::output_callback(osd_output_channel channel, const util::format_argument_pack<char> &args)
{
	if ((channel == OSD_OUTPUT_CHANNEL_VERBOSE) && !verbose())
		return;
	if (channel == OSD_OUTPUT_CHANNEL_DEBUG)
		return;

	std::string const text = util::string_format(args);
	m_io.log(channel, text);

	// mirror to stderr for hosts that capture it
	std::fputs(text.c_str(), stderr);
}
