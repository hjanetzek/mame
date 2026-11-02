// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  pluginlayout.cpp - the knobs and sliders of a layout, without
//  the Lua script layouts usually drive them with
//
//============================================================

#include "emu.h"

#include "pluginlayout.h"

#include "render.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>
#include <string>


namespace {

// dragging a knob over this part of the view's height goes through its
// range; item and pointer coordinates are normalised to the view
constexpr float KNOB_DRAG_RANGE = 0.25f;

// adjusters and analog inputs are what knobs and sliders change
ioport_field *control_field(std::pair<ioport_port *, ioport_value> const &input)
{
	if (!input.first || !input.second)
		return nullptr;
	ioport_field *const field = input.first->field(input.second);
	if (!field || ((field->type() != IPT_ADJUSTER) && !field->is_analog()) || field->analog_wraps())
		return nullptr;
	return field;
}

// the scripts of the layouts loaded on this thread: each machine loads its
// layouts on its emulation thread
thread_local std::vector<std::string> t_scripts;

} // anonymous namespace


void plugin_layout_controls::add_script(char const *script)
{
	t_scripts.emplace_back(script ? script : "");
}

void plugin_layout_controls::clear_scripts()
{
	t_scripts.clear();
}

namespace {

// an argument of a script's call: a string, possibly built with a loop
// variable ("slider_area_" .. i), or a number; anything else can't be read
struct script_argument
{
	std::vector<std::string> parts;   // the strings, with an empty one where the variable goes
	bool variable = false;
	bool readable = true;
	float number = 0.0f;
	bool is_number = false;

	std::string value(int index) const
	{
		std::string result;
		for (std::string const &part : parts)
			result += part.empty() ? std::to_string(index) : part;
		return result;
	}
};

std::vector<script_argument> read_arguments(std::string const &text)
{
	// split at the top level commas, outside strings
	std::vector<std::string> raw(1);
	bool quoted = false;
	for (char c : text)
	{
		if (c == '"')
			quoted = !quoted;
		if ((c == ',') && !quoted)
			raw.emplace_back();
		else
			raw.back() += c;
	}

	std::vector<script_argument> result;
	std::regex const token(R"re(\s*(?:"([^"]*)"|([A-Za-z_][A-Za-z0-9_]*)|([0-9.]+))\s*(\.\.)?)re");
	for (std::string const &argument : raw)
	{
		script_argument arg;
		std::string::const_iterator from = argument.begin();
		std::smatch match;
		while ((from != argument.end()) && std::regex_search(from, argument.end(), match, token, std::regex_constants::match_continuous))
		{
			if (match[1].matched)
				arg.parts.push_back(match[1].str());
			else if (match[2].matched)
			{
				arg.parts.emplace_back();
				arg.variable = true;
			}
			else
			{
				arg.number = std::stof(match[3].str());
				arg.is_number = true;
			}
			from = match[0].second;
		}
		arg.readable = (from == argument.end()) || std::all_of(from, argument.end(), [] (char c) { return std::isspace(u8(c)); });
		result.push_back(std::move(arg));
	}
	return result;
}

} // anonymous namespace

void plugin_layout_controls::find_script_controls(render_target &target, layout_view &view)
{
	// the calls of the "slider and knob" and "keys and sliders" libraries
	// that set up controls; their arguments name the items and the port:
	//   add_slider / add_sprung_slider / add_vertical_slider (view, area, knob, port)
	//   add_knob (view, area, port, scale, direction, dot)
	//   add_simplecounter_knob (view, area, port, scale)
	//   SliderHandler:create (view, area, knob, port, invert, autocenter)
	std::regex const call(R"re((add_sprung_slider|add_vertical_slider|add_slider|add_simplecounter_knob|add_knob|SliderHandler:create)\(\s*view\s*,([^()]*)\))re");

	std::map<std::string, layout_view_item *> items;
	for (layout_view_item &item : view.items())
		if (!item.id().empty())
			items.emplace(item.id(), &item);

	auto port_field = [&target] (std::string const &tag) -> ioport_field *
	{
		ioport_port *const port = target.manager().machine().root_device().ioport(tag);
		if (!port)
			return nullptr;
		for (ioport_field &field : port->fields())
			if (field.is_analog() || (field.type() == IPT_ADJUSTER))
				return &field;
		return nullptr;
	};

	for (std::string const &script : t_scripts)
	{
		for (auto it = std::sregex_iterator(script.begin(), script.end(), call); it != std::sregex_iterator(); ++it)
		{
			std::string const function = (*it)[1];
			std::vector<script_argument> const args = read_arguments((*it)[2]);
			bool const knob = (function == "add_knob") || (function == "add_simplecounter_knob");
			std::size_t const area_arg = 0, port_arg = knob ? 1 : 2;
			std::size_t const knob_arg = knob ? ((function == "add_knob") ? 4 : ~std::size_t(0)) : 1;
			std::size_t const scale_arg = knob ? 2 : ~std::size_t(0);
			if ((args.size() <= std::max(area_arg, port_arg)) || !args[area_arg].readable || !args[port_arg].readable || args[area_arg].parts.empty() || args[port_arg].parts.empty())
				continue;
			bool const variable = args[area_arg].variable || args[port_arg].variable || ((knob_arg < args.size()) && args[knob_arg].variable);

			// a loop's variable: the indices whose items and port exist
			for (int index = variable ? 0 : -1; index < (variable ? 100 : 0); index++)
			{
				std::string const area = args[area_arg].value(index);
				std::string const port = args[port_arg].value(index);
				std::string const moving = ((knob_arg < args.size()) && args[knob_arg].readable && !args[knob_arg].parts.empty()) ? args[knob_arg].value(index) : std::string();
				ioport_field *const field = port_field(port);
				if (!items.count(area) || !field || (!moving.empty() && !items.count(moving)))
					continue;
				float const scale = ((scale_arg < args.size()) && args[scale_arg].is_number) ? args[scale_arg].number : 0.0f;
				found_control const control{ u32(knob ? MP_CONTROL_KNOB : MP_CONTROL_SLIDER), area, moving, port, field, scale };

				// the script names the items better than the layout's bindings,
				// e.g. a slider's travel besides its knob
				auto const known = std::find_if(m_found.begin(), m_found.end(), [field] (found_control const &c) { return c.field == field; });
				if (known != m_found.end())
					*known = control;
				else
					m_found.push_back(control);
			}
		}
	}
}

void plugin_layout_controls::find_keys(render_target &target, layout_view &view)
{
	// KeyboardHandler:create(view, "<background>", "<key prefix>", "<port prefix>", ...)
	// of the "keys and sliders" library (the Ensoniq VFX family's layouts)
	std::regex const call(R"re(KeyboardHandler:create\(\s*view\s*,\s*"[^"]*"\s*,\s*"([^"]+)"\s*,\s*"([^"]+)")re");
	for (std::string const &script : t_scripts)
	{
		for (auto it = std::sregex_iterator(script.begin(), script.end(), call); it != std::sregex_iterator(); ++it)
		{
			std::string const prefix = (*it)[1];
			std::string const port_prefix = (*it)[2];
			std::size_t const keys_before = m_keys.size();
			for (layout_view_item &item : view.items())
			{
				if ((item.id().size() <= prefix.size()) || item.id().compare(0, prefix.size(), prefix))
					continue;
				if (std::any_of(m_keys.begin(), m_keys.end(), [&item] (key const &k) { return k.item == &item; }))
					continue;
				std::string const number = item.id().substr(prefix.size());
				if (number.find_first_not_of("0123456789") != std::string::npos)
					continue;
				ioport_port *const port = target.manager().machine().root_device().ioport(port_prefix + number);
				ioport_field *const field = port ? port->field(~ioport_value(0)) : nullptr;
				if (field && field->is_analog())
					m_keys.push_back(key{ &item, field });
			}
			if (m_keys.size() > keys_before)
				m_found.push_back(found_control{ MP_CONTROL_KEYBOARD, prefix, std::string(), port_prefix, nullptr, 0.0f });
		}
	}
}


void plugin_layout_controls::attach(render_target &target)
{
	layout_view &view = target.current_view();

	// the layout's script would hide these collections
	auto const &toggles = view.visibility_toggles();
	for (unsigned i = 0; i < toggles.size(); i++)
	{
		std::string name = toggles[i].name();
		std::transform(name.begin(), name.end(), name.begin(), [] (unsigned char c) { return std::tolower(c); });
		if (name.find("warning") != std::string::npos)
			target.set_visibility_toggle(i, false);
	}

	// and these items, which scripts set to state 0 (e.g. "plugin_warning")
	for (layout_view_item &item : view.items())
	{
		std::string id = item.id();
		std::transform(id.begin(), id.end(), id.begin(), [] (unsigned char c) { return std::tolower(c); });
		if (id.find("warning") != std::string::npos)
			item.set_state(0);
	}

	for (layout_view_item &item : view.items())
	{
		if (ioport_field *const field = control_field(item.animation_input_tag_and_mask()); field && item.bounds_animated())
		{
			render_bounds area = item.animation_bounds().front().bounds;
			for (auto const &step : item.animation_bounds())
				area |= step.bounds;
			m_controls.push_back(control{ &item, field, true, item.animation_input_shift(), area });
		}
		else if (ioport_field *const field = control_field(item.input_tag_and_mask()); field && !item.bounds_animated())
		{
			m_controls.push_back(control{ &item, field, false, 0, item.bounds() });
		}
	}
	// the controls for the host: those the layout binds to inputs, then those
	// its script sets up, then the keyboards
	m_found.clear();
	for (control const &c : m_controls)
	{
		std::string const tag = c.field->port().tag();
		m_found.push_back(found_control{ u32(c.slider ? MP_CONTROL_SLIDER : MP_CONTROL_KNOB), c.item->id(), c.slider ? c.item->id() : std::string(), tag, c.field, 0.0f });
	}
	find_script_controls(target, view);
	find_keys(target, view);
	if (m_controls.empty() && m_keys.empty())
		return;

	view.set_pointer_updated_callback(layout_view::pointer_updated_delegate(&plugin_layout_controls::pointer_updated, this));
	view.set_pointer_left_callback(layout_view::pointer_left_delegate(&plugin_layout_controls::pointer_left, this));
	view.set_pointer_aborted_callback(layout_view::pointer_left_delegate(&plugin_layout_controls::pointer_left, this));
	view.set_forget_pointers_callback(layout_view::forget_pointers_delegate(&plugin_layout_controls::forget_pointers, this));
}

void plugin_layout_controls::detach()
{
	forget_pointers();
	m_controls.clear();
	m_keys.clear();
	m_found.clear();
}

void plugin_layout_controls::release(grab const &grabbed)
{
	// a key lets go; a self-centering analog input (a pitch wheel) springs
	// back, others and adjusters keep their value
	if (grabbed.key)
		grabbed.key->set_value(0);
	else if (grabbed.grabbed->field->is_analog() && grabbed.grabbed->field->centerdelta())
		grabbed.grabbed->field->clear_value();
}

void plugin_layout_controls::pointer_updated(osd::ui_event_handler::pointer type, u16 id, u16 device, float x, float y, u32 buttons, u32 pressed, u32 released, s16 clicks)
{
	if (!BIT(buttons, 0))
	{
		pointer_left(type, id, device, x, y, released, clicks);
		return;
	}

	if (BIT(pressed, 0))
	{
		// sliders first: a knob's value display may lie within a slider's travel
		auto const hit = [x, y] (control const &c) { return c.area.includes(x, y); };
		auto found = std::find_if(m_controls.begin(), m_controls.end(), [&hit] (control const &c) { return c.slider && hit(c); });
		if (found == m_controls.end())
			found = std::find_if(m_controls.begin(), m_controls.end(), [&hit] (control const &c) { return !c.slider && hit(c); });
		if (found == m_controls.end())
		{
			// a key, the topmost (the black keys lie over the white ones);
			// lower on the key strikes it harder
			for (auto k = m_keys.rbegin(); k != m_keys.rend(); ++k)
			{
				render_bounds const bounds = k->item->bounds();
				if (!bounds.includes(x, y))
					continue;
				float const depth = (bounds.height() > 0.0f) ? ((y - bounds.y0) / bounds.height()) : 1.0f;
				k->field->set_value(ioport_value(std::clamp<s32>(s32(std::lround(16.0f + depth * 111.0f)), 1, 127)));
				m_grabs[id] = grab{ nullptr, y, 0, k->field };
				break;
			}
			return;
		}
		m_grabs[id] = grab{ &*found, y, value(*found->field) };
	}

	auto const grabbed = m_grabs.find(id);
	if ((grabbed == m_grabs.end()) || !grabbed->second.grabbed)
		return;
	control const &c = *grabbed->second.grabbed;

	s32 result;
	if (c.slider)
	{
		result = slider_value(c, x, y);
	}
	else
	{
		float const range = float(c.field->maxval() - c.field->minval());
		result = grabbed->second.start_value + s32(std::lround((grabbed->second.start_y - y) * range / KNOB_DRAG_RANGE));
	}
	set_value(*c.field, std::clamp<s32>(result, c.field->minval(), c.field->maxval()));
}

void plugin_layout_controls::pointer_left(osd::ui_event_handler::pointer type, u16 id, u16 device, float x, float y, u32 released, s16 clicks)
{
	auto const grabbed = m_grabs.find(id);
	if (grabbed == m_grabs.end())
		return;

	release(grabbed->second);
	m_grabs.erase(grabbed);
}

void plugin_layout_controls::forget_pointers()
{
	for (auto const &grabbed : m_grabs)
		release(grabbed.second);
	m_grabs.clear();
}

s32 plugin_layout_controls::value(ioport_field const &field)
{
	if (field.type() == IPT_ADJUSTER)
	{
		ioport_field::user_settings settings;
		field.get_user_settings(settings);
		return s32(settings.value);
	}
	return s32((field.port().read() & field.mask()) >> std::countr_zero(field.mask()));
}

void plugin_layout_controls::set_value(ioport_field &field, s32 value)
{
	if (field.type() == IPT_ADJUSTER)
	{
		ioport_field::user_settings settings;
		field.get_user_settings(settings);
		settings.value = ioport_value(value);
		field.set_user_settings(settings);
	}
	else
	{
		field.set_value(ioport_value(value));
	}
}

s32 plugin_layout_controls::slider_value(control const &c, float x, float y)
{
	// the position along the line through the centres of the first and last
	// state's bounds selects the state, which is the input's value
	auto const &steps = c.item->animation_bounds();
	auto const &first = steps.front();
	auto const &last = steps.back();
	float const x0 = (first.bounds.x0 + first.bounds.x1) * 0.5f, y0 = (first.bounds.y0 + first.bounds.y1) * 0.5f;
	float const dx = (last.bounds.x0 + last.bounds.x1) * 0.5f - x0, dy = (last.bounds.y0 + last.bounds.y1) * 0.5f - y0;
	float const length = dx * dx + dy * dy;
	float const t = (length > 0.0f) ? std::clamp(((x - x0) * dx + (y - y0) * dy) / length, 0.0f, 1.0f) : 0.0f;
	s32 const state = s32(first.state + t * (last.state - first.state) + 0.5f);
	return state << c.shift;
}
