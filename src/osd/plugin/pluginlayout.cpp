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
