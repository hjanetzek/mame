// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  pluginlayout.h - the knobs and sliders of a layout, without
//  the Lua script layouts usually drive them with
//
//  Knobs are items showing an adjuster or analog input, dragged
//  vertically; sliders are items animated by one, dragged along
//  their travel. Collections warning that the script isn't
//  running are hidden.
//
//============================================================
#ifndef MAME_OSD_PLUGIN_PLUGINLAYOUT_H
#define MAME_OSD_PLUGIN_PLUGINLAYOUT_H

#pragma once

#include "rendlay.h"

#include <map>
#include <vector>


class plugin_layout_controls
{
public:
	// attaches to the target's current view, once the machine's inputs are resolved
	void attach(render_target &target);

private:
	struct control
	{
		layout_view_item *item;
		ioport_field *field;
		bool slider;
		u8 shift;               // of the slider's animation state
		render_bounds area;     // where the slider can be grabbed: the bounds of all its states
	};

	struct grab
	{
		control const *grabbed;
		float start_y;
		s32 start_value;
	};

	void pointer_updated(osd::ui_event_handler::pointer type, u16 id, u16 device, float x, float y, u32 buttons, u32 pressed, u32 released, s16 clicks);
	void pointer_left(osd::ui_event_handler::pointer type, u16 id, u16 device, float x, float y, u32 released, s16 clicks);
	void forget_pointers();

	static s32 value(ioport_field const &field);
	static void set_value(ioport_field &field, s32 value);
	static s32 slider_value(control const &control, float x, float y);

	std::vector<control> m_controls;
	std::map<u16, grab> m_grabs;
};

#endif // MAME_OSD_PLUGIN_PLUGINLAYOUT_H
