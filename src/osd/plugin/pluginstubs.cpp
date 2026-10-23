// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  pluginstubs.cpp - stand-ins for OSD modules and globals
//  that need an OS window or GUI toolkit and are therefore not
//  built for the plugin OSD
//
//============================================================

#include "emu.h"

#include "modules/debugger/debug_module.h"
#include "modules/monitor/monitor_common.h"
#include "modules/osdmodule.h"
#include "modules/osdwindow.h"
#include "modules/render/render_module.h"

// no windows are ever created, but osdwindow.cpp refers to this
osd_video_config video_config;

namespace osd { namespace { MODULE_NOT_SUPPORTED(video_bgfx, OSD_RENDERER_PROVIDER, "bgfx") } }
namespace osd { namespace { MODULE_NOT_SUPPORTED(debug_qt, OSD_DEBUG_PROVIDER, "qt") } }

namespace osd {

namespace {

// a single fixed monitor, so window-less video initialisation succeeds

class none_monitor_info : public osd_monitor_info
{
public:
	none_monitor_info(monitor_module &module) : osd_monitor_info(module, 1, "none", 4.0f / 3.0f)
	{
		refresh();
	}

	virtual void refresh() override
	{
		m_pos_size = osd_rect(0, 0, 640, 480);
		m_usuable_pos_size = m_pos_size;
		m_is_primary = true;
	}
};

class none_monitor_module : public monitor_module_base
{
public:
	none_monitor_module() : monitor_module_base(OSD_MONITOR_PROVIDER, "none") { }

	virtual std::shared_ptr<osd_monitor_info> monitor_from_rect(const osd_rect &rect) override { return monitor_from_handle(1); }
	virtual std::shared_ptr<osd_monitor_info> monitor_from_window(const osd_window &window) override { return monitor_from_handle(1); }

protected:
	virtual int init_internal(const osd_options &options) override
	{
		add_monitor(std::make_shared<none_monitor_info>(*this));
		return 0;
	}
};

} // anonymous namespace

} // namespace osd

MODULE_DEFINITION(RENDERER_BGFX, osd::video_bgfx)
MODULE_DEFINITION(DEBUG_QT, osd::debug_qt)
MODULE_DEFINITION(MONITOR_NONE, osd::none_monitor_module)
