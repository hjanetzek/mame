-- license:BSD-3-Clause
-- copyright-holders:MAMEdev Team

---------------------------------------------------------------------------
--
--   plugin.lua
--
--   Rules for building the headless audio plugin OSD
--
--   The emulator is built as a shared library exposing the C API in
--   src/osd/plugin/mame_plugin.h; audio and MIDI are exchanged with
--   the host through that API instead of OS devices.
--
---------------------------------------------------------------------------

dofile("modules.lua")

-- the plugin OSD talks to no OS devices, so disable everything optional
_OPTIONS["NO_OPENGL"] = "1"
_OPTIONS["NO_USE_MIDI"] = "1"
_OPTIONS["NO_USE_PORTAUDIO"] = "1"
_OPTIONS["NO_USE_PULSEAUDIO"] = "1"
_OPTIONS["NO_USE_PIPEWIRE"] = "1"
_OPTIONS["USE_QTDEBUG"] = "0"

BASE_TARGETOS       = "unix"
PLUGINOS_TARGETOS   = "unix"
if _OPTIONS["targetos"]=="windows" then
	BASE_TARGETOS       = "win32"
	PLUGINOS_TARGETOS   = "win32"
elseif _OPTIONS["targetos"]=="macosx" then
	PLUGINOS_TARGETOS   = "macosx"
end


-- name of the main project, the mp_* library is lib<"mame_" .. name>
function pluginLibName(_target, _subtarget)
	if (_OPTIONS["SOURCES"] == nil) and (_OPTIONS["SOURCEFILTER"] == nil) then
		return (_target == _subtarget) and _target or (_target .. _subtarget)
	end
	return _subtarget
end


function maintargetosdoptions(_target,_subtarget)
	osdmodulestargetconf()

	if BASE_TARGETOS=="unix" then
		links {
			"m",
			"pthread",
			"dl",
		}
	end

	if _OPTIONS["targetos"]=="linux" then
		-- export only the mp_* C API from the shared library
		linkoptions {
			"-Wl,--version-script=" .. MAME_DIR .. "src/osd/plugin/mame_plugin.map",
			"-Wl,-soname,libmame_" .. pluginLibName(_target, _subtarget) .. ".so",
		}
		-- report unresolved symbols at link time; sanitizer runtimes are
		-- provided by the host executable, so allow them to stay undefined
		if not _OPTIONS["SANITIZE"] then
			linkoptions {
				"-Wl,-z,defs",
			}
		end
	elseif _OPTIONS["targetos"]=="macosx" then
		linkoptions {
			"-Wl,-exported_symbols_list," .. MAME_DIR .. "src/osd/plugin/mame_plugin.exp",
		}
	elseif _OPTIONS["targetos"]=="windows" then
		links {
			"bcrypt",
			"psapi",
			"ole32",
			"winmm",
		}
	end
end


-- the CLAP plugin, a thin wrapper linked against the mp_* library
function pluginClapProject(_target, _subtarget)
	local libname = pluginLibName(_target, _subtarget)

	project (libname .. "_clap")
	uuid (os.uuid(_target .. "_" .. _subtarget .. "_clap"))
	kind "SharedLib"
	targetdir(MAME_DIR)
	targetprefix ""
	targetname ("mame_" .. libname)
	targetextension ".clap"

	addprojectflags()

	files {
		MAME_DIR .. "src/plugin/clap/mame_clap.cpp",
	}
	includedirs {
		MAME_DIR .. "src/osd/plugin",
		MAME_DIR .. "3rdparty/clap/include",
	}
	links {
		libname,
	}

	if _OPTIONS["targetos"]=="linux" then
		-- find the library next to the plugin, export only the CLAP entry
		linkoptions {
			"-Wl,-rpath,'$$ORIGIN'",
			"-Wl,--version-script=" .. MAME_DIR .. "src/plugin/clap/mame_clap.map",
		}
	elseif _OPTIONS["targetos"]=="macosx" then
		linkoptions {
			"-Wl,-rpath,@loader_path",
			"-Wl,-exported_symbols_list," .. MAME_DIR .. "src/plugin/clap/mame_clap.exp",
		}
	end
end


project ("osd_" .. _OPTIONS["osd"])
	uuid (os.uuid("osd_" .. _OPTIONS["osd"]))
	kind (LIBTYPE)

	dofile("plugin_cfg.lua")
	osdmodulesbuild()

	-- the BGFX renderer needs an OS window; pluginstubs.cpp replaces it
	removefiles {
		MAME_DIR .. "src/osd/modules/render/drawbgfx.cpp",
	}
	removefiles(os.matchfiles(MAME_DIR .. "src/osd/modules/render/bgfx/*.cpp"))

	includedirs {
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "src/osd/modules/file",
		MAME_DIR .. "src/osd/modules/render",
		MAME_DIR .. "3rdparty",
		MAME_DIR .. "src/osd/plugin",
	}

	files {
		MAME_DIR .. "src/osd/osdepend.h",
		MAME_DIR .. "src/osd/modules/osdwindow.cpp",
		MAME_DIR .. "src/osd/modules/osdwindow.h",
		MAME_DIR .. "src/osd/plugin/mame_plugin.h",
		MAME_DIR .. "src/osd/plugin/pluginlayout.cpp",
		MAME_DIR .. "src/osd/plugin/pluginlayout.h",
		MAME_DIR .. "src/osd/plugin/pluginmidi.h",
		MAME_DIR .. "src/osd/plugin/pluginosd.cpp",
		MAME_DIR .. "src/osd/plugin/pluginosd.h",
		MAME_DIR .. "src/osd/plugin/pluginstubs.cpp",
		MAME_DIR .. "src/osd/plugin/spscqueue.h",
	}

project ("ocore_" .. _OPTIONS["osd"])
	uuid (os.uuid("ocore_" .. _OPTIONS["osd"]))
	kind (LIBTYPE)

	removeflags {
		"SingleOutputDir",
	}

	dofile("plugin_cfg.lua")

	includedirs {
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		ext_includedir("asio"),
	}

	files {
		MAME_DIR .. "src/osd/asio.cpp",
		MAME_DIR .. "src/osd/asio.h",
		MAME_DIR .. "src/osd/osdcore.cpp",
		MAME_DIR .. "src/osd/osdcore.h",
		MAME_DIR .. "src/osd/osdfile.h",
		MAME_DIR .. "src/osd/strconv.cpp",
		MAME_DIR .. "src/osd/strconv.h",
		MAME_DIR .. "src/osd/osdsync.cpp",
		MAME_DIR .. "src/osd/osdsync.h",
		MAME_DIR .. "src/osd/modules/osdmodule.cpp",
		MAME_DIR .. "src/osd/modules/osdmodule.h",
		MAME_DIR .. "src/osd/modules/lib/osdlib_" .. PLUGINOS_TARGETOS .. ".cpp",
		MAME_DIR .. "src/osd/modules/lib/osdlib.h",
	}

	if BASE_TARGETOS=="unix" then
		files {
			MAME_DIR .. "src/osd/modules/file/posixdir.cpp",
			MAME_DIR .. "src/osd/modules/file/posixfile.cpp",
			MAME_DIR .. "src/osd/modules/file/posixfile.h",
			MAME_DIR .. "src/osd/modules/file/posixptty.cpp",
			MAME_DIR .. "src/osd/modules/file/posixsocket.cpp",
		}
	elseif BASE_TARGETOS=="win32" then
		includedirs {
			MAME_DIR .. "src/osd/windows",
		}
		files {
			MAME_DIR .. "src/osd/modules/file/windir.cpp",
			MAME_DIR .. "src/osd/modules/file/winfile.cpp",
			MAME_DIR .. "src/osd/modules/file/winfile.h",
			MAME_DIR .. "src/osd/modules/file/winptty.cpp",
			MAME_DIR .. "src/osd/modules/file/winsocket.cpp",
			MAME_DIR .. "src/osd/windows/winutil.cpp", -- FIXME put the necessary functions somewhere more appropriate?
			MAME_DIR .. "src/osd/windows/winutil.h",
		}
	else
		files {
			MAME_DIR .. "src/osd/modules/file/stdfile.cpp",
		}
	end
