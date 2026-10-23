-- license:BSD-3-Clause
-- copyright-holders:MAMEdev Team

dofile('modules.lua')

defines {
	"OSD_PLUGIN",
}

if _OPTIONS["targetos"]=="windows" then
	configuration { "mingw* or vs*" }
		defines {
			"UNICODE",
			"_UNICODE",
			"_WIN32_WINNT=0x0A00",
			"WIN32_LEAN_AND_MEAN",
			"NOMINMAX",
		}

	configuration { }
end

configuration { "osx*" }
	includedirs {
		MAME_DIR .. "3rdparty/bx/include/compat/osx",
	}

configuration { "freebsd" }
	includedirs {
		MAME_DIR .. "3rdparty/bx/include/compat/freebsd",
	}

configuration { "netbsd" }
	includedirs {
		MAME_DIR .. "3rdparty/bx/include/compat/freebsd",
	}

configuration { }
