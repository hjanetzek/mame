-- license:BSD-3-Clause
-- copyright-holders:MAMEdev Team

---------------------------------------------------------------------------
--
--   wersi.lua
--
--   Wersi EX-20 minimal build target
--
---------------------------------------------------------------------------

CPUS["M6809"] = true
CPUS["Z8"] = true

VIDEOS["ROC10937"] = true

SOUNDS["DAC"] = true
SOUNDS["WERSI_SLM2_VOICE"] = true

MACHINES["ACIA6850"] = true
MACHINES["6840PTM"] = true

BUSES["MIDI"] = true


function createProjects_mame_wersi(_target, _subtarget)
	project ("mame_wersi")
	targetsubdir(_target .."_" .. _subtarget)
	kind (LIBTYPE)
	uuid (os.uuid("drv-mame_wersi"))
	addprojectflags()
	precompiledheaders_novs()

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/mame/shared",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "3rdparty",
		GEN_DIR  .. "mame/layout",
	}

files {
	MAME_DIR .. "src/mame/wersi/ex20.cpp",
	MAME_DIR .. "src/mame/wersi/slm2test.cpp",
}

end

function linkProjects_mame_wersi(_target, _subtarget)
	links {
		"mame_wersi",
	}
end
