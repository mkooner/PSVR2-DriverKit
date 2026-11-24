# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0

# The native PS VR2 SteamVR driver: build/steamvr-monado-psvr2/bin/linux64/driver_monado_psvr2.so
set(OVRN_DIR "${PROJECT_BINARY_DIR}/steamvr-monado-psvr2")

add_library(driver_monado_psvr2 MODULE main.c)
target_link_libraries(
	driver_monado_psvr2
	PRIVATE
		st_ovrn
		xrt-external-openvr
		aux_util
		st_prober
		target_lists
	)
# With Monado's compositor when it is built: driver direct mode (DriverKit's setting, off by default) runs it inside
# vrserver to show SteamVR's frames. With direct mode off no compositor is created (devices only, SteamVR owns the
# display), the same as with target_instance_no_comp.
if(TARGET target_instance)
	target_link_libraries(driver_monado_psvr2 PRIVATE target_instance)
else()
	target_link_libraries(driver_monado_psvr2 PRIVATE target_instance_no_comp)
endif()

file(COPY driver.vrdrivermanifest DESTINATION ${OVRN_DIR})

# The resources (input profiles, icons, localization), first written for the previous plugin, with its driver name in the
# resource paths ({monado}) replaced by this driver's ({monado_psvr2}).
file(COPY resources DESTINATION ${OVRN_DIR})
# Copied at configure time: re-run the configure step when a resource file changes, so an edited binding or profile
# reaches the built plugin (it didn't: the legacy binding's poses, 2026-11-18).
file(GLOB_RECURSE OVRN_RESOURCE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/resources/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${OVRN_RESOURCE_SOURCES})
file(GLOB_RECURSE OVRN_RESOURCE_TEXT "${OVRN_DIR}/resources/*.json" "${OVRN_DIR}/resources/*.vrresources"
     "${OVRN_DIR}/resources/*.vrsettings"
	)
foreach(OVRN_FILE ${OVRN_RESOURCE_TEXT})
	file(READ "${OVRN_FILE}" OVRN_TEXT)
	string(REPLACE "{monado}" "{monado_psvr2}" OVRN_TEXT "${OVRN_TEXT}")
	string(REPLACE "driver_monado\"" "driver_monado_psvr2\"" OVRN_TEXT "${OVRN_TEXT}")
	string(REPLACE "\"resource_root\": \"monado\"" "\"resource_root\": \"monado_psvr2\"" OVRN_TEXT "${OVRN_TEXT}")
	string(REPLACE "\"driver_name\": \"monado\"" "\"driver_name\": \"monado_psvr2\"" OVRN_TEXT "${OVRN_TEXT}")
	file(WRITE "${OVRN_FILE}" "${OVRN_TEXT}")
endforeach()

set_target_properties(
	driver_monado_psvr2
	PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${OVRN_DIR}/bin/linux64"
		   PREFIX ""
		   BUILD_RPATH "$ORIGIN"
		   INSTALL_RPATH "$ORIGIN"
	)

# DT_RPATH (not RUNPATH): the loader also applies it to the bundled libraries' own dependencies (Ceres -> SuiteSparse),
# so a set bundled next to the plugin resolves as a whole inside Steam's runtime container.
target_link_options(driver_monado_psvr2 PRIVATE "-Wl,--disable-new-dtags")

# OVRN_BUNDLE_LIBS: copy the libraries Steam's runtime doesn't provide next to the plugin after each build
# (bundle_libs.py), so SteamVR can load it inside its Steam Linux Runtime container (sniper) and Steam games, which run
# inside it too, can connect (2026-11-19). OVRN_STEAM_RUNTIME: the runtime's files dir ("auto": the newest sniper
# platform under ~/.local/share/Steam). Meant for a minimal build (only the PS VR2 drivers, no OpenCV/GStreamer/onnx).
option(OVRN_BUNDLE_LIBS "Bundle the SteamVR plugin's non-runtime libraries next to it" OFF)
set(OVRN_STEAM_RUNTIME "auto" CACHE STRING "Steam runtime files dir for OVRN_BUNDLE_LIBS (auto: newest sniper)")
if(OVRN_BUNDLE_LIBS)
	find_package(Python3 REQUIRED COMPONENTS Interpreter)
	add_custom_command(
		TARGET driver_monado_psvr2
		POST_BUILD
		COMMAND ${Python3_EXECUTABLE} -I "${CMAKE_CURRENT_SOURCE_DIR}/bundle_libs.py"
			$<TARGET_FILE:driver_monado_psvr2> "${OVRN_STEAM_RUNTIME}"
		COMMENT "Bundling the SteamVR plugin's libraries for Steam's runtime"
		VERBATIM
		)
endif()

install(DIRECTORY "${OVRN_DIR}" DESTINATION "${CMAKE_INSTALL_PREFIX}/share")
