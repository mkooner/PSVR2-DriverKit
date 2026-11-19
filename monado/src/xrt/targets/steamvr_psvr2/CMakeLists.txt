# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0

# The native PS VR2 SteamVR driver: build/steamvr-monado-psvr2/bin/linux64/driver_monado_psvr2.so
set(OVRN_DIR "${PROJECT_BINARY_DIR}/steamvr-monado-psvr2")

add_library(driver_monado_psvr2 MODULE main.c)
# Monado's devices without a compositor: SteamVR owns the display.
target_link_libraries(
	driver_monado_psvr2
	PRIVATE
		st_ovrn
		xrt-external-openvr
		aux_util
		st_prober
		target_lists
		target_instance_no_comp
	)

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

install(DIRECTORY "${OVRN_DIR}" DESTINATION "${CMAKE_INSTALL_PREFIX}/share")
