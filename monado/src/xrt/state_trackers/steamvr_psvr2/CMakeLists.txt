# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0

add_library(
	st_ovrn STATIC
	ovrn_backend.hpp
	ovrn_backend_monado.cpp
	ovrn_backend_sim.cpp
	ovrn_bluetooth.cpp
	ovrn_bluetooth.hpp
	ovrn_camera.hpp
	ovrn_camera_fit.hpp
	ovrn_controller.hpp
	ovrn_device.hpp
	ovrn_direct_mode.hpp
	ovrn_frame_recorder.hpp
	ovrn_hand_skeleton.hpp
	ovrn_hmd.hpp
	ovrn_interface.h
	ovrn_log.hpp
	ovrn_steamvr_internal.hpp
	ovrn_provider.cpp
	ovrn_room.cpp
	ovrn_room.hpp
	ovrn_room_log.hpp
	ovrn_room_state.hpp
	ovrn_settings.hpp
	ovrn_steamvr_sync.hpp
	ovrn_vk_wait.hpp
	)
target_include_directories(st_ovrn INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
	st_ovrn PRIVATE xrt-external-openvr xrt-external-bcdec xrt-external-cjson xrt-interfaces aux_math aux_util
	)

# Driver direct mode waits on SteamVR's per-texture timeline semaphores with a small Vulkan device.
target_link_libraries(st_ovrn PRIVATE Vulkan::Vulkan)

# Disconnecting the Sense controllers through BlueZ when SteamVR closes (system D-Bus).
if(XRT_HAVE_DBUS)
	target_link_libraries(st_ovrn PRIVATE ${DBUS_LIBRARIES})
	target_include_directories(st_ovrn SYSTEM PRIVATE ${DBUS_INCLUDE_DIRS})
endif()

# The room calibration measures the floor with a resting Sense controller's LED outline.
if(XRT_BUILD_DRIVER_PSSENSE)
	target_link_libraries(st_ovrn PRIVATE drv_pssense drv_includes tracking-includes constellation)
endif()
# The room calibration follows the headset's SLAM; the passthrough fits SteamVR's lens model to the cameras' own.
if(XRT_BUILD_DRIVER_PSVR2)
	target_link_libraries(st_ovrn PRIVATE drv_psvr2 drv_includes aux_tracking)
endif()
