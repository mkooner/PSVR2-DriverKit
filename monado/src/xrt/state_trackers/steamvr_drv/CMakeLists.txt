# Copyright 2020-2021, Collabora, Ltd.
# Copyright 2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_subdirectory(steamvr_bindings)

add_library(st_ovrd STATIC ovrd_driver.cpp ovrd_interface.h ovrd_room.cpp ovrd_room.hpp)

target_include_directories(st_ovrd INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
	st_ovrd PRIVATE xrt-interfaces xrt-external-openvr aux_math ovrd_generated_bindings
	)
# Driver direct mode waits on SteamVR's per-texture timeline semaphores with a small Vulkan device.
target_link_libraries(st_ovrd PRIVATE Vulkan::Vulkan)

# The room calibration measures the floor with a resting Sense controller's LED outline.
if(XRT_BUILD_DRIVER_PSSENSE)
	target_link_libraries(st_ovrd PRIVATE drv_pssense drv_includes tracking-includes constellation)
endif()
# The room calibration drops the room when the headset's SLAM re-originates.
if(XRT_BUILD_DRIVER_PSVR2)
	target_link_libraries(st_ovrd PRIVATE drv_psvr2 drv_includes)
endif()
