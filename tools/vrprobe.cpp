// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Can this process reach the running SteamVR? Connects as a background client (never starts SteamVR), prints the
// result and the headset it sees, and exits. Run it on the host and inside Steam's runtime containers to find out
// whether games in those containers can reach a SteamVR started on the host (2026-11-19).
//   ./vrprobe              (./vrprobe --props: also every device property, render size, adapter, projection)
//   ~/.local/share/Steam/steamapps/common/SteamLinuxRuntime_4/run -- ./vrprobe      (as Proton games run)
//   ~/.local/share/Steam/steamapps/common/SteamLinuxRuntime_sniper/run -- ./vrprobe (as native Steam games run)
// Build: g++ -O2 -o vrprobe vrprobe.cpp -I<openvr headers> -L<SteamVR>/bin/linux64 -lopenvr_api -Wl,-rpath,<SteamVR>/bin/linux64
#include "openvr.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

struct P { const char *name; vr::ETrackedDeviceProperty id; char type; };
static const P kProps[] = {
	{"Prop_TrackingSystemName_String", (vr::ETrackedDeviceProperty)1000, 'S'},
	{"Prop_ModelNumber_String", (vr::ETrackedDeviceProperty)1001, 'S'},
	{"Prop_SerialNumber_String", (vr::ETrackedDeviceProperty)1002, 'S'},
	{"Prop_RenderModelName_String", (vr::ETrackedDeviceProperty)1003, 'S'},
	{"Prop_WillDriftInYaw_Bool", (vr::ETrackedDeviceProperty)1004, 'B'},
	{"Prop_ManufacturerName_String", (vr::ETrackedDeviceProperty)1005, 'S'},
	{"Prop_TrackingFirmwareVersion_String", (vr::ETrackedDeviceProperty)1006, 'S'},
	{"Prop_HardwareRevision_String", (vr::ETrackedDeviceProperty)1007, 'S'},
	{"Prop_AllWirelessDongleDescriptions_String", (vr::ETrackedDeviceProperty)1008, 'S'},
	{"Prop_ConnectedWirelessDongle_String", (vr::ETrackedDeviceProperty)1009, 'S'},
	{"Prop_DeviceIsWireless_Bool", (vr::ETrackedDeviceProperty)1010, 'B'},
	{"Prop_DeviceIsCharging_Bool", (vr::ETrackedDeviceProperty)1011, 'B'},
	{"Prop_DeviceBatteryPercentage_Float", (vr::ETrackedDeviceProperty)1012, 'F'},
	{"Prop_StatusDisplayTransform_Matrix34", (vr::ETrackedDeviceProperty)1013, 'M'},
	{"Prop_Firmware_UpdateAvailable_Bool", (vr::ETrackedDeviceProperty)1014, 'B'},
	{"Prop_Firmware_ManualUpdate_Bool", (vr::ETrackedDeviceProperty)1015, 'B'},
	{"Prop_Firmware_ManualUpdateURL_String", (vr::ETrackedDeviceProperty)1016, 'S'},
	{"Prop_HardwareRevision_Uint64", (vr::ETrackedDeviceProperty)1017, 'U'},
	{"Prop_FirmwareVersion_Uint64", (vr::ETrackedDeviceProperty)1018, 'U'},
	{"Prop_FPGAVersion_Uint64", (vr::ETrackedDeviceProperty)1019, 'U'},
	{"Prop_VRCVersion_Uint64", (vr::ETrackedDeviceProperty)1020, 'U'},
	{"Prop_RadioVersion_Uint64", (vr::ETrackedDeviceProperty)1021, 'U'},
	{"Prop_DongleVersion_Uint64", (vr::ETrackedDeviceProperty)1022, 'U'},
	{"Prop_BlockServerShutdown_Bool", (vr::ETrackedDeviceProperty)1023, 'B'},
	{"Prop_CanUnifyCoordinateSystemWithHmd_Bool", (vr::ETrackedDeviceProperty)1024, 'B'},
	{"Prop_ContainsProximitySensor_Bool", (vr::ETrackedDeviceProperty)1025, 'B'},
	{"Prop_DeviceProvidesBatteryStatus_Bool", (vr::ETrackedDeviceProperty)1026, 'B'},
	{"Prop_DeviceCanPowerOff_Bool", (vr::ETrackedDeviceProperty)1027, 'B'},
	{"Prop_Firmware_ProgrammingTarget_String", (vr::ETrackedDeviceProperty)1028, 'S'},
	{"Prop_DeviceClass_Int32", (vr::ETrackedDeviceProperty)1029, 'I'},
	{"Prop_HasCamera_Bool", (vr::ETrackedDeviceProperty)1030, 'B'},
	{"Prop_DriverVersion_String", (vr::ETrackedDeviceProperty)1031, 'S'},
	{"Prop_Firmware_ForceUpdateRequired_Bool", (vr::ETrackedDeviceProperty)1032, 'B'},
	{"Prop_ViveSystemButtonFixRequired_Bool", (vr::ETrackedDeviceProperty)1033, 'B'},
	{"Prop_ParentDriver_Uint64", (vr::ETrackedDeviceProperty)1034, 'U'},
	{"Prop_ResourceRoot_String", (vr::ETrackedDeviceProperty)1035, 'S'},
	{"Prop_RegisteredDeviceType_String", (vr::ETrackedDeviceProperty)1036, 'S'},
	{"Prop_InputProfilePath_String", (vr::ETrackedDeviceProperty)1037, 'S'},
	{"Prop_NeverTracked_Bool", (vr::ETrackedDeviceProperty)1038, 'B'},
	{"Prop_NumCameras_Int32", (vr::ETrackedDeviceProperty)1039, 'I'},
	{"Prop_CameraFrameLayout_Int32", (vr::ETrackedDeviceProperty)1040, 'I'},
	{"Prop_CameraStreamFormat_Int32", (vr::ETrackedDeviceProperty)1041, 'I'},
	{"Prop_AdditionalDeviceSettingsPath_String", (vr::ETrackedDeviceProperty)1042, 'S'},
	{"Prop_Identifiable_Bool", (vr::ETrackedDeviceProperty)1043, 'B'},
	{"Prop_BootloaderVersion_Uint64", (vr::ETrackedDeviceProperty)1044, 'U'},
	{"Prop_AdditionalSystemReportData_String", (vr::ETrackedDeviceProperty)1045, 'S'},
	{"Prop_CompositeFirmwareVersion_String", (vr::ETrackedDeviceProperty)1046, 'S'},
	{"Prop_Firmware_RemindUpdate_Bool", (vr::ETrackedDeviceProperty)1047, 'B'},
	{"Prop_PeripheralApplicationVersion_Uint64", (vr::ETrackedDeviceProperty)1048, 'U'},
	{"Prop_ManufacturerSerialNumber_String", (vr::ETrackedDeviceProperty)1049, 'S'},
	{"Prop_ComputedSerialNumber_String", (vr::ETrackedDeviceProperty)1050, 'S'},
	{"Prop_EstimatedDeviceFirstUseTime_Int32", (vr::ETrackedDeviceProperty)1051, 'I'},
	{"Prop_DevicePowerUsage_Float", (vr::ETrackedDeviceProperty)1052, 'F'},
	{"Prop_IgnoreMotionForStandby_Bool", (vr::ETrackedDeviceProperty)1053, 'B'},
	{"Prop_ActualTrackingSystemName_String", (vr::ETrackedDeviceProperty)1054, 'S'},
	{"Prop_AllowCameraToggle_Bool", (vr::ETrackedDeviceProperty)1055, 'B'},
	{"Prop_AllowLightSourceFrequency_Bool", (vr::ETrackedDeviceProperty)1056, 'B'},
	{"Prop_SteamRemoteClientID_Uint64", (vr::ETrackedDeviceProperty)1057, 'U'},
	{"Prop_ReportsTimeSinceVSync_Bool", (vr::ETrackedDeviceProperty)2000, 'B'},
	{"Prop_SecondsFromVsyncToPhotons_Float", (vr::ETrackedDeviceProperty)2001, 'F'},
	{"Prop_DisplayFrequency_Float", (vr::ETrackedDeviceProperty)2002, 'F'},
	{"Prop_UserIpdMeters_Float", (vr::ETrackedDeviceProperty)2003, 'F'},
	{"Prop_CurrentUniverseId_Uint64", (vr::ETrackedDeviceProperty)2004, 'U'},
	{"Prop_DisplayFirmwareVersion_Uint64", (vr::ETrackedDeviceProperty)2006, 'U'},
	{"Prop_IsOnDesktop_Bool", (vr::ETrackedDeviceProperty)2007, 'B'},
	{"Prop_DisplayMCType_Int32", (vr::ETrackedDeviceProperty)2008, 'I'},
	{"Prop_DisplayMCOffset_Float", (vr::ETrackedDeviceProperty)2009, 'F'},
	{"Prop_DisplayMCScale_Float", (vr::ETrackedDeviceProperty)2010, 'F'},
	{"Prop_EdidVendorID_Int32", (vr::ETrackedDeviceProperty)2011, 'I'},
	{"Prop_DisplayMCImageLeft_String", (vr::ETrackedDeviceProperty)2012, 'S'},
	{"Prop_DisplayMCImageRight_String", (vr::ETrackedDeviceProperty)2013, 'S'},
	{"Prop_DisplayGCBlackClamp_Float", (vr::ETrackedDeviceProperty)2014, 'F'},
	{"Prop_EdidProductID_Int32", (vr::ETrackedDeviceProperty)2015, 'I'},
	{"Prop_CameraToHeadTransform_Matrix34", (vr::ETrackedDeviceProperty)2016, 'M'},
	{"Prop_DisplayGCType_Int32", (vr::ETrackedDeviceProperty)2017, 'I'},
	{"Prop_DisplayGCOffset_Float", (vr::ETrackedDeviceProperty)2018, 'F'},
	{"Prop_DisplayGCScale_Float", (vr::ETrackedDeviceProperty)2019, 'F'},
	{"Prop_DisplayGCPrescale_Float", (vr::ETrackedDeviceProperty)2020, 'F'},
	{"Prop_DisplayGCImage_String", (vr::ETrackedDeviceProperty)2021, 'S'},
	{"Prop_LensCenterLeftU_Float", (vr::ETrackedDeviceProperty)2022, 'F'},
	{"Prop_LensCenterLeftV_Float", (vr::ETrackedDeviceProperty)2023, 'F'},
	{"Prop_LensCenterRightU_Float", (vr::ETrackedDeviceProperty)2024, 'F'},
	{"Prop_LensCenterRightV_Float", (vr::ETrackedDeviceProperty)2025, 'F'},
	{"Prop_UserHeadToEyeDepthMeters_Float", (vr::ETrackedDeviceProperty)2026, 'F'},
	{"Prop_CameraFirmwareVersion_Uint64", (vr::ETrackedDeviceProperty)2027, 'U'},
	{"Prop_CameraFirmwareDescription_String", (vr::ETrackedDeviceProperty)2028, 'S'},
	{"Prop_DisplayFPGAVersion_Uint64", (vr::ETrackedDeviceProperty)2029, 'U'},
	{"Prop_DisplayBootloaderVersion_Uint64", (vr::ETrackedDeviceProperty)2030, 'U'},
	{"Prop_DisplayHardwareVersion_Uint64", (vr::ETrackedDeviceProperty)2031, 'U'},
	{"Prop_AudioFirmwareVersion_Uint64", (vr::ETrackedDeviceProperty)2032, 'U'},
	{"Prop_CameraCompatibilityMode_Int32", (vr::ETrackedDeviceProperty)2033, 'I'},
	{"Prop_ScreenshotHorizontalFieldOfViewDegrees_Float", (vr::ETrackedDeviceProperty)2034, 'F'},
	{"Prop_ScreenshotVerticalFieldOfViewDegrees_Float", (vr::ETrackedDeviceProperty)2035, 'F'},
	{"Prop_DisplaySuppressed_Bool", (vr::ETrackedDeviceProperty)2036, 'B'},
	{"Prop_DisplayAllowNightMode_Bool", (vr::ETrackedDeviceProperty)2037, 'B'},
	{"Prop_DisplayMCImageWidth_Int32", (vr::ETrackedDeviceProperty)2038, 'I'},
	{"Prop_DisplayMCImageHeight_Int32", (vr::ETrackedDeviceProperty)2039, 'I'},
	{"Prop_DisplayMCImageNumChannels_Int32", (vr::ETrackedDeviceProperty)2040, 'I'},
	{"Prop_SecondsFromPhotonsToVblank_Float", (vr::ETrackedDeviceProperty)2042, 'F'},
	{"Prop_DriverDirectModeSendsVsyncEvents_Bool", (vr::ETrackedDeviceProperty)2043, 'B'},
	{"Prop_DisplayDebugMode_Bool", (vr::ETrackedDeviceProperty)2044, 'B'},
	{"Prop_GraphicsAdapterLuid_Uint64", (vr::ETrackedDeviceProperty)2045, 'U'},
	{"Prop_DriverProvidedChaperonePath_String", (vr::ETrackedDeviceProperty)2048, 'S'},
	{"Prop_ExpectedTrackingReferenceCount_Int32", (vr::ETrackedDeviceProperty)2049, 'I'},
	{"Prop_ExpectedControllerCount_Int32", (vr::ETrackedDeviceProperty)2050, 'I'},
	{"Prop_NamedIconPathControllerLeftDeviceOff_String", (vr::ETrackedDeviceProperty)2051, 'S'},
	{"Prop_NamedIconPathControllerRightDeviceOff_String", (vr::ETrackedDeviceProperty)2052, 'S'},
	{"Prop_NamedIconPathTrackingReferenceDeviceOff_String", (vr::ETrackedDeviceProperty)2053, 'S'},
	{"Prop_DoNotApplyPrediction_Bool", (vr::ETrackedDeviceProperty)2054, 'B'},
	{"Prop_DistortionMeshResolution_Int32", (vr::ETrackedDeviceProperty)2056, 'I'},
	{"Prop_DriverIsDrawingControllers_Bool", (vr::ETrackedDeviceProperty)2057, 'B'},
	{"Prop_DriverRequestsApplicationPause_Bool", (vr::ETrackedDeviceProperty)2058, 'B'},
	{"Prop_DriverRequestsReducedRendering_Bool", (vr::ETrackedDeviceProperty)2059, 'B'},
	{"Prop_MinimumIpdStepMeters_Float", (vr::ETrackedDeviceProperty)2060, 'F'},
	{"Prop_AudioBridgeFirmwareVersion_Uint64", (vr::ETrackedDeviceProperty)2061, 'U'},
	{"Prop_ImageBridgeFirmwareVersion_Uint64", (vr::ETrackedDeviceProperty)2062, 'U'},
	{"Prop_ImuToHeadTransform_Matrix34", (vr::ETrackedDeviceProperty)2063, 'M'},
	{"Prop_ImuFactoryGyroBias_Vector3", (vr::ETrackedDeviceProperty)2064, 'V'},
	{"Prop_ImuFactoryGyroScale_Vector3", (vr::ETrackedDeviceProperty)2065, 'V'},
	{"Prop_ImuFactoryAccelerometerBias_Vector3", (vr::ETrackedDeviceProperty)2066, 'V'},
	{"Prop_ImuFactoryAccelerometerScale_Vector3", (vr::ETrackedDeviceProperty)2067, 'V'},
	{"Prop_ConfigurationIncludesLighthouse20Features_Bool", (vr::ETrackedDeviceProperty)2069, 'B'},
	{"Prop_AdditionalRadioFeatures_Uint64", (vr::ETrackedDeviceProperty)2070, 'U'},
	{"Prop_ExpectedControllerType_String", (vr::ETrackedDeviceProperty)2074, 'S'},
	{"Prop_HmdTrackingStyle_Int32", (vr::ETrackedDeviceProperty)2075, 'I'},
	{"Prop_DriverProvidedChaperoneVisibility_Bool", (vr::ETrackedDeviceProperty)2076, 'B'},
	{"Prop_HmdColumnCorrectionSettingPrefix_String", (vr::ETrackedDeviceProperty)2077, 'S'},
	{"Prop_CameraSupportsCompatibilityModes_Bool", (vr::ETrackedDeviceProperty)2078, 'B'},
	{"Prop_SupportsRoomViewDepthProjection_Bool", (vr::ETrackedDeviceProperty)2079, 'B'},
	{"Prop_DisplaySupportsMultipleFramerates_Bool", (vr::ETrackedDeviceProperty)2081, 'B'},
	{"Prop_DisplayColorMultLeft_Vector3", (vr::ETrackedDeviceProperty)2082, 'V'},
	{"Prop_DisplayColorMultRight_Vector3", (vr::ETrackedDeviceProperty)2083, 'V'},
	{"Prop_DisplaySupportsRuntimeFramerateChange_Bool", (vr::ETrackedDeviceProperty)2084, 'B'},
	{"Prop_DisplaySupportsAnalogGain_Bool", (vr::ETrackedDeviceProperty)2085, 'B'},
	{"Prop_DisplayMinAnalogGain_Float", (vr::ETrackedDeviceProperty)2086, 'F'},
	{"Prop_DisplayMaxAnalogGain_Float", (vr::ETrackedDeviceProperty)2087, 'F'},
	{"Prop_CameraExposureTime_Float", (vr::ETrackedDeviceProperty)2088, 'F'},
	{"Prop_CameraGlobalGain_Float", (vr::ETrackedDeviceProperty)2089, 'F'},
	{"Prop_DashboardLayoutPathName_String", (vr::ETrackedDeviceProperty)2090, 'S'},
	{"Prop_DashboardScale_Float", (vr::ETrackedDeviceProperty)2091, 'F'},
	{"Prop_PeerButtonInfo_String", (vr::ETrackedDeviceProperty)2092, 'S'},
	{"Prop_Hmd_SupportsHDR10_Bool", (vr::ETrackedDeviceProperty)2093, 'B'},
	{"Prop_Hmd_EnableParallelRenderCameras_Bool", (vr::ETrackedDeviceProperty)2094, 'B'},
	{"Prop_DriverProvidedChaperoneJson_String", (vr::ETrackedDeviceProperty)2095, 'S'},
	{"Prop_ForceSystemLayerUseAppPoses_Bool", (vr::ETrackedDeviceProperty)2096, 'B'},
	{"Prop_DashboardLinkSupport_Int32", (vr::ETrackedDeviceProperty)2097, 'I'},
	{"Prop_DisplayMinUIAnalogGain_Float", (vr::ETrackedDeviceProperty)2098, 'F'},
	{"Prop_IpdUIRangeMinMeters_Float", (vr::ETrackedDeviceProperty)2100, 'F'},
	{"Prop_IpdUIRangeMaxMeters_Float", (vr::ETrackedDeviceProperty)2101, 'F'},
	{"Prop_Hmd_SupportsHDCP14LegacyCompat_Bool", (vr::ETrackedDeviceProperty)2102, 'B'},
	{"Prop_Hmd_SupportsMicMonitoring_Bool", (vr::ETrackedDeviceProperty)2103, 'B'},
	{"Prop_Hmd_SupportsDisplayPortTrainingMode_Bool", (vr::ETrackedDeviceProperty)2104, 'B'},
	{"Prop_Hmd_SupportsRoomViewDirect_Bool", (vr::ETrackedDeviceProperty)2105, 'B'},
	{"Prop_Hmd_SupportsAppThrottling_Bool", (vr::ETrackedDeviceProperty)2106, 'B'},
	{"Prop_Hmd_SupportsGpuBusMonitoring_Bool", (vr::ETrackedDeviceProperty)2107, 'B'},
	{"Prop_DriverDisplaysIPDChanges_Bool", (vr::ETrackedDeviceProperty)2108, 'B'},
	{"Prop_Driver_RecenterSupport_Int32", (vr::ETrackedDeviceProperty)2109, 'I'},
	{"Prop_Hmd_MaxDistortedTextureWidth_Int32", (vr::ETrackedDeviceProperty)2113, 'I'},
	{"Prop_Hmd_MaxDistortedTextureHeight_Int32", (vr::ETrackedDeviceProperty)2114, 'I'},
	{"Prop_Hmd_AllowSupersampleFiltering_Bool", (vr::ETrackedDeviceProperty)2115, 'B'},
	{"Prop_DriverRequestedMuraCorrectionMode_Int32", (vr::ETrackedDeviceProperty)2200, 'I'},
	{"Prop_DriverRequestedMuraFeather_InnerLeft_Int32", (vr::ETrackedDeviceProperty)2201, 'I'},
	{"Prop_DriverRequestedMuraFeather_InnerRight_Int32", (vr::ETrackedDeviceProperty)2202, 'I'},
	{"Prop_DriverRequestedMuraFeather_InnerTop_Int32", (vr::ETrackedDeviceProperty)2203, 'I'},
	{"Prop_DriverRequestedMuraFeather_InnerBottom_Int32", (vr::ETrackedDeviceProperty)2204, 'I'},
	{"Prop_DriverRequestedMuraFeather_OuterLeft_Int32", (vr::ETrackedDeviceProperty)2205, 'I'},
	{"Prop_DriverRequestedMuraFeather_OuterRight_Int32", (vr::ETrackedDeviceProperty)2206, 'I'},
	{"Prop_DriverRequestedMuraFeather_OuterTop_Int32", (vr::ETrackedDeviceProperty)2207, 'I'},
	{"Prop_DriverRequestedMuraFeather_OuterBottom_Int32", (vr::ETrackedDeviceProperty)2208, 'I'},
	{"Prop_Audio_DefaultPlaybackDeviceId_String", (vr::ETrackedDeviceProperty)2300, 'S'},
	{"Prop_Audio_DefaultRecordingDeviceId_String", (vr::ETrackedDeviceProperty)2301, 'S'},
	{"Prop_Audio_DefaultPlaybackDeviceVolume_Float", (vr::ETrackedDeviceProperty)2302, 'F'},
	{"Prop_Audio_SupportsDualSpeakerAndJackOutput_Bool", (vr::ETrackedDeviceProperty)2303, 'B'},
	{"Prop_Audio_DriverManagesPlaybackVolumeControl_Bool", (vr::ETrackedDeviceProperty)2304, 'B'},
	{"Prop_Audio_DriverPlaybackVolume_Float", (vr::ETrackedDeviceProperty)2305, 'F'},
	{"Prop_Audio_DriverPlaybackMute_Bool", (vr::ETrackedDeviceProperty)2306, 'B'},
	{"Prop_Audio_DriverManagesRecordingVolumeControl_Bool", (vr::ETrackedDeviceProperty)2307, 'B'},
	{"Prop_Audio_DriverRecordingVolume_Float", (vr::ETrackedDeviceProperty)2308, 'F'},
	{"Prop_Audio_DriverRecordingMute_Bool", (vr::ETrackedDeviceProperty)2309, 'B'},
	{"Prop_Audio_PipewirePlaybackNode_Int32", (vr::ETrackedDeviceProperty)2400, 'I'},
	{"Prop_Audio_PipewireRecordingNode_Int32", (vr::ETrackedDeviceProperty)2401, 'I'},
	{"Prop_AttachedDeviceId_String", (vr::ETrackedDeviceProperty)3000, 'S'},
	{"Prop_SupportedButtons_Uint64", (vr::ETrackedDeviceProperty)3001, 'U'},
	{"Prop_Axis0Type_Int32", (vr::ETrackedDeviceProperty)3002, 'I'},
	{"Prop_Axis1Type_Int32", (vr::ETrackedDeviceProperty)3003, 'I'},
	{"Prop_Axis2Type_Int32", (vr::ETrackedDeviceProperty)3004, 'I'},
	{"Prop_Axis3Type_Int32", (vr::ETrackedDeviceProperty)3005, 'I'},
	{"Prop_Axis4Type_Int32", (vr::ETrackedDeviceProperty)3006, 'I'},
	{"Prop_ControllerRoleHint_Int32", (vr::ETrackedDeviceProperty)3007, 'I'},
	{"Prop_FieldOfViewLeftDegrees_Float", (vr::ETrackedDeviceProperty)4000, 'F'},
	{"Prop_FieldOfViewRightDegrees_Float", (vr::ETrackedDeviceProperty)4001, 'F'},
	{"Prop_FieldOfViewTopDegrees_Float", (vr::ETrackedDeviceProperty)4002, 'F'},
	{"Prop_FieldOfViewBottomDegrees_Float", (vr::ETrackedDeviceProperty)4003, 'F'},
	{"Prop_TrackingRangeMinimumMeters_Float", (vr::ETrackedDeviceProperty)4004, 'F'},
	{"Prop_TrackingRangeMaximumMeters_Float", (vr::ETrackedDeviceProperty)4005, 'F'},
	{"Prop_ModeLabel_String", (vr::ETrackedDeviceProperty)4006, 'S'},
	{"Prop_CanWirelessIdentify_Bool", (vr::ETrackedDeviceProperty)4007, 'B'},
	{"Prop_Nonce_Int32", (vr::ETrackedDeviceProperty)4008, 'I'},
	{"Prop_IconPathName_String", (vr::ETrackedDeviceProperty)5000, 'S'},
	{"Prop_NamedIconPathDeviceOff_String", (vr::ETrackedDeviceProperty)5001, 'S'},
	{"Prop_NamedIconPathDeviceSearching_String", (vr::ETrackedDeviceProperty)5002, 'S'},
	{"Prop_NamedIconPathDeviceSearchingAlert_String", (vr::ETrackedDeviceProperty)5003, 'S'},
	{"Prop_NamedIconPathDeviceReady_String", (vr::ETrackedDeviceProperty)5004, 'S'},
	{"Prop_NamedIconPathDeviceReadyAlert_String", (vr::ETrackedDeviceProperty)5005, 'S'},
	{"Prop_NamedIconPathDeviceNotReady_String", (vr::ETrackedDeviceProperty)5006, 'S'},
	{"Prop_NamedIconPathDeviceStandby_String", (vr::ETrackedDeviceProperty)5007, 'S'},
	{"Prop_NamedIconPathDeviceAlertLow_String", (vr::ETrackedDeviceProperty)5008, 'S'},
	{"Prop_NamedIconPathDeviceStandbyAlert_String", (vr::ETrackedDeviceProperty)5009, 'S'},
	{"Prop_OverrideContainer_Uint64", (vr::ETrackedDeviceProperty)5152, 'U'},
	{"Prop_UserConfigPath_String", (vr::ETrackedDeviceProperty)6000, 'S'},
	{"Prop_InstallPath_String", (vr::ETrackedDeviceProperty)6001, 'S'},
	{"Prop_HasDisplayComponent_Bool", (vr::ETrackedDeviceProperty)6002, 'B'},
	{"Prop_HasControllerComponent_Bool", (vr::ETrackedDeviceProperty)6003, 'B'},
	{"Prop_HasCameraComponent_Bool", (vr::ETrackedDeviceProperty)6004, 'B'},
	{"Prop_HasDriverDirectModeComponent_Bool", (vr::ETrackedDeviceProperty)6005, 'B'},
	{"Prop_HasVirtualDisplayComponent_Bool", (vr::ETrackedDeviceProperty)6006, 'B'},
	{"Prop_HasSpatialAnchorsSupport_Bool", (vr::ETrackedDeviceProperty)6007, 'B'},
	{"Prop_SupportsXrTextureSets_Bool", (vr::ETrackedDeviceProperty)6008, 'B'},
	{"Prop_SupportsXrEyeGazeInteraction_Bool", (vr::ETrackedDeviceProperty)6009, 'B'},
	{"Prop_DeviceHasNoIMU_Bool", (vr::ETrackedDeviceProperty)6010, 'B'},
	{"Prop_UseAdvancedPrediction_Bool", (vr::ETrackedDeviceProperty)6011, 'B'},
	{"Prop_ControllerType_String", (vr::ETrackedDeviceProperty)7000, 'S'},
	{"Prop_LegacyInputProfile_String", (vr::ETrackedDeviceProperty)7001, 'S'},
	{"Prop_ControllerHandSelectionPriority_Int32", (vr::ETrackedDeviceProperty)7002, 'I'},
};

int
main(int argc, char **argv)
{
	const char *inside = getenv("PRESSURE_VESSEL_RUNTIME") ? "inside a Steam runtime container" : "on the host (no container)";
	printf("vrprobe: running %s\n", inside);
	vr::EVRInitError e = vr::VRInitError_None;
	vr::IVRSystem *sys = vr::VR_Init(&e, vr::VRApplication_Background);
	if (e != vr::VRInitError_None || sys == nullptr) {
		printf("vrprobe: CAN'T reach SteamVR: %s (%d)\n", vr::VR_GetVRInitErrorAsEnglishDescription(e), (int)e);
		return 1;
	}
	char name[128] = "", model[128] = "";
	sys->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String, name, sizeof(name));
	sys->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_ModelNumber_String, model, sizeof(model));
	printf("vrprobe: reached SteamVR: headset '%s' from driver '%s'\n", model, name);
	if (argc > 1 && strcmp(argv[1], "--props") == 0) {
		for (vr::TrackedDeviceIndex_t d = 0; d < vr::k_unMaxTrackedDeviceCount; d++) {
			vr::ETrackedDeviceClass c = sys->GetTrackedDeviceClass(d);
			if (c == vr::TrackedDeviceClass_Invalid) continue;
			for (const P &p : kProps) {
				vr::ETrackedPropertyError err = vr::TrackedProp_Success;
				char buf[1024] = "";
				switch (p.type) {
				case 'S': sys->GetStringTrackedDeviceProperty(d, p.id, buf, sizeof(buf), &err); break;
				case 'B': { bool b = sys->GetBoolTrackedDeviceProperty(d, p.id, &err); snprintf(buf, sizeof(buf), "%s", b ? "true" : "false"); } break;
				case 'F': { float f = sys->GetFloatTrackedDeviceProperty(d, p.id, &err); snprintf(buf, sizeof(buf), "%.6g", f); } break;
				case 'I': { int32_t i = sys->GetInt32TrackedDeviceProperty(d, p.id, &err); snprintf(buf, sizeof(buf), "%d", i); } break;
				case 'U': { uint64_t u = sys->GetUint64TrackedDeviceProperty(d, p.id, &err); snprintf(buf, sizeof(buf), "0x%llx", (unsigned long long)u); } break;
				case 'M': { vr::HmdMatrix34_t m = sys->GetMatrix34TrackedDeviceProperty(d, p.id, &err); snprintf(buf, sizeof(buf), "[%.4g %.4g %.4g %.4g | %.4g %.4g %.4g %.4g | %.4g %.4g %.4g %.4g]", m.m[0][0], m.m[0][1], m.m[0][2], m.m[0][3], m.m[1][0], m.m[1][1], m.m[1][2], m.m[1][3], m.m[2][0], m.m[2][1], m.m[2][2], m.m[2][3]); } break;
				default: continue;
				}
				if (err == vr::TrackedProp_Success) printf("dev%u class%d %s = %s\n", d, (int)c, p.name, buf);
			}
		}
		uint32_t w = 0, h = 0;
		sys->GetRecommendedRenderTargetSize(&w, &h);
		int32_t adapter = -1;
		sys->GetDXGIOutputInfo(&adapter);
		uint64_t dev = 0;
		sys->GetOutputDevice(&dev, vr::TextureType_Vulkan, nullptr);
		printf("system RecommendedRenderTargetSize = %ux%u\nsystem DXGIOutputInfo adapter = %d\nsystem OutputDevice(Vulkan) = 0x%llx\n", w, h, adapter, (unsigned long long)dev);
		for (int e = 0; e < 2; e++) {
			float l, r, t, b;
			sys->GetProjectionRaw((vr::EVREye)e, &l, &r, &t, &b);
			vr::HmdMatrix34_t m = sys->GetEyeToHeadTransform((vr::EVREye)e);
			printf("system eye%d ProjectionRaw = %.5f %.5f %.5f %.5f, EyeToHead x %.5f\n", e, l, r, t, b, m.m[0][3]);
		}
	}
	vr::VR_Shutdown();
	return 0;
}
