package goovr

const OVR_KEY_USER = "User"                                   // string
const OVR_KEY_NAME = "Name"                                   // string
const OVR_KEY_GENDER = "Gender"                               // string "Male", "Female", or "Unknown"
const OVR_KEY_PLAYER_HEIGHT = "PlayerHeight"                  // float meters
const OVR_KEY_EYE_HEIGHT = "EyeHeight"                        // float meters
const OVR_KEY_IPD = "IPD"                                     // float meters
const OVR_KEY_NECK_TO_EYE_DISTANCE = "NeckEyeDistance"        // float[2] meters
const OVR_KEY_EYE_RELIEF_DIAL = "EyeReliefDial"               // int in range of 0-10
const OVR_KEY_EYE_TO_NOSE_DISTANCE = "EyeToNoseDist"          // float[2] meters
const OVR_KEY_MAX_EYE_TO_PLATE_DISTANCE = "MaxEyeToPlateDist" // float[2] meters
const OVR_KEY_EYE_CUP = "EyeCup"                              // char[16] "A", "B", or "C"
const OVR_KEY_CUSTOM_EYE_RENDER = "CustomEyeRender"           // bool
//Legacy profile value tied to the device and serial
const OVR_KEY_CAMERA_POSITION_1 = "CenteredFromWorld" // double[7] ovrPosef quat rotation x, y, z, w, translation x, y, z
//New value that now only ties to the device so that swapping headsets retains the offset from the tracker
const OVR_KEY_CAMERA_POSITION_2 = "CenteredFromWorld2" // double[7] ovrPosef quat rotation x, y, z, w, translation x, y, z
const OVR_KEY_CAMERA_POSITION = OVR_KEY_CAMERA_POSITION_2

// Default measurements empirically determined at Oculus to make us happy
// The neck model numbers were derived as an average of the male and female averages from ANSUR-88
// NECK_TO_EYE_HORIZONTAL = H22 - H43 = INFRAORBITALE_BACK_OF_HEAD - TRAGION_BACK_OF_HEAD
// NECK_TO_EYE_VERTICAL = H21 - H15 = GONION_TOP_OF_HEAD - ECTOORBITALE_TOP_OF_HEAD
// These were determined to be the best in a small user study, clearly beating out the previous default values
const OVR_DEFAULT_GENDER = "Unknown"
const OVR_DEFAULT_PLAYER_HEIGHT = 1.778
const OVR_DEFAULT_EYE_HEIGHT = 1.675
const OVR_DEFAULT_IPD = 0.064
const OVR_DEFAULT_NECK_TO_EYE_HORIZONTAL = 0.0805
const OVR_DEFAULT_NECK_TO_EYE_VERTICAL = 0.075
const OVR_DEFAULT_EYE_RELIEF_DIAL = 3

var OVR_DEFAULT_CAMERA_POSITION = []int{0, 0, 0, 1, 0, 0, 0}

const OVR_PERF_HUD_MODE = "PerfHudMode" // allowed values are defined in enum ovrPerfHudMode

const OVR_LAYER_HUD_MODE = "LayerHudMode"                  // allowed values are defined in enum ovrLayerHudMode
const OVR_LAYER_HUD_CURRENT_LAYER = "LayerHudCurrentLayer" // The layer to show
const OVR_LAYER_HUD_SHOW_ALL_LAYERS = "LayerHudShowAll"    // Hide other layers when the hud is enabled

const OVR_DEBUG_HUD_STEREO_MODE = "DebugHudStereoMode"                              // allowed values are defined in enum ovrDebugHudStereoMode
const OVR_DEBUG_HUD_STEREO_GUIDE_INFO_ENABLE = "DebugHudStereoGuideInfoEnable"      // bool
const OVR_DEBUG_HUD_STEREO_GUIDE_SIZE = "DebugHudStereoGuideSize2f"                 // float[2]
const OVR_DEBUG_HUD_STEREO_GUIDE_POSITION = "DebugHudStereoGuidePosition3f"         // float[3]
const OVR_DEBUG_HUD_STEREO_GUIDE_YAWPITCHROLL = "DebugHudStereoGuideYawPitchRoll3f" // float[3]
const OVR_DEBUG_HUD_STEREO_GUIDE_COLOR = "DebugHudStereoGuideColor4f"               // float[4]
