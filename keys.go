package goovr

const OVR_KEY_USER = "User" // string

const OVR_KEY_NAME = "Name" // string

const OVR_KEY_GENDER = "Gender" // string "Male", "Female", or "Unknown"
const OVR_DEFAULT_GENDER = "Unknown"

const OVR_KEY_PLAYER_HEIGHT = "PlayerHeight" // float meters
const OVR_DEFAULT_PLAYER_HEIGHT = 1.778

const OVR_KEY_EYE_HEIGHT = "EyeHeight" // float meters
const OVR_DEFAULT_EYE_HEIGHT = 1.675

const OVR_KEY_NECK_TO_EYE_DISTANCE = "NeckEyeDistance" // float[2] meters
const OVR_DEFAULT_NECK_TO_EYE_HORIZONTAL = 0.0805
const OVR_DEFAULT_NECK_TO_EYE_VERTICAL = 0.075

const OVR_KEY_EYE_TO_NOSE_DISTANCE = "EyeToNoseDist" // float[2] meters

const OVR_PERF_HUD_MODE = "PerfHudMode" // int, allowed values are defined in enum ovrPerfHudMode

const OVR_LAYER_HUD_MODE = "LayerHudMode"                  // int, allowed values are defined in enum ovrLayerHudMode
const OVR_LAYER_HUD_CURRENT_LAYER = "LayerHudCurrentLayer" // int, The layer to show
const OVR_LAYER_HUD_SHOW_ALL_LAYERS = "LayerHudShowAll"    // bool, Hide other layers when the hud is enabled

const OVR_DEBUG_HUD_STEREO_MODE = "DebugHudStereoMode"                              // int, allowed values are defined in enum ovrDebugHudStereoMode
const OVR_DEBUG_HUD_STEREO_GUIDE_INFO_ENABLE = "DebugHudStereoGuideInfoEnable"      // bool
const OVR_DEBUG_HUD_STEREO_GUIDE_SIZE = "DebugHudStereoGuideSize2f"                 // float[2]
const OVR_DEBUG_HUD_STEREO_GUIDE_POSITION = "DebugHudStereoGuidePosition3f"         // float[3]
const OVR_DEBUG_HUD_STEREO_GUIDE_YAWPITCHROLL = "DebugHudStereoGuideYawPitchRoll3f" // float[3]
const OVR_DEBUG_HUD_STEREO_GUIDE_COLOR = "DebugHudStereoGuideColor4f"               // float[4]
