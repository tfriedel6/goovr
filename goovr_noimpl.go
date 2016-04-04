// +build !windows

package goovr

/*
#cgo CXXFLAGS: -std=c++11
*/
import "C"
import (
	"errors"
)

var errNoImpl = errors.New("OVR not implemented on this platform")

/// A 2D vector with integer components.
type Vector2i struct {
	X, Y int
}

/// A 2D size with integer components.
type Sizei struct {
	W, H int
}

/// A 2D rectangle with a position and size.
/// All components are integers.
type Recti struct {
	Pos  Vector2i
	Size Sizei
}

/// A quaternion rotation.
type Quatf struct {
	X, Y, Z, W float32
}

/// A 2D vector with float components.
type Vector2f struct {
	X, Y float32
}

/// A 3D vector with float components.
type Vector3f struct {
	X, Y, Z float32
}

/// A 4x4 matrix with float elements.
type Matrix4f struct {
	M [4][4]float32
}

/// Position and orientation together.
type Posef struct {
	Orientation Quatf
	Position    Vector3f
}

/// A full pose (rigid body) configuration with first and second derivatives.
///
/// Body refers to any object for which ovrPoseStatef is providing data.
/// It can be the HMD, Touch controller, sensor or something else. The context
/// depends on the usage of the struct.
type PoseStatef struct {
	ThePose             Posef    //< Position and orientation.
	AngularVelocity     Vector3f //< Angular velocity in radians per second.
	LinearVelocity      Vector3f //< Velocity in meters per second.
	AngularAcceleration Vector3f //< Angular acceleration in radians per second per second.
	LinearAcceleration  Vector3f //< Acceleration in meters per second per second.
	TimeInSeconds       float64  //< Absolute time of this state sample.
}

/// Describes the up, down, left, and right angles of the field of view.
///
/// Field Of View (FOV) tangent of the angle units.
/// \note For a standard 90 degree vertical FOV, we would
/// have: { UpTan = tan(90 degrees / 2), DownTan = tan(90 degrees / 2) }.
type FovPort struct {
	UpTan    float32 //< The tangent of the angle between the viewing vector and the top edge of the field of view.
	DownTan  float32 //< The tangent of the angle between the viewing vector and the bottom edge of the field of view.
	LeftTan  float32 //< The tangent of the angle between the viewing vector and the left edge of the field of view.
	RightTan float32 //< The tangent of the angle between the viewing vector and the right edge of the field of view.
}

/// Enumerates all HMD types that we support.
///
/// The currently released developer kits are ovrHmd_DK1 and ovrHmd_DK2. The other enumerations are for internal use only.
type HmdType int32

const (
	Hmd_None HmdType = 0
	Hmd_DK1
	Hmd_DKHD
	Hmd_DK2
	Hmd_CB
	Hmd_Other
	Hmd_E3_2015
	Hmd_ES06
	Hmd_ES09
	Hmd_ES11
	Hmd_CV1
)

/// HMD capability bits reported by device.
///
type HmdCaps int32

const (
	HmdCap_DebugDevice HmdCaps = 0 ///< <B>(read only)</B> Specifies that the HMD is a virtual debug device.
)

/// Tracking capability bits reported by the device.
/// Used with ovr_GetTrackingCaps.
type TrackingCaps int32

const (
	TrackingCap_Orientation      TrackingCaps = 0 ///< Supports orientation tracking (IMU).
	TrackingCap_MagYawCorrection                  ///< Supports yaw drift correction via a magnetometer or other means.
	TrackingCap_Position                          ///< Supports positional tracking.
)

/// Specifies which eye is being used for rendering.
/// This type explicitly does not include a third "NoStereo" monoscopic option, as such is
/// not required for an HMD-centered API.
type EyeType int32

const (
	Eye_Left  EyeType = 0 ///< The left eye, from the viewer's perspective.
	Eye_Right             ///< The right eye, from the viewer's perspective.
	Eye_Count             ///< \internal Count of enumerated elements.
)

/// Specifies the coordinate system ovrTrackingState returns tracking poses in.
/// Used with ovr_SetTrackingOriginType()
type TrackingOrigin int32

const (
	/// \brief Tracking system origin reported at eye (HMD) height
	/// \details Prefer using this origin when your application requires
	/// matching user's current physical head pose to a virtual head pose
	/// without any regards to a the height of the floor. Cockpit-based,
	/// or 3rd-person experiences are ideal candidates.
	/// When used, all poses in ovrTrackingState are reported as an offset
	/// transform from the profile calibrated or recentered HMD pose.
	/// It is recommended that apps using this origin type call ovr_RecenterTrackingOrigin
	/// prior to starting the VR experience, but notify the user before doing so
	/// to make sure the user is in a comfortable pose, facing a comfortable
	/// direction.
	TrackingOrigin_EyeLevel TrackingOrigin = 0

	/// \brief Tracking system origin reported at floor height
	/// \details Prefer using this origin when your application requires the
	/// physical floor height to match the virtual floor height, such as
	/// standing experiences.
	/// When used, all poses in ovrTrackingState are reported as an offset
	/// transform from the profile calibrated floor pose. Calling ovr_RecenterTrackingOrigin
	/// will recenter the X & Z axes as well as yaw, but the Y-axis (i.e. height) will continue
	/// to be reported using the floor height as the origin for all poses.
	TrackingOrigin_FloorLevel

	TrackingOrigin_Count ///< \internal Count of enumerated elements.
)

/// Identifies a graphics device in a platform-specific way.
/// For Windows this is a LUID type.
type GraphicsLuid struct {
	Reserved [8]byte
}

/// This is a complete descriptor of the HMD.
type HmdDesc struct {
	Type                  HmdType
	ProductName           string
	Manufacturer          string
	VendorId              int16
	ProductId             int16
	SerialNumber          [24]byte
	FirmwareMajor         int16
	FirmwareMinor         int16
	AvailableHmdCaps      HmdCaps
	DefaultHmdCaps        HmdCaps
	AvailableTrackingCaps TrackingCaps
	DefaultTrackingCaps   TrackingCaps
	DefaultEyeFov         [Eye_Count]FovPort
	MaxEyeFov             [Eye_Count]FovPort
	Resolution            Sizei
	DisplayRefreshRate    float32
}

/// Used as an opaque pointer to an OVR session.
type Session struct {
}

/// Bit flags describing the current status of sensor tracking.
///  The values must be the same as in enum StatusBits
///
/// \see ovrTrackingState
type StatusBits int32

const (
	Status_OrientationTracked StatusBits = 0 ///< Orientation is currently tracked (connected and in use).
	Status_PositionTracked                   ///< Position is currently tracked (false if out of range).
)

///  Specifies the description of a single sensor.
///
/// \see ovrGetTrackerDesc
type TrackerDesc struct {
	FrustumHFovInRadians float32
	FrustumVFovInRadians float32
	FrustumNearZInMeters float32
	FrustumFarZInMeters  float32
}

///  Specifies sensor flags.
///
///  /see ovrTrackerPose
type TrackerFlags int32

const (
	Tracker_Connected   TrackerFlags = 0 ///< The sensor is present, else the sensor is absent or offline.
	Tracker_PoseTracked                  ///< The sensor has a valid pose, else the pose is unavailable. This will only be set if ovrTracker_Connected is set.
)

///  Specifies the pose for a single sensor.
type TrackerPose struct {
	TrackerFlags uint32 ///< ovrTrackerFlags.
	Pose         Posef  ///< The sensor's pose. This pose includes sensor tilt (roll and pitch). For a leveled coordinate system use LeveledPose.
	LeveledPose  Posef  ///< The sensor's leveled pose, aligned with gravity. This value includes position and yaw of the sensor, but not roll and pitch. It can be used as a reference point to render real-world objects in the correct location.
}

/// Tracking state at a given absolute time (describes predicted HMD pose, etc.).
/// Returned by ovr_GetTrackingState.
///
/// \see ovr_GetTrackingState
type TrackingState struct {
	/// Predicted head pose (and derivatives) at the requested absolute time.
	HeadPose PoseStatef

	/// HeadPose tracking status described by ovrStatusBits.
	StatusFlags StatusBits

	/// The most recent calculated pose for each hand when hand controller tracking is present.
	/// HandPoses[ovrHand_Left] refers to the left hand and HandPoses[ovrHand_Right] to the right hand.
	/// These values can be combined with ovrInputState for complete hand controller information.
	HandPoses [2]PoseStatef

	/// HandPoses status flags described by ovrStatusBits.
	/// Only ovrStatus_OrientationTracked and ovrStatus_PositionTracked are reported.
	HandStatusFlags [2]StatusBits

	/// The pose of the origin captured during calibration.
	/// Like all other poses here, this is expressed in the space set by ovr_RecenterTrackingOrigin,
	/// and so will change every time that is called. This pose can be used to calculate
	/// where the calibrated origin lands in the new recentered space.
	/// If an application never calls ovr_RecenterTrackingOrigin, expect this value to be the identity
	/// pose and as such will point respective origin based on ovrTrackingOrigin requested when
	/// calling ovr_GetTrackingState.
	CalibratedOrigin Posef
}

/// Rendering information for each eye. Computed by ovr_GetRenderDesc() based on the
/// specified FOV. Note that the rendering viewport is not included
/// here as it can be specified separately and modified per frame by
/// passing different Viewport values in the layer structure.
///
/// \see ovr_GetRenderDesc
type EyeRenderDesc struct {
	Eye                       EyeType  ///< The eye index to which this instance corresponds.
	Fov                       FovPort  ///< The field of view.
	DistortedViewport         Recti    ///< Distortion viewport.
	PixelsPerTanAngleAtCenter Vector2f ///< How many display pixels will fit in tan(angle) = 1.
	HmdToEyeOffset            Vector3f ///< Translation of each eye, in meters.
}

/// Projection information for ovrLayerEyeFovDepth.
///
/// Use the utility function ovrTimewarpProjectionDesc_FromProjection to
/// generate this structure from the application's projection matrix.
///
/// \see ovrLayerEyeFovDepth, ovrTimewarpProjectionDesc_FromProjection
type TimewarpProjectionDesc struct {
	Projection22 float32 ///< Projection matrix element [2][2].
	Projection23 float32 ///< Projection matrix element [2][3].
	Projection32 float32 ///< Projection matrix element [3][2].
}

/// Contains the data necessary to properly calculate position info for various layer types.
/// - HmdToEyeOffset is the same value pair provided in ovrEyeRenderDesc.
/// - HmdSpaceToWorldScaleInMeters is used to scale player motion into in-application units.
///   In other words, it is how big an in-application unit is in the player's physical meters.
///   For example, if the application uses inches as its units then HmdSpaceToWorldScaleInMeters would be 0.0254.
///   Note that if you are scaling the player in size, this must also scale. So if your application
///   units are inches, but you're shrinking the player to half their normal size, then
///   HmdSpaceToWorldScaleInMeters would be 0.0254*2.0.
///
/// \see ovrEyeRenderDesc, ovr_SubmitFrame
type ViewScaleDesc struct {
	HmdToEyeOffset               [Eye_Count]Vector3f ///< Translation of each eye.
	HmdSpaceToWorldScaleInMeters float32             ///< Ratio of viewer units to meter units.
}

/// The type of texture resource.
///
/// \see ovrTextureSwapChainDesc
type TextureType int32

const (
	Texture_2D          TextureType = 0 ///< 2D textures.
	Texture_2D_External                 ///< External 2D texture. Not used on PC
	Texture_Cube                        ///< Cube maps. Not currently supported on PC.
	Texture_Count
)

/// The bindings required for texture swap chain.
///
/// All texture swap chains are automatically bindable as shader
/// input resources since the Oculus runtime needs this to read them.
///
/// \see ovrTextureSwapChainDesc
type TextureBindFlags int32

const (
	TextureBind_None               TextureBindFlags = 0
	TextureBind_DX_RenderTarget                     ///< The application can write into the chain with pixel shader
	TextureBind_DX_UnorderedAccess                  ///< The application can write to the chain with compute shader
	TextureBind_DX_DepthStencil                     ///< The chain buffers can be bound as depth and/or stencil buffers
)

/// The format of a texture.
///
/// \see ovrTextureSwapChainDesc
type TextureFormat int32

const (
	FORMAT_UNKNOWN        TextureFormat = 0
	FORMAT_B5G6R5_UNORM                 ///< Not currently supported on PC. Would require a DirectX 11.1 device.
	FORMAT_B5G5R5A1_UNORM               ///< Not currently supported on PC. Would require a DirectX 11.1 device.
	FORMAT_B4G4R4A4_UNORM               ///< Not currently supported on PC. Would require a DirectX 11.1 device.
	FORMAT_R8G8B8A8_UNORM
	FORMAT_R8G8B8A8_UNORM_SRGB
	FORMAT_B8G8R8A8_UNORM
	FORMAT_B8G8R8A8_UNORM_SRGB ///< Not supported for OpenGL applications
	FORMAT_B8G8R8X8_UNORM      ///< Not supported for OpenGL applications
	FORMAT_B8G8R8X8_UNORM_SRGB ///< Not supported for OpenGL applications
	FORMAT_R16G16B16A16_FLOAT
	FORMAT_D16_UNORM
	FORMAT_D24_UNORM_S8_UINT
	FORMAT_D32_FLOAT
	FORMAT_D32_FLOAT_S8X24_UINT
)

/// Misc flags overriding particular
///   behaviors of a texture swap chain
///
/// \see ovrTextureSwapChainDesc
type TextureMiscFlags int32

const (
	TextureMisc_None TextureMiscFlags = 0

	/// DX only: The underlying texture is created with a TYPELESS equivalent of the
	/// format specified in the texture desc. The SDK will still access the
	/// texture using the format specified in the texture desc, but the app can
	/// create views with different formats if this is specified.
	TextureMisc_DX_Typeless

	/// DX only: Allow generation of the mip chain on the GPU via the GenerateMips
	/// call. This flag requires that RenderTarget binding also be specified.
	TextureMisc_AllowGenerateMips
)

/// Description used to create a texture swap chain.
///
/// \see ovr_CreateTextureSwapChainDX
/// \see ovr_CreateTextureSwapChainGL
type TextureSwapChainDesc struct {
	Typ         TextureType
	Format      TextureFormat
	ArraySize   int ///< Only supported with ovrTexture_2D. Not supported on PC at this time.
	Width       int
	Height      int
	MipLevels   int
	SampleCount int              ///< Current only supported on depth textures
	StaticImage bool             ///< Not buffered in a chain. For images that don't change
	MiscFlags   TextureMiscFlags ///< ovrTextureMiscFlags
	BindFlags   TextureBindFlags ///< ovrTextureBindFlags. Not used for GL.
}

/// Description used to create a mirror texture.
///
/// \see ovr_CreateMirrorTextureDX
/// \see ovr_CreateMirrorTextureGL
type MirrorTextureDesc struct {
	Format    TextureFormat
	Width     int
	Height    int
	MiscFlags TextureMiscFlags ///< ovrTextureMiscFlags
}

type TextureSwapChain struct {
}

type MirrorTexture struct {
}

/// Describes button input types.
/// Button inputs are combined; that is they will be reported as pressed if they are
/// pressed on either one of the two devices.
/// The ovrButton_Up/Down/Left/Right map to both XBox D-Pad and directional buttons.
/// The ovrButton_Enter and ovrButton_Return map to Start and Back controller buttons, respectively.
type Button int

const (
	Button_A Button = 0
	Button_B
	Button_RThumb
	Button_RShoulder

	// Bit mask of all buttons on the right Touch controller
	Button_RMask

	Button_X
	Button_Y
	Button_LThumb
	Button_LShoulder

	// Bit mask of all buttons on the left Touch controller
	Button_LMask

	// Navigation through DPad.
	Button_Up
	Button_Down
	Button_Left
	Button_Right
	Button_Enter   // Start on XBox controller.
	Button_Back    // Back on Xbox controller.
	Button_VolUp   // only supported by Remote.
	Button_VolDown // only supported by Remote.
	Button_Home
	Button_Private
)

/// Describes touch input types.
/// These values map to capacitive touch values reported ovrInputState::Touch.
/// Some of these values are mapped to button bits for consistency.
type Touch int

const (
	Touch_A Touch = 0
	Touch_B
	Touch_RThumb
	Touch_RIndexTrigger

	// Bit mask of all buttons on the right Touch controller
	Touch_RButtonMask

	Touch_X
	Touch_Y
	Touch_LThumb
	Touch_LIndexTrigger

	// Bit mask of all the button touches on the left controller
	Touch_LButtonMask

	// Finger pose state
	// Derived internally based on distance, proximity to sensors and filtering.
	Touch_RIndexPointing
	Touch_RThumbUp

	// Bit mask of all right controller poses
	Touch_RPoseMask

	Touch_LIndexPointing
	Touch_LThumbUp

	// Bit mask of all left controller poses
	Touch_LPoseMask
)

/// Specifies which controller is connected; multiple can be connected at once.
type ControllerType int

const (
	ControllerType_None ControllerType = 0
	ControllerType_LTouch
	ControllerType_RTouch
	ControllerType_Touch
	ControllerType_Remote
	ControllerType_XBox

	ControllerType_Active ///< Operate on or query whichever controller is active.
)

/// Provides names for the left and right hand array indexes.
///
/// \see ovrInputState, ovrTrackingState
type HandType int

const (
	Hand_Left HandType = 0
	Hand_Right
	Hand_Count
)

/// ovrInputState describes the complete controller input state, including Oculus Touch,
/// and XBox gamepad. If multiple inputs are connected and used at the same time,
/// their inputs are combined.
type InputState struct {
	// System type when the controller state was last updated.
	TimeInSeconds float64

	// Values for buttons described by ovrButton.
	Buttons Button

	// Touch values for buttons and sensors as described by ovrTouch.
	Touches Touch

	// Left and right finger trigger values (ovrHand_Left and ovrHand_Right), in the range 0.0 to 1.0f.
	IndexTrigger [2]float32

	// Left and right hand trigger values (ovrHand_Left and ovrHand_Right), in the range 0.0 to 1.0f.
	HandTrigger [2]float32

	// Horizontal and vertical thumbstick axis values (ovrHand_Left and ovrHand_Right), in the range -1.0f to 1.0f.
	Thumbstick [2]Vector2f

	// The type of the controller this state is for.
	ControllerType ControllerType
}

/// Initialization flags.
///
/// \see ovrInitParams, ovr_Initialize
type InitFlags int32

const (
	/// When a debug library is requested, a slower debugging version of the library will
	/// run which can be used to help solve problems in the library and debug application code.
	Init_Debug InitFlags = 0

	/// When a version is requested, the LibOVR runtime respects the RequestedMinorVersion
	/// field and verifies that the RequestedMinorVersion is supported.
	Init_RequestVersion

	// These bits are writable by user code.
	Init_WritableBits
)

/// Logging levels
///
/// \see ovrInitParams, ovrLogCallback
type LogLevel int32

const (
	LogLevel_Debug LogLevel = 0 ///< Debug-level log event.
	LogLevel_Info               ///< Info-level log event.
	LogLevel_Error              ///< Error-level log event.
)

/// Signature of the logging callback function pointer type.
///
/// \param[in] userData is an arbitrary value specified by the user of ovrInitParams.
/// \param[in] level is one of the ovrLogLevel constants.
/// \param[in] message is a UTF8-encoded null-terminated string.
/// \see ovrInitParams, ovrLogLevel, ovr_Initialize
type LogCallback func(userData uintptr, level int, message string)

/// Parameters for ovr_Initialize.
///
/// \see ovr_Initialize
type InitParams struct {
	/// Flags from ovrInitFlags to override default behavior.
	/// Use 0 for the defaults.
	Flags InitFlags

	/// Requests a specific minimum minor version of the LibOVR runtime.
	/// Flags must include ovrInit_RequestVersion or this will be ignored
	/// and OVR_MINOR_VERSION will be used.
	RequestedMinorVersion uint32

	/// User-supplied log callback function, which may be called at any time
	/// asynchronously from multiple threads until ovr_Shutdown completes.
	/// Use NULL to specify no log callback.
	LogCallback LogCallback

	/// User-supplied data which is passed as-is to LogCallback. Typically this
	/// is used to store an application-specific pointer which is read in the
	/// callback function.
	UserData uintptr

	/// Relative number of milliseconds to wait for a connection to the server
	/// before failing. Use 0 for the default timeout.
	ConnectionTimeoutMS uint32
}

// -----------------------------------------------------------------------------------
// ***** API Interfaces

// Overview of the API
//
// Setup:
//  - ovr_Initialize().
//  - ovr_Create(&hmd, &graphicsId).
//  - Use hmd members and ovr_GetFovTextureSize() to determine graphics configuration
//    and ovr_GetRenderDesc() to get per-eye rendering parameters.
//  - Allocate texture swap chains with ovr_CreateTextureSwapChainDX() or
//    ovr_CreateTextureSwapChainGL(). Create any associated render target views or
//    frame buffer objects.
//
// Application Loop:
//  - Call ovr_GetPredictedDisplayTime() to get the current frame timing information.
//  - Call ovr_GetTrackingState() and ovr_CalcEyePoses() to obtain the predicted
//    rendering pose for each eye based on timing.
//  - Render the scene content into the current buffer of the texture swapchains
//    for each eye and layer you plan to update this frame. If you render into a
//    texture swap chain, you must call ovr_CommitTextureSwapChain() on it to commit
//    the changes before you reference the chain this frame (otherwise, your latest
//    changes won't be picked up).
//  - Call ovr_SubmitFrame() to render the distorted layers to and present them on the HMD.
//    If ovr_SubmitFrame returns ovrSuccess_NotVisible, there is no need to render the scene
//    for the next loop iteration. Instead, just call ovr_SubmitFrame again until it returns
//    ovrSuccess.
//
// Shutdown:
//  - ovr_Destroy().
//  - ovr_Shutdown().

/// Initializes LibOVR
///
/// Initialize LibOVR for application usage. This includes finding and loading the LibOVRRT
/// shared library. No LibOVR API functions, other than ovr_GetLastErrorInfo, can be called
/// unless ovr_Initialize succeeds. A successful call to ovr_Initialize must be eventually
/// followed by a call to ovr_Shutdown. ovr_Initialize calls are idempotent.
/// Calling ovr_Initialize twice does not require two matching calls to ovr_Shutdown.
/// If already initialized, the return value is ovr_Success.
///
/// LibOVRRT shared library search order:
///      -# Current working directory (often the same as the application directory).
///      -# Module directory (usually the same as the application directory,
///         but not if the module is a separate shared library).
///      -# Application directory
///      -# Development directory (only if OVR_ENABLE_DEVELOPER_SEARCH is enabled,
///         which is off by default).
///      -# Standard OS shared library search location(s) (OS-specific).
///
/// \param params Specifies custom initialization options. May be NULL to indicate default options.
/// \return Returns an ovrResult indicating success or failure. In the case of failure, use
///         ovr_GetLastErrorInfo to get more information. Example failed results include:
///     - ovrError_Initialize: Generic initialization error.
///     - ovrError_LibLoad: Couldn't load LibOVRRT.
///     - ovrError_LibVersion: LibOVRRT version incompatibility.
///     - ovrError_ServiceConnection: Couldn't connect to the OVR Service.
///     - ovrError_ServiceVersion: OVR Service version incompatibility.
///     - ovrError_IncompatibleOS: The operating system version is incompatible.
///     - ovrError_DisplayInit: Unable to initialize the HMD display.
///     - ovrError_ServerStart:  Unable to start the server. Is it already running?
///     - ovrError_Reinitialization: Attempted to re-initialize with a different version.
///
/// <b>Example code</b>
///     \code{.cpp}
///         ovrResult result = ovr_Initialize(NULL);
///         if(OVR_FAILURE(result)) {
///             ovrErrorInfo errorInfo;
///             ovr_GetLastErrorInfo(&errorInfo);
///             DebugLog("ovr_Initialize failed: %s", errorInfo.ErrorString);
///             return false;
///         }
///         [...]
///     \endcode
///
/// \see ovr_Shutdown
func Initialize(params *InitParams) error {
	return errNoImpl
}

/// Shuts down LibOVR
///
/// A successful call to ovr_Initialize must be eventually matched by a call to ovr_Shutdown.
/// After calling ovr_Shutdown, no LibOVR functions can be called except ovr_GetLastErrorInfo
/// or another ovr_Initialize. ovr_Shutdown invalidates all pointers, references, and created objects
/// previously returned by LibOVR functions. The LibOVRRT shared library can be unloaded by
/// ovr_Shutdown.
///
/// \see ovr_Initialize
func Shutdown() {
}

/// Returns information about the most recent failed return value by the
/// current thread for this library.
///
/// This function itself can never generate an error.
/// The last error is never cleared by LibOVR, but will be overwritten by new errors.
/// Do not use this call to determine if there was an error in the last API
/// call as successful API calls don't clear the last ovrErrorInfo.
/// To avoid any inconsistency, ovr_GetLastErrorInfo should be called immediately
/// after an API function that returned a failed ovrResult, with no other API
/// functions called in the interim.
///
/// \param[out] errorInfo The last ovrErrorInfo for the current thread.
///
/// \see ovrErrorInfo
func GetLastErrorInfo() ErrorInfo {
	return ErrorInfo{}
}

/// Returns the version string representing the LibOVRRT version.
///
/// The returned string pointer is valid until the next call to ovr_Shutdown.
///
/// Note that the returned version string doesn't necessarily match the current
/// OVR_MAJOR_VERSION, etc., as the returned string refers to the LibOVRRT shared
/// library version and not the locally compiled interface version.
///
/// The format of this string is subject to change in future versions and its contents
/// should not be interpreted.
///
/// \return Returns a UTF8-encoded null-terminated version string.
func GetVersionString() string {
	return ""
}

/// Writes a message string to the LibOVR tracing mechanism (if enabled).
///
/// This message will be passed back to the application via the ovrLogCallback if
/// it was registered.
///
/// \param[in] level One of the ovrLogLevel constants.
/// \param[in] message A UTF8-encoded null-terminated string.
/// \return returns the strlen of the message or a negative value if the message is too large.
///
/// \see ovrLogLevel, ovrLogCallback
func TraceMessage(level int, message string) (int, error) {
	return 0, errNoImpl
}

/// Returns information about the current HMD.
///
/// ovr_Initialize must have first been called in order for this to succeed, otherwise ovrHmdDesc::Type
/// will be reported as ovrHmd_None.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create, else NULL in which
///                case this function detects whether an HMD is present and returns its info if so.
///
/// \return Returns an ovrHmdDesc. If the hmd is NULL and ovrHmdDesc::Type is ovrHmd_None then
///         no HMD is present.
func (s *Session) GetHmdDesc() *HmdDesc {
	return nil
}

/// Returns the number of sensors.
///
/// The number of sensors may change at any time, so this function should be called before use
/// as opposed to once on startup.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
///
/// \return Returns unsigned int count.
func (s *Session) GetTrackerCount() uint {
	return 0
}

/// Returns a given sensor description.
///
/// It's possible that sensor desc [0] may indicate a unconnnected or non-pose tracked sensor, but
/// sensor desc [1] may be connected.
///
/// ovr_Initialize must have first been called in order for this to succeed, otherwise the returned
/// trackerDescArray will be zero-initialized. The data returned by this function can change at runtime.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
///
/// \param[in] trackerDescIndex Specifies a sensor index. The valid indexes are in the range of 0 to
///            the sensor count returned by ovr_GetTrackerCount.
///
/// \return Returns ovrTrackerDesc. An empty ovrTrackerDesc will be returned if trackerDescIndex is out of range.
///
/// \see ovrTrackerDesc, ovr_GetTrackerCount
func (s *Session) GetTrackerDesc(trackerDescIndex uint) TrackerDesc {
	return TrackerDesc{}
}

/// Creates a handle to a VR session.
///
/// Upon success the returned ovrSession must be eventually freed with ovr_Destroy when it is no longer needed.
/// A second call to ovr_Create will result in an error return value if the previous Hmd has not been destroyed.
///
/// \param[out] pSession Provides a pointer to an ovrSession which will be written to upon success.
/// \param[out] luid Provides a system specific graphics adapter identifier that locates which
/// graphics adapter has the HMD attached. This must match the adapter used by the application
/// or no rendering output will be possible. This is important for stability on multi-adapter systems. An
/// application that simply chooses the default adapter will not run reliably on multi-adapter systems.
/// \return Returns an ovrResult indicating success or failure. Upon failure
///         the returned pHmd will be NULL.
///
/// <b>Example code</b>
///     \code{.cpp}
///         ovrSession session;
///         ovrGraphicsLuid luid;
///         ovrResult result = ovr_Create(&session, &luid);
///         if(OVR_FAILURE(result))
///            ...
///     \endcode
///
/// \see ovr_Destroy
func Create(pLuid *GraphicsLuid) (*Session, error) {
	return nil, errNoImpl
}

/// Destroys the HMD.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \see ovr_Create
func (s *Session) Destroy() {
}

/// Specifies status information for the current session.
///
/// \see ovr_GetSessionStatus
type SessionStatus struct {
	IsVisible      bool ///< True if the process has VR focus and thus is visible in the HMD.
	HmdPresent     bool ///< True if an HMD is present.
	HmdMounted     bool ///< True if the HMD is on the user's head.
	DisplayLost    bool ///< True if the session is in a display-lost state. See ovr_SubmitFrame.
	ShouldQuit     bool ///< True if the application should initiate shutdown.
	ShouldRecenter bool ///< True if UX has requested re-centering. Must call ovr_ClearShouldRecenterFlag or ovr_RecenterTrackingOrigin.
}

/// Returns status information for the application.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[out] sessionStatus Provides an ovrSessionStatus that is filled in.
///
/// \return Returns an ovrResult indicating success or failure. In the case of
///         failure, use ovr_GetLastErrorInfo to get more information.
//          Return values include but aren't limited to:
///     - ovrSuccess: Completed successfully.
///     - ovrError_ServiceConnection: The service connection was lost and the application
//        must destroy the session.
func (s *Session) GetSessionStatus() (SessionStatus, error) {
	return SessionStatus{}, errNoImpl
}

/// Sets the tracking origin type
///
/// When the tracking origin is changed, all of the calls that either provide
/// or accept ovrPosef will use the new tracking origin provided.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] origin Specifies an ovrTrackingOrigin to be used for all ovrPosef
///
/// \return Returns an ovrResult indicating success or failure. In the case of failure, use
///         ovr_GetLastErrorInfo to get more information.
///
/// \see ovrTrackingOrigin, ovr_GetTrackingOriginType
func (s *Session) SetTrackingOriginType(origin TrackingOrigin) {
}

/// Gets the tracking origin state
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
///
/// \return Returns the ovrTrackingOrigin that was either set by default, or previous set by the application.
///
/// \see ovrTrackingOrigin, ovr_SetTrackingOriginType
func (s *Session) GetTrackingOriginType() TrackingOrigin {
	return 0
}

/// Re-centers the sensor position and orientation.
///
/// This resets the (x,y,z) positional components and the yaw orientation component.
/// The Roll and pitch orientation components are always determined by gravity and cannot
/// be redefined. All future tracking will report values relative to this new reference position.
/// If you are using ovrTrackerPoses then you will need to call ovr_GetTrackerPose after
/// this, because the sensor position(s) will change as a result of this.
///
/// The headset cannot be facing vertically upward or downward but rather must be roughly
/// level otherwise this function will fail with ovrError_InvalidHeadsetOrientation.
///
/// For more info, see the notes on each ovrTrackingOrigin enumeration to understand how
/// recenter will vary slightly in its behavior based on the current ovrTrackingOrigin setting.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
///
/// \return Returns an ovrResult indicating success or failure. In the case of failure, use
///         ovr_GetLastErrorInfo to get more information. Return values include but aren't limited to:
///     - ovrSuccess: Completed successfully.
///     - ovrError_InvalidHeadsetOrientation: The headset was facing an invalid direction when
///       attempting recentering, such as facing vertically.
///
/// \see ovrTrackingOrigin, ovr_GetTrackerPose
func (s *Session) RecenterTrackingOrigin() error {
	return errNoImpl
}

/// Clears the ShouldRecenter status bit in ovrSessionStatus.
///
/// Clears the ShouldRecenter status bit in ovrSessionStatus, allowing further recenter
/// requests to be detected. Since this is automatically done by ovr_RecenterTrackingOrigin,
/// this is only needs to be called when application is doing its own re-centering.
func (s *Session) ClearShouldRecenterFlag() {
}

/// Returns tracking state reading based on the specified absolute system time.
///
/// Pass an absTime value of 0.0 to request the most recent sensor reading. In this case
/// both PredictedPose and SamplePose will have the same value.
///
/// This may also be used for more refined timing of front buffer rendering logic, and so on.
/// This may be called by multiple threads.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] absTime Specifies the absolute future time to predict the return
///            ovrTrackingState value. Use 0 to request the most recent tracking state.
/// \param[in] latencyMarker Specifies that this call is the point in time where
///            the "App-to-Mid-Photon" latency timer starts from. If a given ovrLayer
///            provides "SensorSampleTimestamp", that will override the value stored here.
/// \return Returns the ovrTrackingState that is predicted for the given absTime.
///
/// \see ovrTrackingState, ovr_GetEyePoses, ovr_GetTimeInSeconds
func (s *Session) GetTrackingState(absTime float64, latencyMarker bool) TrackingState {
	return TrackingState{}
}

/// Returns the ovrTrackerPose for the given sensor.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] trackerPoseIndex Index of the sensor being requested.
///
/// \return Returns the requested ovrTrackerPose. An empty ovrTrackerPose will be returned if trackerPoseIndex is out of range.
///
/// \see ovr_GetTrackerCount
///
func (s *Session) GetTrackerPose(trackerPoseIndex uint) TrackerPose {
	return TrackerPose{}
}

/// Returns the most recent input state for controllers, without positional tracking info.
///
/// \param[out] inputState Input state that will be filled in.
/// \param[in] ovrControllerType Specifies which controller the input will be returned for.
/// \return Returns ovrSuccess if the new state was successfully obtained.
///
/// \see ovrControllerType
func (s *Session) GetInputState(controllerType ControllerType) (InputState, error) {
	return InputState{}, errNoImpl
}

/// Returns controller types connected to the system OR'ed together.
///
/// \return A bitmask of ovrControllerTypes connected to the system.
///
/// \see ovrControllerType
func (s *Session) GetConnectedControllerTypes() uint {
	return 0
}

/// Turns on vibration of the given controller.
///
/// To disable vibration, call ovr_SetControllerVibration with an amplitude of 0.
/// Vibration automatically stops after a nominal amount of time, so if you want vibration
/// to be continuous over multiple seconds then you need to call this function periodically.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] controllerType Specifies the controller to apply the vibration to.
/// \param[in] frequency Specifies a vibration frequency in the range of 0.0 to 1.0.
///            Currently the only valid values are 0.0, 0.5, and 1.0 and other values will
///            be clamped to one of these.
/// \param[in] amplitude Specifies a vibration amplitude in the range of 0.0 to 1.0.
///
/// \return Returns ovrSuccess upon success.
///
/// \see ovrControllerType
func (s *Session) SetControllerVibration(controllerType ControllerType, frequency, amplitude float32) error {
	return errNoImpl
}

///  Specifies the maximum number of layers supported by ovr_SubmitFrame.
///
///  /see ovr_SubmitFrame
const MaxLayerCount = 0

/// Describes layer types that can be passed to ovr_SubmitFrame.
/// Each layer type has an associated struct, such as ovrLayerEyeFov.
///
/// \see ovrLayerHeader
type LayerType int32

const (
	LayerType_Disabled LayerType = 0 ///< Layer is disabled.
	LayerType_EyeFov                 ///< Described by ovrLayerEyeFov.
	LayerType_Quad                   ///< Described by ovrLayerQuad. Previously called ovrLayerType_QuadInWorld.
	/// enum 4 used to be ovrLayerType_QuadHeadLocked. Instead, use ovrLayerType_Quad with ovrLayerFlag_HeadLocked.
	LayerType_EyeMatrix ///< Described by ovrLayerEyeMatrix.
)

/// Identifies flags used by ovrLayerHeader and which are passed to ovr_SubmitFrame.
///
/// \see ovrLayerHeader
type LayerFlags int32

const (
	/// ovrLayerFlag_HighQuality enables 4x anisotropic sampling during the composition of the layer.
	/// The benefits are mostly visible at the periphery for high-frequency & high-contrast visuals.
	/// For best results consider combining this flag with an ovrTextureSwapChain that has mipmaps and
	/// instead of using arbitrary sized textures, prefer texture sizes that are powers-of-two.
	/// Actual rendered viewport and doesn't necessarily have to fill the whole texture.
	LayerFlag_HighQuality LayerFlags = 0

	/// ovrLayerFlag_TextureOriginAtBottomLeft: the opposite is TopLeft.
	/// Generally this is false for D3D, true for OpenGL.
	LayerFlag_TextureOriginAtBottomLeft

	/// Mark this surface as "headlocked", which means it is specified
	/// relative to the HMD and moves with it, rather than being specified
	/// relative to sensor/torso space and remaining still while the head moves.
	/// What used to be ovrLayerType_QuadHeadLocked is now ovrLayerType_Quad plus this flag.
	/// However the flag can be applied to any layer type to achieve a similar effect.
	LayerFlag_HeadLocked
)

/// Defines properties shared by all ovrLayer structs, such as ovrLayerEyeFov.
///
/// ovrLayerHeader is used as a base member in these larger structs.
/// This struct cannot be used by itself except for the case that Type is ovrLayerType_Disabled.
///
/// \see ovrLayerType, ovrLayerFlags
type LayerHeader struct {
	Type  LayerType
	Flags LayerFlags
}

// Added for the go bindings
type LayerInterface interface {
}

/// Describes a layer that specifies a monoscopic or stereoscopic view.
/// This is the kind of layer that's typically used as layer 0 to ovr_SubmitFrame,
/// as it is the kind of layer used to render a 3D stereoscopic view.
///
/// Three options exist with respect to mono/stereo texture usage:
///    - ColorTexture[0] and ColorTexture[1] contain the left and right stereo renderings, respectively.
///      Viewport[0] and Viewport[1] refer to ColorTexture[0] and ColorTexture[1], respectively.
///    - ColorTexture[0] contains both the left and right renderings, ColorTexture[1] is NULL,
///      and Viewport[0] and Viewport[1] refer to sub-rects with ColorTexture[0].
///    - ColorTexture[0] contains a single monoscopic rendering, and Viewport[0] and
///      Viewport[1] both refer to that rendering.
///
/// \see ovrTextureSwapChain, ovr_SubmitFrame
type LayerEyeFov struct {
	/// Header.Type must be ovrLayerType_EyeFov.
	Header LayerHeader

	/// ovrTextureSwapChains for the left and right eye respectively.
	/// The second one of which can be NULL for cases described above.
	ColorTexture [Eye_Count]TextureSwapChain

	/// Specifies the ColorTexture sub-rect UV coordinates.
	/// Both Viewport[0] and Viewport[1] must be valid.
	Viewport [Eye_Count]Recti

	/// The viewport field of view.
	Fov [Eye_Count]FovPort

	/// Specifies the position and orientation of each eye view, with the position specified in meters.
	/// RenderPose will typically be the value returned from ovr_CalcEyePoses,
	/// but can be different in special cases if a different head pose is used for rendering.
	RenderPose [Eye_Count]Posef

	/// Specifies the timestamp when the source ovrPosef (used in calculating RenderPose)
	/// was sampled from the SDK. Typically retrieved by calling ovr_GetTimeInSeconds
	/// around the instant the application calls ovr_GetTrackingState
	/// The main purpose for this is to accurately track app tracking latency.
	SensorSampleTime float64
}

/// Describes a layer that specifies a monoscopic or stereoscopic view.
/// This uses a direct 3x4 matrix to map from view space to the UV coordinates.
/// It is essentially the same thing as ovrLayerEyeFov but using a much
/// lower level. This is mainly to provide compatibility with specific apps.
/// Unless the application really requires this flexibility, it is usually better
/// to use ovrLayerEyeFov.
///
/// Three options exist with respect to mono/stereo texture usage:
///    - ColorTexture[0] and ColorTexture[1] contain the left and right stereo renderings, respectively.
///      Viewport[0] and Viewport[1] refer to ColorTexture[0] and ColorTexture[1], respectively.
///    - ColorTexture[0] contains both the left and right renderings, ColorTexture[1] is NULL,
///      and Viewport[0] and Viewport[1] refer to sub-rects with ColorTexture[0].
///    - ColorTexture[0] contains a single monoscopic rendering, and Viewport[0] and
///      Viewport[1] both refer to that rendering.
///
/// \see ovrTextureSwapChain, ovr_SubmitFrame
type LayerEyeMatrix struct {
	/// Header.Type must be ovrLayerType_EyeMatrix.
	Header LayerHeader

	/// ovrTextureSwapChains for the left and right eye respectively.
	/// The second one of which can be NULL for cases described above.
	ColorTexture [Eye_Count]TextureSwapChain

	/// Specifies the ColorTexture sub-rect UV coordinates.
	/// Both Viewport[0] and Viewport[1] must be valid.
	Viewport [Eye_Count]Recti

	/// Specifies the position and orientation of each eye view, with the position specified in meters.
	/// RenderPose will typically be the value returned from ovr_CalcEyePoses,
	/// but can be different in special cases if a different head pose is used for rendering.
	RenderPose [Eye_Count]Posef

	/// Specifies the mapping from a view-space vector
	/// to a UV coordinate on the textures given above.
	/// P = (x,y,z,1)*Matrix
	/// TexU  = P.x/P.z
	/// TexV  = P.y/P.z
	Matrix [Eye_Count]Matrix4f

	/// Specifies the timestamp when the source ovrPosef (used in calculating RenderPose)
	/// was sampled from the SDK. Typically retrieved by calling ovr_GetTimeInSeconds
	/// around the instant the application calls ovr_GetTrackingState
	/// The main purpose for this is to accurately track app tracking latency.
	SensorSampleTime float64
}

/// Describes a layer of Quad type, which is a single quad in world or viewer space.
/// It is used for ovrLayerType_Quad. This type of layer represents a single
/// object placed in the world and not a stereo view of the world itself.
///
/// A typical use of ovrLayerType_Quad is to draw a television screen in a room
/// that for some reason is more convenient to draw as a layer than as part of the main
/// view in layer 0. For example, it could implement a 3D popup GUI that is drawn at a
/// higher resolution than layer 0 to improve fidelity of the GUI.
///
/// Quad layers are visible from both sides; they are not back-face culled.
///
/// \see ovrTextureSwapChain, ovr_SubmitFrame
type LayerQuad struct {
	/// Header.Type must be ovrLayerType_Quad.
	Header LayerHeader

	/// Contains a single image, never with any stereo view.
	ColorTexture TextureSwapChain

	/// Specifies the ColorTexture sub-rect UV coordinates.
	Viewport Recti

	/// Specifies the orientation and position of the center point of a Quad layer type.
	/// The supplied direction is the vector perpendicular to the quad.
	/// The position is in real-world meters (not the application's virtual world,
	/// the physical world the user is in) and is relative to the "zero" position
	/// set by ovr_RecenterTrackingOrigin unless the ovrLayerFlag_HeadLocked flag is used.
	QuadPoseCenter Posef

	/// Width and height (respectively) of the quad in meters.
	QuadSize Vector2f
}

/// @name SDK Distortion Rendering
///
/// All of rendering functions including the configure and frame functions
/// are not thread safe. It is OK to use ConfigureRendering on one thread and handle
/// frames on another thread, but explicit synchronization must be done since
/// functions that depend on configured state are not reentrant.
///
/// These functions support rendering of distortion by the SDK.
///
//@{

/// TextureSwapChain creation is rendering API-specific.
/// ovr_CreateTextureSwapChainDX and ovr_CreateTextureSwapChainGL can be found in the
/// rendering API-specific headers, such as OVR_CAPI_D3D.h and OVR_CAPI_GL.h

/// Gets the number of buffers in an ovrTextureSwapChain.
///
/// \param[in]  session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in]  chain Specifies the ovrTextureSwapChain for which the length should be retrieved.
/// \param[out] out_Length Returns the number of buffers in the specified chain.
///
/// \return Returns an ovrResult for which OVR_SUCCESS(result) is false upon error.
///
/// \see ovr_CreateTextureSwapChainDX, ovr_CreateTextureSwapChainGL
func (s *Session) GetTextureSwapChainLength(chain TextureSwapChain) (int, error) {
	return 0, errNoImpl
}

/// Gets the current index in an ovrTextureSwapChain.
///
/// \param[in]  session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in]  chain Specifies the ovrTextureSwapChain for which the index should be retrieved.
/// \param[out] out_Index Returns the current (free) index in specified chain.
///
/// \return Returns an ovrResult for which OVR_SUCCESS(result) is false upon error.
///
/// \see ovr_CreateTextureSwapChainDX, ovr_CreateTextureSwapChainGL
func (s *Session) GetTextureSwapChainCurrentIndex(chain TextureSwapChain) (int, error) {
	return 0, errNoImpl
}

/// Gets the description of the buffers in an ovrTextureSwapChain
///
/// \param[in]  session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in]  chain Specifies the ovrTextureSwapChain for which the description should be retrieved.
/// \param[out] out_Desc Returns the description of the specified chain.
///
/// \return Returns an ovrResult for which OVR_SUCCESS(result) is false upon error.
///
/// \see ovr_CreateTextureSwapChainDX, ovr_CreateTextureSwapChainGL
func (s *Session) GetTextureSwapChainDesc(chain TextureSwapChain) (TextureSwapChainDesc, error) {
	return TextureSwapChainDesc{}, errNoImpl
}

/// Commits any pending changes to an ovrTextureSwapChain, and advances its current index
///
/// \param[in]  session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in]  chain Specifies the ovrTextureSwapChain to commit.
///
/// \note When Commit is called, the texture at the current index is considered ready for use by the
/// runtime, and further writes to it should be avoided. The swap chain's current index is advanced,
/// providing there's room in the chain. The next time the SDK dereferences this texture swap chain,
/// it will synchronize with the app's graphics context and pick up the submitted index, opening up
/// room in the swap chain for further commits.
///
/// \return Returns an ovrResult for which OVR_SUCCESS(result) is false upon error.
///         Failures include but aren't limited to:
///     - ovrError_TextureSwapChainFull: ovr_CommitTextureSwapChain was called too many times on a texture swapchain without calling submit to use the chain.
///
/// \see ovr_CreateTextureSwapChainDX, ovr_CreateTextureSwapChainGL
func (s *Session) CommitTextureSwapChain(chain TextureSwapChain) error {
	return errNoImpl
}

/// Destroys an ovrTextureSwapChain and frees all the resources associated with it.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] chain Specifies the ovrTextureSwapChain to destroy. If it is NULL then this function has no effect.
///
/// \see ovr_CreateTextureSwapChainDX, ovr_CreateTextureSwapChainGL
func (s *Session) DestroyTextureSwapChain(chain TextureSwapChain) {
}

/// MirrorTexture creation is rendering API-specific.
/// ovr_CreateMirrorTextureDX and ovr_CreateMirrorTextureGL can be found in the
/// rendering API-specific headers, such as OVR_CAPI_D3D.h and OVR_CAPI_GL.h

/// Destroys a mirror texture previously created by one of the mirror texture creation functions.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] mirrorTexture Specifies the ovrTexture to destroy. If it is NULL then this function has no effect.
///
/// \see ovr_CreateMirrorTextureDX, ovr_CreateMirrorTextureGL
func (s *Session) DestroyMirrorTexture(mirrorTexture MirrorTexture) {
}

/// Calculates the recommended viewport size for rendering a given eye within the HMD
/// with a given FOV cone.
///
/// Higher FOV will generally require larger textures to maintain quality.
/// Apps packing multiple eye views together on the same texture should ensure there are
/// at least 8 pixels of padding between them to prevent texture filtering and chromatic
/// aberration causing images to leak between the two eye views.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] eye Specifies which eye (left or right) to calculate for.
/// \param[in] fov Specifies the ovrFovPort to use.
/// \param[in] pixelsPerDisplayPixel Specifies the ratio of the number of render target pixels
///            to display pixels at the center of distortion. 1.0 is the default value. Lower
///            values can improve performance, higher values give improved quality.
/// \return Returns the texture width and height size.
func (s *Session) GetFovTextureSize(eye EyeType, fov FovPort, pixelsPerDisplayPixel float32) Sizei {
	return Sizei{}
}

/// Computes the distortion viewport, view adjust, and other rendering parameters for
/// the specified eye.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] eyeType Specifies which eye (left or right) for which to perform calculations.
/// \param[in] fov Specifies the ovrFovPort to use.
///
/// \return Returns the computed ovrEyeRenderDesc for the given eyeType and field of view.
///
/// \see ovrEyeRenderDesc
func (s *Session) GetRenderDesc(eye EyeType, fov FovPort) EyeRenderDesc {
	return EyeRenderDesc{}
}

/// Submits layers for distortion and display.
///
/// ovr_SubmitFrame triggers distortion and processing which might happen asynchronously.
/// The function will return when there is room in the submission queue and surfaces
/// are available. Distortion might or might not have completed.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
///
/// \param[in] frameIndex Specifies the targeted application frame index, or 0 to refer to one frame
///        after the last time ovr_SubmitFrame was called.
///
/// \param[in] viewScaleDesc Provides additional information needed only if layerPtrList contains
///        an ovrLayerType_Quad. If NULL, a default version is used based on the current configuration and a 1.0 world scale.
///
/// \param[in] layerPtrList Specifies a list of ovrLayer pointers, which can include NULL entries to
///        indicate that any previously shown layer at that index is to not be displayed.
///        Each layer header must be a part of a layer structure such as ovrLayerEyeFov or ovrLayerQuad,
///        with Header.Type identifying its type. A NULL layerPtrList entry in the array indicates the
//         absence of the given layer.
///
/// \param[in] layerCount Indicates the number of valid elements in layerPtrList. The maximum
///        supported layerCount is not currently specified, but may be specified in a future version.
///
/// - Layers are drawn in the order they are specified in the array, regardless of the layer type.
///
/// - Layers are not remembered between successive calls to ovr_SubmitFrame. A layer must be
///   specified in every call to ovr_SubmitFrame or it won't be displayed.
///
/// - If a layerPtrList entry that was specified in a previous call to ovr_SubmitFrame is
///   passed as NULL or is of type ovrLayerType_Disabled, that layer is no longer displayed.
///
/// - A layerPtrList entry can be of any layer type and multiple entries of the same layer type
///   are allowed. No layerPtrList entry may be duplicated (i.e. the same pointer as an earlier entry).
///
/// <b>Example code</b>
///     \code{.cpp}
///         ovrLayerEyeFov  layer0;
///         ovrLayerQuad    layer1;
///           ...
///         ovrLayerHeader* layers[2] = { &layer0.Header, &layer1.Header };
///         ovrResult result = ovr_SubmitFrame(hmd, frameIndex, nullptr, layers, 2);
///     \endcode
///
/// \return Returns an ovrResult for which OVR_SUCCESS(result) is false upon error and true
///         upon success. Return values include but aren't limited to:
///     - ovrSuccess: rendering completed successfully.
///     - ovrSuccess_NotVisible: rendering completed successfully but was not displayed on the HMD,
///       usually because another application currently has ownership of the HMD. Applications receiving
///       this result should stop rendering new content, but continue to call ovr_SubmitFrame periodically
///       until it returns a value other than ovrSuccess_NotVisible.
///     - ovrError_DisplayLost: The session has become invalid (such as due to a device removal)
///       and the shared resources need to be released (ovr_DestroyTextureSwapChain), the session needs to
///       destroyed (ovr_Destroy) and recreated (ovr_Create), and new resources need to be created
///       (ovr_CreateTextureSwapChainXXX). The application's existing private graphics resources do not
///       need to be recreated unless the new ovr_Create call returns a different GraphicsLuid.
///     - ovrError_TextureSwapChainInvalid: The ovrTextureSwapChain is in an incomplete or inconsistent state.
///       Ensure ovr_CommitTextureSwapChain was called at least once first.
///
/// \see ovr_GetPredictedDisplayTime, ovrViewScaleDesc, ovrLayerHeader
func (s *Session) SubmitFrame(frameIndex uint64, viewScaleDesc *ViewScaleDesc, layers []LayerInterface) (bool, error) {
	return false, errNoImpl
}

//-------------------------------------------------------------------------------------
/// @name Frame Timing
///
//@{

/// Gets the time of the specified frame midpoint.
///
/// Predicts the time at which the given frame will be displayed. The predicted time
/// is the middle of the time period during which the corresponding eye images will
/// be displayed.
///
/// The application should increment frameIndex for each successively targeted frame,
/// and pass that index to any relevent OVR functions that need to apply to the frame
/// identified by that index.
///
/// This function is thread-safe and allows for multiple application threads to target
/// their processing to the same displayed frame.
///
/// In the even that prediction fails due to various reasons (e.g. the display being off
/// or app has yet to present any frames), the return value will be current CPU time.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] frameIndex Identifies the frame the caller wishes to target.
///            A value of zero returns the next frame index.
/// \return Returns the absolute frame midpoint time for the given frameIndex.
/// \see ovr_GetTimeInSeconds
///
func (s *Session) GetPredictedDisplayTime(frameIndex uint64) float64 {
	return 0
}

/// Returns global, absolute high-resolution time in seconds.
///
/// The time frame of reference for this function is not specified and should not be
/// depended upon.
///
/// \return Returns seconds as a floating point value.
/// \see ovrPoseStatef, ovrFrameTiming
func GetTimeInSeconds() float64 {
	return 0
}

/// Performance HUD enables the HMD user to see information critical to
/// the real-time operation of the VR application such as latency timing,
/// and CPU & GPU performance metrics
///
///     App can toggle performance HUD modes as such:
///     \code{.cpp}
///         ovrPerfHudMode PerfHudMode = ovrPerfHud_LatencyTiming;
///         ovr_SetInt(Hmd, OVR_PERF_HUD_MODE, (int)PerfHudMode);
///     \endcode
type PerfHudMode int32

const (
	PerfHud_Off              PerfHudMode = 0 ///< Turns off the performance HUD
	PerfHud_PerfSummary                      ///< Shows performance summary and headroom
	PerfHud_LatencyTiming                    ///< Shows latency related timing info
	PerfHud_AppRenderTiming                  ///< Shows render timing info for application
	PerfHud_CompRenderTiming                 ///< Shows render timing info for OVR compositor
	PerfHud_VersionInfo                      ///< Shows SDK & HMD version Info
	PerfHud_Count                            ///< \internal Count of enumerated elements.
)

/// Layer HUD enables the HMD user to see information about a layer
///
///     App can toggle layer HUD modes as such:
///     \code{.cpp}
///         ovrLayerHudMode LayerHudMode = ovrLayerHud_Info;
///         ovr_SetInt(Hmd, OVR_LAYER_HUD_MODE, (int)LayerHudMode);
///     \endcode
type LayerHudMode int32

const (
	LayerHud_Off  LayerHudMode = 0 ///< Turns off the layer HUD
	LayerHud_Info                  ///< Shows info about a specific layer
)

/// Debug HUD is provided to help developers gauge and debug the fidelity of their app's
/// stereo rendering characteristics. Using the provided quad and crosshair guides,
/// the developer can verify various aspects such as VR tracking units (e.g. meters),
/// stereo camera-parallax properties (e.g. making sure objects at infinity are rendered
/// with the proper separation), measuring VR geometry sizes and distances and more.
///
///     App can toggle the debug HUD modes as such:
///     \code{.cpp}
///         ovrDebugHudStereoMode DebugHudMode = ovrDebugHudStereo_QuadWithCrosshair;
///         ovr_SetInt(Hmd, OVR_DEBUG_HUD_STEREO_MODE, (int)DebugHudMode);
///     \endcode
///
/// The app can modify the visual properties of the stereo guide (i.e. quad, crosshair)
/// using the ovr_SetFloatArray function. For a list of tweakable properties,
/// see the OVR_DEBUG_HUD_STEREO_GUIDE_* keys in the OVR_CAPI_Keys.h header file.
type DebugHudStereoMode int32

const (
	DebugHudStereo_Off                 DebugHudStereoMode = 0 ///< Turns off the Stereo Debug HUD
	DebugHudStereo_Quad                                       ///< Renders Quad in world for Stereo Debugging
	DebugHudStereo_QuadWithCrosshair                          ///< Renders Quad+crosshair in world for Stereo Debugging
	DebugHudStereo_CrosshairAtInfinity                        ///< Renders screen-space crosshair at infinity for Stereo Debugging
	DebugHudStereo_Count                                      ///< \internal Count of enumerated elements
)

// -----------------------------------------------------------------------------------
/// @name Property Access
///
/// These functions read and write OVR properties. Supported properties
/// are defined in OVR_CAPI_Keys.h
///
//@{

/// Reads a boolean property.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid for only the call.
/// \param[in] defaultVal specifes the value to return if the property couldn't be read.
/// \return Returns the property interpreted as a boolean value. Returns defaultVal if
///         the property doesn't exist.
func (s *Session) GetBool(propertyName string, defaultVal bool) bool {
	return false
}

/// Writes or creates a boolean property.
/// If the property wasn't previously a boolean property, it is changed to a boolean property.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid only for the call.
/// \param[in] value The value to write.
/// \return Returns true if successful, otherwise false. A false result should only occur if the property
///         name is empty or if the property is read-only.
func (s *Session) SetBool(propertyName string, value bool) error {
	return errNoImpl
}

/// Reads an integer property.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid only for the call.
/// \param[in] defaultVal Specifes the value to return if the property couldn't be read.
/// \return Returns the property interpreted as an integer value. Returns defaultVal if
///         the property doesn't exist.
func (s *Session) GetInt(propertyName string, defaultVal int) int {
	return 0
}

/// Writes or creates an integer property.
///
/// If the property wasn't previously a boolean property, it is changed to an integer property.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid only for the call.
/// \param[in] value The value to write.
/// \return Returns true if successful, otherwise false. A false result should only occur if the property
///         name is empty or if the property is read-only.
func (s *Session) SetInt(propertyName string, value int) error {
	return errNoImpl
}

/// Reads a float property.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid only for the call.
/// \param[in] defaultVal specifes the value to return if the property couldn't be read.
/// \return Returns the property interpreted as an float value. Returns defaultVal if
///         the property doesn't exist.
func (s *Session) GetFloat(propertyName string, defaultVal float32) float32 {
	return 0
}

/// Writes or creates a float property.
/// If the property wasn't previously a float property, it's changed to a float property.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid only for the call.
/// \param[in] value The value to write.
/// \return Returns true if successful, otherwise false. A false result should only occur if the property
///         name is empty or if the property is read-only.
func (s *Session) SetFloat(propertyName string, value float32) error {
	return errNoImpl
}

/// Reads a float array property.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid only for the call.
/// \param[in] values An array of float to write to.
/// \param[in] valuesCapacity Specifies the maximum number of elements to write to the values array.
/// \return Returns the number of elements read, or 0 if property doesn't exist or is empty.
func (s *Session) GetFloatArray(propertyName string, valuesCapacity uint) []float32 {
	return nil
}

/// Writes or creates a float array property.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid only for the call.
/// \param[in] values An array of float to write from.
/// \param[in] valuesSize Specifies the number of elements to write.
/// \return Returns true if successful, otherwise false. A false result should only occur if the property
///         name is empty or if the property is read-only.
func (s *Session) SetFloatArray(propertyName string, values []float32) error {
	return errNoImpl
}

/// Reads a string property.
/// Strings are UTF8-encoded and null-terminated.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid only for the call.
/// \param[in] defaultVal Specifes the value to return if the property couldn't be read.
/// \return Returns the string property if it exists. Otherwise returns defaultVal, which can be specified as NULL.
///         The return memory is guaranteed to be valid until next call to ovr_GetString or
///         until the HMD is destroyed, whichever occurs first.
func (s *Session) GetString(propertyName string, defaultVal string) string {
	return ""
}

/// Writes or creates a string property.
/// Strings are UTF8-encoded and null-terminated.
///
/// \param[in] session Specifies an ovrSession previously returned by ovr_Create.
/// \param[in] propertyName The name of the property, which needs to be valid only for the call.
/// \param[in] value The string property, which only needs to be valid for the duration of the call.
/// \return Returns true if successful, otherwise false. A false result should only occur if the property
///         name is empty or if the property is read-only.
func (s *Session) SetString(propertyName string, value string) error {
	return errNoImpl
}
