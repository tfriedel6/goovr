package goovr

/*
#cgo CXXFLAGS: -std=c++11

#include <stdlib.h>
#include "OVR_CAPI.h"

int logCallback_cgo(int level, const char* message);
*/
import "C"
import (
	"errors"
	"fmt"
	"unsafe"
)

//export logCallback
func logCallback(level C.int, message *C.char) {
	fmt.Printf("OVR(%d) %s\n", level, C.GoString(message))
}

func ovrBool(b bool) C.ovrBool {
	if b {
		return C.ovrTrue
	}
	return C.ovrFalse
}

func goBool(b C.ovrBool) bool { return b == C.ovrTrue }

// A 2D vector with integer components.
type Vector2i struct {
	X, Y int
}

func goVector2i(v C.ovrVector2i) Vector2i { return Vector2i{int(v.x), int(v.y)} }
func cVector2i(v Vector2i) C.ovrVector2i  { return C.ovrVector2i{C.int(v.X), C.int(v.Y)} }

// A 2D size with integer components.
type Sizei struct {
	W, H int
}

func goSizei(v C.ovrSizei) Sizei { return Sizei{int(v.w), int(v.h)} }
func cSizei(v Sizei) C.ovrSizei  { return C.ovrSizei{C.int(v.W), C.int(v.H)} }

// A 2D rectangle with a position and size.
// All components are integers.
type Recti struct {
	Pos  Vector2i
	Size Sizei
}

func goRecti(v C.ovrRecti) Recti { return Recti{goVector2i(v.Pos), goSizei(v.Size)} }
func cRecti(v Recti) C.ovrRecti  { return C.ovrRecti{cVector2i(v.Pos), cSizei(v.Size)} }

// A quaternion rotation.
type Quatf struct {
	X, Y, Z, W float32
}

func goQuatf(v C.ovrQuatf) Quatf { return Quatf{float32(v.x), float32(v.y), float32(v.z), float32(v.w)} }
func cQuatf(v Quatf) C.ovrQuatf {
	return C.ovrQuatf{C.float(v.X), C.float(v.Y), C.float(v.Z), C.float(v.W)}
}

// A 2D vector with float components.
type Vector2f struct {
	X, Y float32
}

func goVector2f(v C.ovrVector2f) Vector2f { return Vector2f{float32(v.x), float32(v.y)} }
func cVector2f(v Vector2f) C.ovrVector2f  { return C.ovrVector2f{C.float(v.X), C.float(v.Y)} }

// A 3D vector with float components.
type Vector3f struct {
	X, Y, Z float32
}

func goVector3f(v C.ovrVector3f) Vector3f { return Vector3f{float32(v.x), float32(v.y), float32(v.z)} }
func cVector3f(v Vector3f) C.ovrVector3f {
	return C.ovrVector3f{C.float(v.X), C.float(v.Y), C.float(v.Z)}
}

// A 4x4 matrix with float elements.
type Matrix4f struct {
	M [4][4]float32
}

func goMatrix4f(v C.ovrMatrix4f) Matrix4f {
	return Matrix4f{M: [4][4]float32{
		{float32(v.M[0][0]), float32(v.M[0][1]), float32(v.M[0][2]), float32(v.M[0][3])},
		{float32(v.M[1][0]), float32(v.M[1][1]), float32(v.M[1][2]), float32(v.M[1][3])},
		{float32(v.M[2][0]), float32(v.M[2][1]), float32(v.M[2][2]), float32(v.M[2][3])},
		{float32(v.M[3][0]), float32(v.M[3][1]), float32(v.M[3][2]), float32(v.M[3][3])}}}
}

func cMatrix4f(v Matrix4f) C.ovrMatrix4f {
	return C.ovrMatrix4f{M: [4][4]C.float{
		{C.float(v.M[0][0]), C.float(v.M[0][1]), C.float(v.M[0][2]), C.float(v.M[0][3])},
		{C.float(v.M[1][0]), C.float(v.M[1][1]), C.float(v.M[1][2]), C.float(v.M[1][3])},
		{C.float(v.M[2][0]), C.float(v.M[2][1]), C.float(v.M[2][2]), C.float(v.M[2][3])},
		{C.float(v.M[3][0]), C.float(v.M[3][1]), C.float(v.M[3][2]), C.float(v.M[3][3])}}}
}

// Position and orientation together.
type Posef struct {
	Orientation Quatf
	Position    Vector3f
}

func goPosef(v C.ovrPosef) Posef {
	return Posef{Orientation: goQuatf(v.Orientation), Position: goVector3f(v.Position)}
}
func cPosef(v Posef) C.ovrPosef {
	return C.ovrPosef{Orientation: cQuatf(v.Orientation), Position: cVector3f(v.Position)}
}

// A full pose (rigid body) configuration with first and second derivatives.
//
// Body refers to any object for which ovrPoseStatef is providing data.
// It can be the camera or something else; the context depends on the usage of the struct.
type PoseStatef struct {
	ThePose             Posef
	AngularVelocity     Vector3f
	LinearVelocity      Vector3f
	AngularAcceleration Vector3f
	LinearAcceleration  Vector3f
	TimeInSeconds       float64
}

// Describes the up, down, left, and right angles of the field of view.
//
// Field Of View (FOV) tangent of the angle units.
// \note For a standard 90 degree vertical FOV, we would
// have: { UpTan = tan(90 degrees / 2), DownTan = tan(90 degrees / 2) }.
type FovPort struct {
	UpTan    float32
	DownTan  float32
	LeftTan  float32
	RightTan float32
}

func goFovPort(v C.ovrFovPort) FovPort {
	return FovPort{float32(v.UpTan), float32(v.DownTan), float32(v.LeftTan), float32(v.RightTan)}
}
func cFovPort(v FovPort) C.ovrFovPort {
	return C.ovrFovPort{C.float(v.UpTan), C.float(v.DownTan), C.float(v.LeftTan), C.float(v.RightTan)}
}

// Enumerates all HMD types that we support.
//
// The currently released developer kits are ovrHmd_DK1 and ovrHmd_DK2. The other enumerations are for internal use only.
type HmdType int32

const (
	Hmd_None      HmdType = C.ovrHmd_None
	Hmd_DK1               = C.ovrHmd_DK1
	Hmd_DKHD              = C.ovrHmd_DKHD
	Hmd_DK2               = C.ovrHmd_DK2
	Hmd_BlackStar         = C.ovrHmd_BlackStar
	Hmd_CB                = C.ovrHmd_CB
	Hmd_Other             = C.ovrHmd_Other
)

// HMD capability bits reported by device.
//
// Set <B>(read/write)</B> flags through ovrHmd_SetEnabledCaps()
type HmdCaps int32

const (
	HmdCap_DebugDevice       HmdCaps = C.ovrHmdCap_DebugDevice
	HmdCap_LowPersistence            = C.ovrHmdCap_LowPersistence
	HmdCap_DynamicPrediction         = C.ovrHmdCap_DynamicPrediction
	HmdCap_NoVSync                   = C.ovrHmdCap_NoVSync
	HmdCap_Writable_Mask             = C.ovrHmdCap_Writable_Mask
	HmdCap_Service_Mask              = C.ovrHmdCap_Service_Mask
)

// Tracking capability bits reported by the device.
// Used with ovrHmd_ConfigureTracking.
type TrackingCaps int32

const (
	TrackingCap_Orientation      TrackingCaps = C.ovrTrackingCap_Orientation
	TrackingCap_MagYawCorrection              = C.ovrTrackingCap_MagYawCorrection
	TrackingCap_Position                      = C.ovrTrackingCap_Position
	TrackingCap_Idle                          = C.ovrTrackingCap_Idle
)

// Specifies which eye is being used for rendering.
// This type explicitly does not include a third "NoStereo" monoscopic option, as such is
// not required for an HMD-centered API.
type EyeType int32

const (
	Eye_Left  EyeType = C.ovrEye_Left
	Eye_Right         = C.ovrEye_Right
	Eye_Count         = C.ovrEye_Count
)

// This is a complete descriptor of the HMD.
type Hmd struct {
	cHmd                       C.ovrHmd
	Handle                     unsafe.Pointer
	Type                       HmdType
	ProductName                string
	Manufacturer               string
	VendorId                   int16
	ProductId                  int16
	SerialNumber               [24]byte
	FirmwareMajor              int16
	FirmwareMinor              int16
	CameraFrustumHFovInRadians float32
	CameraFrustumVFovInRadians float32
	CameraFrustumNearZInMeters float32
	CameraFrustumFarZInMeters  float32
	HmdCaps                    HmdCaps
	TrackingCaps               TrackingCaps
	DefaultEyeFov              [Eye_Count]FovPort
	MaxEyeFov                  [Eye_Count]FovPort
	EyeRenderOrder             [Eye_Count]EyeType
	Resolution                 Sizei
}

func goHmd(cHmd C.ovrHmd) *Hmd {
	var serialNumber [24]byte
	for i := 0; i < 24; i++ {
		serialNumber[i] = byte(cHmd.SerialNumber[i])
	}
	var defaultEyeFov, maxEyeFov [Eye_Count]FovPort
	var eyeRenderOrder [Eye_Count]EyeType
	for i := 0; i < Eye_Count; i++ {
		defaultEyeFov[i] = FovPort{
			UpTan:    float32(cHmd.DefaultEyeFov[i].UpTan),
			DownTan:  float32(cHmd.DefaultEyeFov[i].DownTan),
			LeftTan:  float32(cHmd.DefaultEyeFov[i].LeftTan),
			RightTan: float32(cHmd.DefaultEyeFov[i].RightTan)}
		maxEyeFov[i] = FovPort{
			UpTan:    float32(cHmd.MaxEyeFov[i].UpTan),
			DownTan:  float32(cHmd.MaxEyeFov[i].DownTan),
			LeftTan:  float32(cHmd.MaxEyeFov[i].LeftTan),
			RightTan: float32(cHmd.MaxEyeFov[i].RightTan)}
		eyeRenderOrder[i] = EyeType(cHmd.EyeRenderOrder[i])
	}

	return &Hmd{
		cHmd:                       cHmd,
		Handle:                     unsafe.Pointer(cHmd.Handle),
		Type:                       HmdType(cHmd.Type),
		ProductName:                C.GoString(cHmd.ProductName),
		Manufacturer:               C.GoString(cHmd.Manufacturer),
		VendorId:                   int16(cHmd.VendorId),
		ProductId:                  int16(cHmd.ProductId),
		SerialNumber:               serialNumber,
		FirmwareMajor:              int16(cHmd.FirmwareMajor),
		FirmwareMinor:              int16(cHmd.FirmwareMinor),
		CameraFrustumHFovInRadians: float32(cHmd.CameraFrustumHFovInRadians),
		CameraFrustumVFovInRadians: float32(cHmd.CameraFrustumVFovInRadians),
		CameraFrustumNearZInMeters: float32(cHmd.CameraFrustumNearZInMeters),
		CameraFrustumFarZInMeters:  float32(cHmd.CameraFrustumFarZInMeters),
		HmdCaps:                    HmdCaps(cHmd.HmdCaps),
		TrackingCaps:               TrackingCaps(cHmd.TrackingCaps),
		DefaultEyeFov:              defaultEyeFov,
		MaxEyeFov:                  maxEyeFov,
		EyeRenderOrder:             eyeRenderOrder,
		Resolution:                 goSizei(cHmd.Resolution)}
}

// Bit flags describing the current status of sensor tracking.
//  The values must be the same as in enum StatusBits
//
// \see ovrTrackingState
type StatusBits int32

const (
	Status_OrientationTracked StatusBits = C.ovrStatus_OrientationTracked
	Status_PositionTracked               = C.ovrStatus_PositionTracked
	Status_CameraPoseTracked             = C.ovrStatus_CameraPoseTracked
	Status_PositionConnected             = C.ovrStatus_PositionConnected
	Status_HmdConnected                  = C.ovrStatus_HmdConnected
	Status_EnumSize                      = C.ovrStatus_EnumSize
)

// Specifies a reading we can query from the sensor.
//
// \see ovrTrackingState
type SensorData struct {
	Accelerometer Vector3f
	Gyro          Vector3f
	Magnetometer  Vector3f
	Temperature   float32
	TimeInSeconds float32
}

// Tracking state at a given absolute time (describes predicted HMD pose, etc.).
// Returned by ovrHmd_GetTrackingState.
//
// \see ovrHmd_GetTrackingState
type TrackingState struct {
	HeadPose               PoseStatef
	CameraPose             Posef
	LeveledCameraPose      Posef
	RawSensorData          SensorData
	StatusFlags            StatusBits
	LastCameraFrameCounter uint32
}

func goTrackingState(v C.ovrTrackingState) TrackingState {
	return TrackingState{
		HeadPose: PoseStatef{
			ThePose:             goPosef(v.HeadPose.ThePose),
			AngularVelocity:     goVector3f(v.HeadPose.AngularVelocity),
			LinearVelocity:      goVector3f(v.HeadPose.LinearVelocity),
			AngularAcceleration: goVector3f(v.HeadPose.AngularAcceleration),
			LinearAcceleration:  goVector3f(v.HeadPose.LinearAcceleration),
			TimeInSeconds:       float64(v.HeadPose.TimeInSeconds)},
		CameraPose:        goPosef(v.CameraPose),
		LeveledCameraPose: goPosef(v.LeveledCameraPose),
		RawSensorData: SensorData{
			Accelerometer: goVector3f(v.RawSensorData.Accelerometer),
			Gyro:          goVector3f(v.RawSensorData.Gyro),
			Magnetometer:  goVector3f(v.RawSensorData.Magnetometer),
			Temperature:   float32(v.RawSensorData.Temperature),
			TimeInSeconds: float32(v.RawSensorData.TimeInSeconds)},
		StatusFlags:            StatusBits(v.StatusFlags),
		LastCameraFrameCounter: uint32(v.LastCameraFrameCounter)}
}

// Frame timing data reported by ovrHmd_GetFrameTiming.
//
// \see ovrHmd_GetFrameTiming
type FrameTiming struct {
	DisplayMidpointSeconds float64
	FrameIntervalSeconds   float64
	AppFrameIndex          float64
	DisplayFrameIndex      float64
}

// Rendering information for each eye. Computed by ovrHmd_GetRenderDesc() based on the
// specified FOV. Note that the rendering viewport is not included
// here as it can be specified separately and modified per frame by
// passing different Viewport values in the layer structure.
//
// \see ovrHmd_GetRenderDesc
type EyeRenderDesc struct {
	Eye                       EyeType
	Fov                       FovPort
	DistortedViewport         Recti
	PixelsPerTanAngleAtCenter Vector2f
	HmdToEyeViewOffset        Vector3f
}

// Projection information for ovrLayerEyeFovDepth.
//
// Use the utility function ovrTimewarpProjectionDesc_FromProjection to
// generate this structure from the application's projection matrix.
//
// \see ovrLayerEyeFovDepth, ovrTimewarpProjectionDesc_FromProjection
type TimewarpProjectionDesc struct {
	Projection22 float32
	Projection23 float32
	Projection32 float32
}

// Contains the data necessary to properly calculate position info for various layer types.
// - HmdToEyeViewOffset is the same value pair provided in ovrEyeRenderDesc.
// - HmdSpaceToWorldScaleInMeters is used to scale player motion into in-application units.
//   In other words, it is how big an in-application unit is in the player's physical meters.
//   For example, if the application uses inches as its units then HmdSpaceToWorldScaleInMeters would be 0.0254.
//   Note that if you are scaling the player in size, this must also scale. So if your application
//   units are inches, but you're shrinking the player to half their normal size, then
//   HmdSpaceToWorldScaleInMeters would be 0.0254*2.0.
//
// \see ovrEyeRenderDesc, ovrHmd_SubmitFrame
type ViewScaleDesc struct {
	HmdToEyeViewOffset           [Eye_Count]Vector3f
	HmdSpaceToWorldScaleInMeters float32
}

// These types are used to hide platform-specific details when passing
// render device, OS, and texture data to the API.
//
// The benefit of having these wrappers versus platform-specific API functions is
// that they allow application glue code to be portable. A typical example is an
// engine that has multiple back ends, such as GL and D3D. Portable code that calls
// these back ends can also use LibOVR. To do this, back ends can be modified
// to return portable types such as ovrTexture and ovrRenderAPIConfig.
type RenderAPIType int32

const (
	RenderAPI_None           RenderAPIType = C.ovrRenderAPI_None
	RenderAPI_OpenGL                       = C.ovrRenderAPI_OpenGL
	RenderAPI_Android_GLES                 = C.ovrRenderAPI_Android_GLES
	RenderAPI_D3D9_Obsolete                = C.ovrRenderAPI_D3D9_Obsolete
	RenderAPI_D3D10_Obsolete               = C.ovrRenderAPI_D3D10_Obsolete
	RenderAPI_D3D11                        = C.ovrRenderAPI_D3D11
	RenderAPI_Count                        = C.ovrRenderAPI_Count
)

// API-independent part of a texture descriptor.
//
// ovrTextureHeader is a common struct present in all ovrTexture struct types.
type TextureHeader struct {
	API         RenderAPIType
	TextureSize Sizei
}

// Contains platform-specific information about a texture.
// Aliases to one of ovrD3D11Texture or ovrGLTexture.
//
// \see ovrD3D11Texture, ovrGLTexture.
type Texture struct {
	Header   TextureHeader
	OGL      *GLTextureData
	D3D11    *D3D11TextureData
	cTexture *C.ovrTexture
}

// Describes a set of textures that act as a rendered flip chain.
//
// An ovrSwapTextureSet per layer is passed to ovrHmd_SubmitFrame via one of the ovrLayer types.
// The TextureCount refers to the flip chain count and not an eye count.
// See the layer structs and functions for information about how to use ovrSwapTextureSet.
//
// ovrSwapTextureSets must be created by either the ovrHmd_CreateSwapTextureSetD3D11 or
// ovrHmd_CreateSwapTextureSetGL factory function, and must be destroyed by ovrHmd_DestroySwapTextureSet.
//
// \see ovrHmd_CreateSwapTextureSetD3D11, ovrHmd_CreateSwapTextureSetGL, ovrHmd_DestroySwapTextureSet.
type SwapTextureSet struct {
	cSwapTextureSet *C.ovrSwapTextureSet
	Textures        []Texture
	TextureCount    int
}

// The current index needs to be set on the C struct, so it is not exposed in the Go struct.
func (s *SwapTextureSet) SetCurrentIndex(value int) {
	s.cSwapTextureSet.CurrentIndex = C.int(value)
}

// The current index needs to be set on the C struct, so it is not exposed in the Go struct.
func (s *SwapTextureSet) CurrentIndex() int {
	return int(s.cSwapTextureSet.CurrentIndex)
}

// Initialization flags.
//
// \see ovrInitParams, ovr_Initialize
type InitFlags int32

const (
	Init_Debug          InitFlags = C.ovrInit_Debug
	Init_ServerOptional           = C.ovrInit_ServerOptional
	Init_RequestVersion           = C.ovrInit_RequestVersion
	Init_ForceNoDebug             = C.ovrInit_ForceNoDebug
)

// Logging levels
//
// \see ovrInitParams, ovrLogCallback
type LogLevel int32

const (
	LogLevel_Debug LogLevel = C.ovrLogLevel_Debug
	LogLevel_Info           = C.ovrLogLevel_Info
	LogLevel_Error          = C.ovrLogLevel_Error
)

// Signature of the logging callback function pointer type.
//
// \param[in] level is one of the ovrLogLevel constants.
// \param[in] message is a UTF8-encoded null-terminated string.
// \see ovrInitParams, ovrLogLevel, ovr_Initialize
type LogCallback func(level int, message string)

// Parameters for ovr_Initialize.
//
// \see ovr_Initialize
type InitParams struct {
	Flags                 InitFlags
	RequestedMinorVersion uint32
	LogCallback           LogCallback
	ConnectionTimeoutMS   uint32
}

// Initializes LibOVR
//
// Initialize LibOVR for application usage. This includes finding and loading the LibOVRRT
// shared library. No LibOVR API functions, other than ovr_GetLastErrorInfo, can be called
// unless ovr_Initialize succeeds. A successful call to ovr_Initialize must be eventually
// followed by a call to ovr_Shutdown. ovr_Initialize calls are idempotent.
// Calling ovr_Initialize twice does not require two matching calls to ovr_Shutdown.
// If already initialized, the return value is ovr_Success.
//
// LibOVRRT shared library search order:
//      -# Current working directory (often the same as the application directory).
//      -# Module directory (usually the same as the application directory,
//         but not if the module is a separate shared library).
//      -# Application directory
//      -# Development directory (only if OVR_ENABLE_DEVELOPER_SEARCH is enabled,
//         which is off by default).
//      -# Standard OS shared library search location(s) (OS-specific).
//
// \param params Specifies custom initialization options. May be NULL to indicate default options.
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information. Example failed results include:
//     - ovrError_Initialize: Generic initialization error.
//     - ovrError_LibLoad: Couldn't load LibOVRRT.
//     - ovrError_LibVersion: LibOVRRT version incompatibility.
//     - ovrError_ServiceConnection: Couldn't connect to the OVR Service.
//     - ovrError_ServiceVersion: OVR Service version incompatibility.
//     - ovrError_IncompatibleOS: The operating system version is incompatible.
//     - ovrError_DisplayInit: Unable to initialize the HMD display.
//     - ovrError_ServerStart:  Unable to start the server. Is it already running?
//     - ovrError_Reinitialization: Attempted to re-initialize with a different version.
//
// \see ovr_Shutdown
//
func Initialize(params *InitParams) error {
	var cResult C.ovrResult
	if params == nil {
		cResult = C.ovr_Initialize(nil)
	} else {
		var cParams C.ovrInitParams
		cParams.Flags = C.uint32_t(params.Flags)
		cParams.RequestedMinorVersion = C.uint32_t(params.RequestedMinorVersion)
		cParams.LogCallback = C.ovrLogCallback(C.logCallback_cgo)
		cParams.ConnectionTimeoutMS = C.uint32_t(params.ConnectionTimeoutMS)
		cResult = C.ovr_Initialize(&cParams)
	}
	return errorForResult(cResult)
}

// Shuts down LibOVR
//
// A successful call to ovr_Initialize must be eventually matched by a call to ovr_Shutdown.
// After calling ovr_Shutdown, no LibOVR functions can be called except ovr_GetLastErrorInfo
// or another ovr_Initialize. ovr_Shutdown invalidates all pointers, references, and created objects
// previously returned by LibOVR functions. The LibOVRRT shared library can be unloaded by
// ovr_Shutdown.
//
// \see ovr_Initialize
func Shutdown() {
	C.ovr_Shutdown()
}

// Provides information about the last error.
// \see ovr_GetLastErrorInfo
type ErrorInfo struct {
	Result      uint32
	ErrorString string
}

// Returns information about the most recent failed return value by the
// current thread for this library.
//
// This function itself can never generate an error.
// The last error is never cleared by LibOVR, but will be overwritten by new errors.
// Do not use this call to determine if there was an error in the last API
// call as successful API calls don't clear the last ovrErrorInfo.
// To avoid any inconsistency, ovr_GetLastErrorInfo should be called immediately
// after an API function that returned a failed ovrResult, with no other API
// functions called in the interim.
//
// \param[out] errorInfo The last ovrErrorInfo for the current thread.
//
// \see ovrErrorInfo
func GetLastErrorInfo() ErrorInfo {
	var errorInfo C.ovrErrorInfo
	C.ovr_GetLastErrorInfo(&errorInfo)
	return ErrorInfo{Result: uint32(errorInfo.Result), ErrorString: C.GoString((*C.char)(&errorInfo.ErrorString[0]))}
}

// Returns the version string representing the LibOVRRT version.
//
// The returned string pointer is valid until the next call to ovr_Shutdown.
//
// Note that the returned version string doesn't necessarily match the current
// OVR_MAJOR_VERSION, etc., as the returned string refers to the LibOVRRT shared
// library version and not the locally compiled interface version.
//
// The format of this string is subject to change in future versions and its contents
// should not be interpreted.
//
// \return Returns a UTF8-encoded null-terminated version string.
func GetVersionString() string {
	return C.GoString(C.ovr_GetVersionString())
}

// Writes a message string to the LibOVR tracing mechanism (if enabled).
//
// This message will be passed back to the application via the ovrLogCallback if
// it was registered.
//
// \param[in] level One of the ovrLogLevel constants.
// \param[in] message A UTF8-encoded null-terminated string.
// \return returns the strlen of the message or a negative value if the message is too large.
//
// \see ovrLogLevel, ovrLogCallback
func TraceMessage(level int, message string) (int, error) {
	cMessage := C.CString(message)
	defer C.free(unsafe.Pointer(cMessage))
	result := C.ovr_TraceMessage(C.int(level), cMessage)
	if result < 0 {
		return int(result), errors.New("Message too large")
	}
	return int(result), nil
}

// Detects or re-detects HMDs and reports the total number detected.
//
// This function is useful to determine if an HMD can be created without committing to
// creating it. For example, an application can use this information to present an HMD selection GUI.
//
// If one or more HMDs are present, an integer value is returned which indicates
// the number present. The number present indicates the range of valid indexes that
// can be passed to ovrHmd_Create. If no HMDs are present, the return
// value is zero. If there is an error, a negative error ovrResult value is
// returned.
//
// \return Returns an integer that specifies the number of HMDs currently present. Upon failure, OVR_SUCCESS(result) is false.
//
// \see ovrHmd_Create
func Hmd_Detect() (int, error) {
	result := C.ovrHmd_Detect()
	err := errorForResult(result)
	if err != nil {
		return 0, err
	}
	return int(result), nil
}

// Creates a handle to an HMD which doubles as a description structure.
//
// Upon success the returned ovrHmd* must be freed with ovrHmd_Destroy.
// A second call to ovrHmd_Create with the same index as a previously
// successful call will result in an error return value.
//
// \param[in] index A value in the range of [0 .. ovrHmd_Detect()-1].
// \param[out] pHmd Provides a pointer to an ovrHmd which will be written to upon success.
// \return Returns an ovrResult indicating success or failure.
//
// \see ovrHmd_Destroy
func Hmd_Create(index int) (*Hmd, error) {
	var cHmd C.ovrHmd
	result := C.ovrHmd_Create(C.int(index), &cHmd)
	err := errorForResult(result)
	if err != nil {
		return nil, err
	}
	return goHmd(cHmd), nil
}

// Creates a fake HMD used for debugging only.
//
// This is not tied to specific hardware, but may be used to debug some of the related rendering.
// \param[in] type Indicates the HMD type to emulate.
// \param[out] pHmd Provides a pointer to an ovrHmd which will be written to upon success.
// \return Returns an ovrResult indicating success or failure.
//
// \see ovrHmd_Create
func Hmd_CreateDebug(typ HmdType) (*Hmd, error) {
	var cHmd C.ovrHmd
	result := C.ovrHmd_CreateDebug(C.ovrHmdType(typ), &cHmd)
	err := errorForResult(result)
	if err != nil {
		return nil, err
	}
	return goHmd(cHmd), nil
}

// Destroys the HMD.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \see ovrHmd_Create
func (hmd *Hmd) Destroy() {
	C.ovrHmd_Destroy(hmd.cHmd)
}

// Returns ovrHmdCaps bits that are currently enabled.
//
// Note that this value is different from ovrHmdDesc::HmdCaps, which describes what
// capabilities are available for that HMD.
//
// \return Returns a combination of zero or more ovrHmdCaps.
// \see ovrHmdCaps
func (hmd *Hmd) GetEnabledCaps() HmdCaps {
	return HmdCaps(C.ovrHmd_GetEnabledCaps(hmd.cHmd))
}

// Modifies capability bits described by ovrHmdCaps that can be modified,
// such as ovrHmdCap_LowPersistance.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] hmdCaps A combination of 0 or more ovrHmdCaps.
//
// \see ovrHmdCaps
func (hmd *Hmd) SetEnabledCaps(hmdCaps HmdCaps) {
	C.ovrHmd_SetEnabledCaps(hmd.cHmd, C.uint(hmdCaps))
}

// Starts sensor sampling, enabling specified capabilities, described by ovrTrackingCaps.
//
// Use 0 for both supportedTrackingCaps and requiredTrackingCaps to disable tracking.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
//
// \param[in] supportedTrackingCaps Specifies support that is requested. The function will succeed
//            even if these caps are not available (i.e. sensor or camera is unplugged). Support
//            will automatically be enabled if the device is plugged in later. Software should
//            check ovrTrackingState.StatusFlags for real-time status.
//
// \param[in] requiredTrackingCaps Specifies sensor capabilities required at the time of the call.
//            If they are not available, the function will fail. Pass 0 if only specifying
//            supportedTrackingCaps.
//
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information.
//
// \see ovrTrackingCaps
//
func (hmd *Hmd) ConfigureTracking(supportedTrackingCaps, requiredTrackingCaps TrackingCaps) error {
	result := C.ovrHmd_ConfigureTracking(hmd.cHmd, C.uint(supportedTrackingCaps), C.uint(requiredTrackingCaps))
	return errorForResult(result)
}

// Re-centers the sensor position and orientation.
//
// This resets the (x,y,z) positional components and the yaw orientation component.
// The Roll and pitch orientation components are always determined by gravity and cannot
// be redefined. All future tracking will report values relative to this new reference position.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
func (hmd *Hmd) RecenterPose() {
	C.ovrHmd_RecenterPose(hmd.cHmd)
}

// Returns tracking state reading based on the specified absolute system time.
//
// Pass an absTime value of 0.0 to request the most recent sensor reading. In this case
// both PredictedPose and SamplePose will have the same value.
//
// This may also be used for more refined timing of front buffer rendering logic, and so on.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] absTime Specifies the absolute future time to predict the return
//            ovrTrackingState value. Use 0 to request the most recent tracking state.
// \return Returns the ovrTrackingState that is predicted for the given absTime.
//
// \see ovrTrackingState, ovrHmd_GetEyePoses, ovr_GetTimeInSeconds
func (hmd *Hmd) GetTrackingState(absTime float64) TrackingState {
	result := C.ovrHmd_GetTrackingState(hmd.cHmd, C.double(absTime))
	return goTrackingState(result)
}

// Describes layer types that can be passed to ovrHmd_SubmitFrame.
// Each layer type has an associated struct, such as ovrLayerEyeFov.
//
// \see ovrLayerHeader
type LayerType int32

const (
	LayerType_Disabled       LayerType = C.ovrLayerType_Disabled
	LayerType_EyeFov                   = C.ovrLayerType_EyeFov
	LayerType_EyeFovDepth              = C.ovrLayerType_EyeFovDepth
	LayerType_QuadInWorld              = C.ovrLayerType_QuadInWorld
	LayerType_QuadHeadLocked           = C.ovrLayerType_QuadHeadLocked
	LayerType_Direct                   = C.ovrLayerType_Direct
)

// Identifies flags used by ovrLayerHeader and which are passed to ovrHmd_SubmitFrame.
//
// \see ovrLayerHeader
type LayerFlags int32

const (
	LayerFlag_HighQuality               LayerFlags = C.ovrLayerFlag_HighQuality
	LayerFlag_TextureOriginAtBottomLeft            = C.ovrLayerFlag_TextureOriginAtBottomLeft
)

// Defines properties shared by all ovrLayer structs, such as ovrLayerEyeFov.
//
// ovrLayerHeader is used as a base member in these larger structs.
// This struct cannot be used by itself except for the case that Type is ovrLayerType_Disabled.
//
// \see ovrLayerType, ovrLayerFlags
type LayerHeader struct {
	Type  LayerType
	Flags LayerFlags
}

type LayerInterface interface {
	ptr() *C.ovrLayerHeader
}

// Describes a layer that specifies a monoscopic or stereoscopic view.
// This is the kind of layer that's typically used as layer 0 to ovrHmd_SubmitFrame,
// as it is the kind of layer used to render a 3D stereoscopic view.
//
// Three options exist with respect to mono/stereo texture usage:
//    - ColorTexture[0] and ColorTexture[1] contain the left and right stereo renderings, respectively.
//      Viewport[0] and Viewport[1] refer to ColorTexture[0] and ColorTexture[1], respectively.
//    - ColorTexture[0] contains both the left and right renderings, ColorTexture[1] is NULL,
//      and Viewport[0] and Viewport[1] refer to sub-rects with ColorTexture[0].
//    - ColorTexture[0] contains a single monoscopic rendering, and Viewport[0] and
//      Viewport[1] both refer to that rendering.
//
// \see ovrSwapTextureSet, ovrHmd_SubmitFrame
type LayerEyeFov struct {
	Header       LayerHeader
	ColorTexture [Eye_Count]*SwapTextureSet
	Viewport     [Eye_Count]Recti
	Fov          [Eye_Count]FovPort
	RenderPose   [Eye_Count]Posef
	cStruct      C.ovrLayerEyeFov
}

func (l *LayerEyeFov) ptr() *C.ovrLayerHeader {
	l.cStruct.Header.Type = C.ovrLayerType(l.Header.Type)
	l.cStruct.Header.Flags = C.uint(l.Header.Flags)
	for i := 0; i < Eye_Count; i++ {
		l.cStruct.ColorTexture[i] = l.ColorTexture[i].cSwapTextureSet
		l.cStruct.Viewport[i] = cRecti(l.Viewport[i])
		l.cStruct.Fov[i] = cFovPort(l.Fov[i])
		l.cStruct.RenderPose[i] = cPosef(l.RenderPose[i])
	}
	return (*C.ovrLayerHeader)(unsafe.Pointer(&l.cStruct))
}

// Describes a layer that specifies a monoscopic or stereoscopic view,
// with depth textures in addition to color textures. This is typically used to support
// positional time warp. This struct is the same as ovrLayerEyeFov, but with the addition
// of DepthTexture and ProjectionDesc.
//
// ProjectionDesc can be created using ovrTimewarpProjectionDesc_FromProjection.
//
// Three options exist with respect to mono/stereo texture usage:
//    - ColorTexture[0] and ColorTexture[1] contain the left and right stereo renderings, respectively.
//      Viewport[0] and Viewport[1] refer to ColorTexture[0] and ColorTexture[1], respectively.
//    - ColorTexture[0] contains both the left and right renderings, ColorTexture[1] is NULL,
//      and Viewport[0] and Viewport[1] refer to sub-rects with ColorTexture[0].
//    - ColorTexture[0] contains a single monoscopic rendering, and Viewport[0] and
//      Viewport[1] both refer to that rendering.
//
// \see ovrSwapTextureSet, ovrHmd_SubmitFrame
type LayerEyeFovDepth struct {
	Header         LayerHeader
	ColorTexture   [Eye_Count]*SwapTextureSet
	Viewport       [Eye_Count]Recti
	Fov            [Eye_Count]FovPort
	RenderPose     [Eye_Count]Posef
	DepthTexture   [Eye_Count]*SwapTextureSet
	ProjectionDesc TimewarpProjectionDesc
	cStruct        C.ovrLayerEyeFovDepth
}

func (l *LayerEyeFovDepth) ptr() *C.ovrLayerHeader {
	l.cStruct.Header.Type = C.ovrLayerType(l.Header.Type)
	l.cStruct.Header.Flags = C.uint(l.Header.Flags)
	for i := 0; i < Eye_Count; i++ {
		l.cStruct.ColorTexture[i] = l.ColorTexture[i].cSwapTextureSet
		l.cStruct.Viewport[i] = cRecti(l.Viewport[i])
		l.cStruct.Fov[i] = cFovPort(l.Fov[i])
		l.cStruct.RenderPose[i] = cPosef(l.RenderPose[i])
		l.cStruct.DepthTexture[i] = l.DepthTexture[i].cSwapTextureSet
	}
	return (*C.ovrLayerHeader)(unsafe.Pointer(&l.cStruct))
}

// Describes a layer of Quad type, which is a single quad in world or viewer space.
// It is used for both ovrLayerType_QuadInWorld and ovrLayerType_QuadHeadLocked.
// This type of layer represents a single object placed in the world and not a stereo
// view of the world itself.
//
// A typical use of ovrLayerType_QuadInWorld is to draw a television screen in a room
// that for some reason is more convenient to draw as a layer than as part of the main
// view in layer 0. For example, it could implement a 3D popup GUI that is drawn at a
// higher resolution than layer 0 to improve fidelity of the GUI.
//
// A use of ovrLayerType_QuadHeadLocked might be to implement a debug HUD visible in
// the HMD.
//
// Quad layers are visible from both sides; they are not back-face culled.
//
// \see ovrSwapTextureSet, ovrHmd_SubmitFrame
type LayerQuad struct {
	Header         LayerHeader
	ColorTexture   *SwapTextureSet
	Viewport       Recti
	QuadPoseCenter Posef
	QuadSize       Vector2f
	cStruct        C.ovrLayerQuad
}

func (l *LayerQuad) ptr() *C.ovrLayerHeader {
	l.cStruct.Header.Type = C.ovrLayerType(l.Header.Type)
	l.cStruct.Header.Flags = C.uint(l.Header.Flags)
	l.cStruct.ColorTexture = l.ColorTexture.cSwapTextureSet
	l.cStruct.Viewport = cRecti(l.Viewport)
	l.cStruct.QuadPoseCenter = cPosef(l.QuadPoseCenter)
	l.cStruct.QuadSize = cVector2f(l.QuadSize)
	return (*C.ovrLayerHeader)(unsafe.Pointer(&l.cStruct))
}

// Describes a layer which is copied to the HMD as-is. Neither distortion, time warp,
// nor vignetting is applied to ColorTexture before it's copied to the HMD. The application
// can, however implement these kinds of effects itself before submitting the layer.
// This layer can be used for application-based distortion rendering and can also be
// used for implementing a debug HUD that's viewed on the mirror texture.
//
// \see ovrSwapTextureSet, ovrHmd_SubmitFrame
type LayerDirect struct {
	Header       LayerHeader
	ColorTexture [Eye_Count]*SwapTextureSet
	Viewport     [Eye_Count]Recti
	cStruct      C.ovrLayerDirect
}

func (l *LayerDirect) ptr() *C.ovrLayerHeader {
	l.cStruct.Header.Type = C.ovrLayerType(l.Header.Type)
	l.cStruct.Header.Flags = C.uint(l.Header.Flags)
	for i := 0; i < Eye_Count; i++ {
		l.cStruct.ColorTexture[i] = l.ColorTexture[i].cSwapTextureSet
		l.cStruct.Viewport[i] = cRecti(l.Viewport[i])
	}
	return (*C.ovrLayerHeader)(unsafe.Pointer(&l.cStruct))
}

// Destroys an ovrSwapTextureSet and frees all the resources associated with it.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] textureSet Specifies the ovrSwapTextureSet to destroy.
//
// \see ovrHmd_CreateSwapTextureSetD3D11, ovrHmd_CreateSwapTextureSetGL
func (hmd *Hmd) DestroySwapTextureSet(textureSet *SwapTextureSet) {
	C.ovrHmd_DestroySwapTextureSet(hmd.cHmd, textureSet.cSwapTextureSet)
}

// Destroys a mirror texture previously created by one of the mirror texture creation functions.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] mirrorTexture Specifies the ovrTexture to destroy.
//
// \see ovrHmd_CreateMirrorTextureD3D11, ovrHmd_CreateMirrorTextureGL
func (hmd *Hmd) DestroyMirrorTexture(mirrorTexture *Texture) {
	C.ovrHmd_DestroyMirrorTexture(hmd.cHmd, mirrorTexture.cTexture)
}

// Calculates the recommended viewport size for rendering a given eye within the HMD
// with a given FOV cone.
//
// Higher FOV will generally require larger textures to maintain quality.
// Apps packing multiple eye views together on the same texture should ensure there are
// at least 8 pixels of padding between them to prevent texture filtering and chromatic
// aberration causing images to leak between the two eye views.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] eye Specifies which eye (left or right) to calculate for.
// \param[in] fov Specifies the ovrFovPort to use.
// \param[in] pixelsPerDisplayPixel Specifies the ratio of the number of render target pixels
//            to display pixels at the center of distortion. 1.0 is the default value. Lower
//            values can improve performance, higher values give improved quality.
// \return Returns the texture width and height size.
func (hmd *Hmd) GetFovTextureSize(eye EyeType, fov FovPort, pixelsPerDisplayPixel float32) Sizei {
	return goSizei(C.ovrHmd_GetFovTextureSize(hmd.cHmd, C.ovrEyeType(eye), cFovPort(fov), C.float(pixelsPerDisplayPixel)))
}

// Computes the distortion viewport, view adjust, and other rendering parameters for
// the specified eye.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] eyeType Specifies which eye (left or right) for which to perform calculations.
// \param[in] fov Specifies the ovrFovPort to use.
// \return Returns the computed ovrEyeRenderDesc for the given eyeType and field of view.
//
// \see ovrEyeRenderDesc
func (hmd *Hmd) GetRenderDesc(eye EyeType, fov FovPort) EyeRenderDesc {
	result := C.ovrHmd_GetRenderDesc(hmd.cHmd, C.ovrEyeType(eye), cFovPort(fov))
	return EyeRenderDesc{
		Eye:                       EyeType(result.Eye),
		Fov:                       goFovPort(result.Fov),
		DistortedViewport:         goRecti(result.DistortedViewport),
		PixelsPerTanAngleAtCenter: goVector2f(result.PixelsPerTanAngleAtCenter),
		HmdToEyeViewOffset:        goVector3f(result.HmdToEyeViewOffset)}
}

// Submits layers for distortion and display.
//
// ovrHmd_SubmitFrame triggers distortion and processing which might happen asynchronously.
// The function will return when there is room in the submission queue and surfaces
// are available. Distortion might or might not have completed.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
//
// \param[in] frameIndex Specifies the targeted frame index, or 0, to refer to one frame after the last
//        time ovrHmd_SubmitFrame was called.
//
// \param[in] viewScaleDesc Provides additional information needed only if layerPtrList contains
//        a ovrLayerType_QuadInWorld or ovrLayerType_QuadHeadLocked. If NULL, a default
//        version is used based on the current configuration and a 1.0 world scale.
//
// \param[in] layerPtrList Specifies a list of ovrLayer pointers, which can include NULL entries to
//        indicate that any previously shown layer at that index is to not be displayed.
//        Each layer header must be a part of a layer structure such as ovrLayerEyeFov or ovrLayerQuad,
//        with Header.Type identifying its type. A NULL layerPtrList entry in the array indicates the
//         absence of the given layer.
//
// \param[in] layerCount Indicates the number of valid elements in layerPtrList. The maximum
//        supported layerCount is not currently specified, but may be specified in a future version.
//
// - Layers are drawn in the order they are specified in the array, regardless of the layer type.
//
// - Layers are not remembered between successive calls to ovrHmd_SubmitFrame. A layer must be
//   specified in every call to ovrHmd_SubmitFrame or it won't be displayed.
//
// - If a layerPtrList entry that was specified in a previous call to ovrHmd_SubmitFrame is
//   passed as NULL or is of type ovrLayerType_Disabled, that layer is no longer displayed.
//
// - A layerPtrList entry can be of any layer type and multiple entries of the same layer type
//   are allowed. No layerPtrList entry may be duplicated (i.e. the same pointer as an earlier entry).
//
// <b>Example code</b>
//     \code{.cpp}
//         ovrLayerEyeFov  layer0;
//         ovrLayerQuad    layer1;
//           ...
//         ovrLayerHeader* layers[2] = { &layer0.Header, &layer1.Header };
//         ovrResult result = ovrHmd_SubmitFrame(hmd, frameIndex, nullptr, layers, 2);
//     \endcode
//
// \return Returns an ovrResult for which OVR_SUCCESS(result) is false upon error and true
//         upon one of the possible success values:
//     - ovrSuccess: rendering completed successfully.
//     - ovrSuccess_NotVisible: rendering completed successfully but was not displayed on the HMD,
//       usually because another application currently has ownership of the HMD. Applications receiving
//       this result should stop rendering new content, but continue to call ovrHmd_SubmitFrame periodically
//       until it returns a value other than ovrSuccess_NotVisible.
//
// \see ovrHmd_GetFrameTiming, ovrViewScaleDesc, ovrLayerHeader
func (hmd *Hmd) SubmitFrame(frameIndex uint, viewScaleDesc *ViewScaleDesc, layers []LayerInterface) (bool, error) {
	var cViewScaleDesc C.ovrViewScaleDesc
	var cViewScaleDescPtr *C.ovrViewScaleDesc = nil
	if viewScaleDesc != nil {
		for i := 0; i < Eye_Count; i++ {
			cViewScaleDesc.HmdToEyeViewOffset[i] = cVector3f(viewScaleDesc.HmdToEyeViewOffset[i])
		}
		cViewScaleDesc.HmdSpaceToWorldScaleInMeters = C.float(viewScaleDesc.HmdSpaceToWorldScaleInMeters)
		cViewScaleDescPtr = &cViewScaleDesc
	}

	layerCount := len(layers)
	if cLayers == nil || cap(cLayers) < layerCount {
		cLayers = make([]*C.ovrLayerHeader, 0, layerCount*2)
	}
	cLayers = cLayers[:0]
	for i := 0; i < layerCount; i++ {
		cLayers = append(cLayers, layers[i].ptr())
	}
	result := C.ovrHmd_SubmitFrame(hmd.cHmd, C.uint(frameIndex), cViewScaleDescPtr, &cLayers[0], C.uint(layerCount))

	if result == C.ovrSuccess_NotVisible {
		return false, nil
	}
	err := errorForResult(result)
	if err != nil {
		return false, err
	}
	return true, nil
}

// This is used in SubmitFrame to pass the layers to C. Having a package variable avoids allocating memory on every call.
var cLayers []*C.ovrLayerHeader

// Gets the ovrFrameTiming for the given frame index.
//
// The application should increment frameIndex for each successively targeted frame,
// and pass that index to any relevent OVR functions that need to apply to the frame
// identified by that index.
//
// This function is thread-safe and allows for multiple application threads to target
// their processing to the same displayed frame.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] frameIndex Identifies the frame the caller wishes to target.
// \return Returns the ovrFrameTiming for the given frameIndex.
// \see ovrFrameTiming, ovrHmd_ResetFrameTiming
func (hmd *Hmd) GetFrameTiming(frameIndex uint) FrameTiming {
	result := C.ovrHmd_GetFrameTiming(hmd.cHmd, C.uint(frameIndex))
	return FrameTiming{
		DisplayMidpointSeconds: float64(result.DisplayMidpointSeconds),
		FrameIntervalSeconds:   float64(result.FrameIntervalSeconds),
		AppFrameIndex:          float64(result.AppFrameIndex),
		DisplayFrameIndex:      float64(result.DisplayFrameIndex)}
}

// Initializes and resets frame time tracking.
//
// This is typically not necessary, but is helpful if the application changes vsync state or
// video mode. vsync is assumed to be on if this isn't called. Resets internal frame index to
// the specified number.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] frameIndex Identifies the frame the caller wishes to target.
// \see ovrHmd_GetFrameTiming
func (hmd *Hmd) ResetFrameTiming(frameIndex uint) {
	C.ovrHmd_ResetFrameTiming(hmd.cHmd, C.uint(frameIndex))
}

// Returns global, absolute high-resolution time in seconds.
//
// The time frame of reference for this function is not specified and should not be
// depended upon.
//
// \return Returns seconds as a floating point value.
// \see ovrPoseStatef, ovrSensorData, ovrFrameTiming
//
func GetTimeInSeconds() float64 {
	return float64(C.ovr_GetTimeInSeconds())
}

// Reads a boolean property.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid for only the call.
// \param[in] defaultVal specifes the value to return if the property couldn't be read.
// \return Returns the property interpreted as a boolean value. Returns defaultVal if
//         the property doesn't exist.
func (hmd *Hmd) GetBool(propertyName string, defaultVal bool) bool {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))

	return goBool(C.ovrHmd_GetBool(hmd.cHmd, cPropertyName, ovrBool(defaultVal)))
}

// Writes or creates a boolean property.
// If the property wasn't previously a boolean property, it is changed to a boolean property.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid only for the call.
// \param[in] value The value to write.
// \return Returns true if successful, otherwise false. A false result should only occur if the property
//         name is empty or if the property is read-only.
func (hmd *Hmd) SetBool(propertyName string, value bool) error {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))

	result := C.ovrHmd_SetBool(hmd.cHmd, cPropertyName, ovrBool(value))
	if result == C.ovrFalse {
		return errors.New("Failed to set property")
	}
	return nil
}

// Reads an integer property.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid only for the call.
// \param[in] defaultVal Specifes the value to return if the property couldn't be read.
// \return Returns the property interpreted as an integer value. Returns defaultVal if
//         the property doesn't exist.
func (hmd *Hmd) GetInt(propertyName string, defaultVal int) int {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))

	return int(C.ovrHmd_GetInt(hmd.cHmd, cPropertyName, C.int(defaultVal)))
}

// Writes or creates an integer property.
//
// If the property wasn't previously a boolean property, it is changed to an integer property.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid only for the call.
// \param[in] value The value to write.
// \return Returns true if successful, otherwise false. A false result should only occur if the property
//         name is empty or if the property is read-only.
func (hmd *Hmd) SetInt(propertyName string, value int) error {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))

	result := C.ovrHmd_SetInt(hmd.cHmd, cPropertyName, C.int(value))
	if result == C.ovrFalse {
		return errors.New("Failed to set property")
	}
	return nil
}

// Reads a float property.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid only for the call.
// \param[in] defaultVal specifes the value to return if the property couldn't be read.
// \return Returns the property interpreted as an float value. Returns defaultVal if
//         the property doesn't exist.
func (hmd *Hmd) GetFloat(propertyName string, defaultVal float32) float32 {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))

	return float32(C.ovrHmd_GetFloat(hmd.cHmd, cPropertyName, C.float(defaultVal)))
}

// Writes or creates a float property.
// If the property wasn't previously a float property, it's changed to a float property.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid only for the call.
// \param[in] value The value to write.
// \return Returns true if successful, otherwise false. A false result should only occur if the property
//         name is empty or if the property is read-only.
func (hmd *Hmd) SetFloat(propertyName string, value float32) error {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))

	result := C.ovrHmd_SetFloat(hmd.cHmd, cPropertyName, C.float(value))
	if result == C.ovrFalse {
		return errors.New("Failed to set property")
	}
	return nil
}

// Reads a float array property.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid only for the call.
// \param[in] values An array of float to write to.
// \param[in] valuesCapacity Specifies the maximum number of elements to write to the values array.
// \return Returns the number of elements read, or 0 if property doesn't exist or is empty.
func (hmd *Hmd) GetFloatArray(propertyName string, valuesCapacity uint) []float32 {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))

	result := make([]float32, valuesCapacity)
	count := uint(C.ovrHmd_GetFloatArray(hmd.cHmd, cPropertyName, (*C.float)(&result[0]), C.uint(valuesCapacity)))
	return result[:count]
}

// Writes or creates a float array property.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid only for the call.
// \param[in] values An array of float to write from.
// \param[in] valuesSize Specifies the number of elements to write.
// \return Returns true if successful, otherwise false. A false result should only occur if the property
//         name is empty or if the property is read-only.
func (hmd *Hmd) SetFloatArray(propertyName string, values []float32) error {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))

	result := C.ovrHmd_SetFloatArray(hmd.cHmd, cPropertyName, (*C.float)(&values[0]), C.uint(len(values)))
	if result == C.ovrFalse {
		return errors.New("Failed to set property")
	}
	return nil
}

// Reads a string property.
// Strings are UTF8-encoded and null-terminated.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid only for the call.
// \param[in] defaultVal Specifes the value to return if the property couldn't be read.
// \return Returns the string property if it exists. Otherwise returns defaultVal, which can be specified as NULL.
//         The return memory is guaranteed to be valid until next call to ovrHmd_GetString or
//         until the HMD is destroyed, whichever occurs first.
func (hmd *Hmd) GetString(propertyName string, defaultVal string) string {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))
	cDefaultVal := C.CString(defaultVal)
	defer C.free(unsafe.Pointer(cDefaultVal))

	return C.GoString(C.ovrHmd_GetString(hmd.cHmd, cPropertyName, cDefaultVal))
}

// Writes or creates a string property.
// Strings are UTF8-encoded and null-terminated.
//
// \param[in] hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in] propertyName The name of the property, which needs to be valid only for the call.
// \param[in] value The string property, which only needs to be valid for the duration of the call.
// \return Returns true if successful, otherwise false. A false result should only occur if the property
//         name is empty or if the property is read-only.
func (hmd *Hmd) SetString(propertyName string, value string) error {
	cPropertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(cPropertyName))
	cValue := C.CString(value)
	defer C.free(unsafe.Pointer(cValue))

	result := C.ovrHmd_SetString(hmd.cHmd, cPropertyName, cValue)
	if result == C.ovrFalse {
		return errors.New("Failed to set property")
	}
	return nil
}
