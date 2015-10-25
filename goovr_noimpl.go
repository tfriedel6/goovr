// +build !windows

package goovr

import (
	"errors"
	"time"
	"unsafe"
)

var notAvailableErr = errors.New("OVR not available on this platform")

type Vector2i struct {
	X, Y int
}

type Sizei struct {
	W, H int
}

type Recti struct {
	Pos  Vector2i
	Size Sizei
}

type Quatf struct {
	X, Y, Z, W float32
}

type Vector2f struct {
	X, Y float32
}

type Vector3f struct {
	X, Y, Z float32
}

type Matrix4f struct {
	M [4][4]float32
}

type Posef struct {
	Orientation Quatf
	Position    Vector3f
}

type PoseStatef struct {
	ThePose             Posef
	AngularVelocity     Vector3f
	LinearVelocity      Vector3f
	AngularAcceleration Vector3f
	LinearAcceleration  Vector3f
	TimeInSeconds       float64
}

type FovPort struct {
	UpTan    float32
	DownTan  float32
	LeftTan  float32
	RightTan float32
}

type HmdType int32

const (
	Hmd_None HmdType = iota
	Hmd_DK1
	Hmd_DKHD
	Hmd_DK2
	Hmd_CB
	Hmd_Other
	Hmd_E3_2015
	Hmd_ES06
	Hmd_ES09
)

type HmdCaps int32

const (
	HmdCap_DebugDevice HmdCaps = iota
	HmdCap_Writable_Mask
	HmdCap_Service_Mask
)

// Tracking capability bits reported by the device.
// Used with ovrHmd_ConfigureTracking.
type TrackingCaps int32

const (
	TrackingCap_Orientation TrackingCaps = iota
	TrackingCap_MagYawCorrection
	TrackingCap_Position
	TrackingCap_Idle
)

// Specifies which eye is being used for rendering.
// This type explicitly does not include a third "NoStereo" monoscopic option, as such is
// not required for an HMD-centered API.
type EyeType int32

const (
	Eye_Left EyeType = iota
	Eye_Right
	Eye_Count
)

type GraphicsLuid struct {
	Reserved [8]byte
}

// This is a complete descriptor of the HMD.
type Session struct {
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
	AvailableHmdCaps           HmdCaps
	DefaultHmdCaps             HmdCaps
	AvailableTrackingCaps      TrackingCaps
	DefaultTrackingCaps        TrackingCaps
	DefaultEyeFov              [Eye_Count]FovPort
	MaxEyeFov                  [Eye_Count]FovPort
	Resolution                 Sizei
}

type StatusBits int32

const (
	Status_OrientationTracked StatusBits = iota
	Status_PositionTracked
	Status_CameraPoseTracked
	Status_PositionConnected
	Status_HmdConnected
	Status_EnumSize
)

type SensorData struct {
	Accelerometer Vector3f
	Gyro          Vector3f
	Magnetometer  Vector3f
	Temperature   float32
	TimeInSeconds float32
}

type TrackingState struct {
	HeadPose               PoseStatef
	CameraPose             Posef
	LeveledCameraPose      Posef
	HandPoses              [2]PoseStatef
	RawSensorData          SensorData
	StatusFlags            StatusBits
	HandStatusFlags        [2]StatusBits
	LastCameraFrameCounter uint32
}

type EyeRenderDesc struct {
	Eye                       EyeType
	Fov                       FovPort
	DistortedViewport         Recti
	PixelsPerTanAngleAtCenter Vector2f
	HmdToEyeViewOffset        Vector3f
}

type TimewarpProjectionDesc struct {
	Projection22 float32
	Projection23 float32
	Projection32 float32
}

type ViewScaleDesc struct {
	HmdToEyeViewOffset           [Eye_Count]Vector3f
	HmdSpaceToWorldScaleInMeters float32
}

type RenderAPIType int32

const (
	RenderAPI_None RenderAPIType = iota
	RenderAPI_OpenGL
	RenderAPI_Android_GLES
	RenderAPI_D3D11
	RenderAPI_Count
)

type TextureHeader struct {
	API         RenderAPIType
	TextureSize Sizei
}

type Texture struct {
	Header TextureHeader
	OGL    *GLTextureData
	D3D11  *D3D11TextureData
}

type SwapTextureSet struct {
	Textures     []Texture
	TextureCount int
}

func (s *SwapTextureSet) SetCurrentIndex(value int) {
}

func (s *SwapTextureSet) CurrentIndex() int {
	return 0
}

type Button int

const (
	Button_A Button = iota
	Button_B

	Button_RThumb
	Button_RShoulder
	Button_X
	Button_Y
	Button_LThumb
	Button_LShoulder

	Button_Up
	Button_Down
	Button_Left
	Button_Right
	Button_Enter
	Button_Back

	Button_Private
)

type Touch int

const (
	Touch_A Touch = iota
	Touch_B
	Touch_RThumb
	Touch_RIndexTrigger
	Touch_X
	Touch_Y
	Touch_LThumb
	Touch_LIndexTrigger

	Touch_RIndexPointing
	Touch_RThumbUp
	Touch_LIndexPointing
	Touch_LThumbUp
)

type ControllerType int

const (
	ControllerType_None ControllerType = iota
	ControllerType_LTouch
	ControllerType_RTouch
	ControllerType_Touch
	ControllerType_XBox
	ControllerType_All
)

type HandType int

const (
	Hand_Left HandType = iota
	Hand_Right
)

type InputState struct {
	TimeInSeconds            float64
	ConnectedControllerTypes uint
	Buttons                  Button
	Touches                  Touch
	IndexTrigger             [2]float32
	HandTrigger              [2]float32
	Thumbstick               [2]Vector2f
}

type InitFlags int32

const (
	Init_Debug InitFlags = iota
	Init_RequestVersion
	Init_ForceNoDebug
)

type LogLevel int32

const (
	LogLevel_Debug LogLevel = iota
	LogLevel_Info
	LogLevel_Error
)

type LogCallback func(level int, message string)

type InitParams struct {
	Flags                 InitFlags
	RequestedMinorVersion uint32
	LogCallback           LogCallback
	UserData              uintptr
	ConnectionTimeoutMS   uint32
}

func Initialize(params *InitParams) error {
	return notAvailableErr
}

func Shutdown() {
}

type ErrorInfo struct {
	Result      uint32
	ErrorString string
}

func GetLastErrorInfo() ErrorInfo {
	return ErrorInfo{Result: 0, ErrorString: ""}
}

func GetVersionString() string {
	return ""
}

func TraceMessage(level int, message string) (int, error) {
	return 0, nil
}

func Create(pLuid *GraphicsLuid) (*Session, error) {
	return nil, notAvailableErr
}

func (hmd *Session) Destroy() {
}

type SessionStatus struct {
	HasVrFocus bool
	HmdPresent bool
}

func (hmd *Session) GetSessionStatus() (SessionStatus, error) {
	return SessionStatus{}, nil
}

func (hmd *Session) GetEnabledCaps() HmdCaps {
	return 0
}

func (hmd *Session) SetEnabledCaps(hmdCaps HmdCaps) {
}

func (hmd *Session) GetTrackingCaps() TrackingCaps {
	return 0
}

func (hmd *Session) ConfigureTracking(supportedTrackingCaps, requiredTrackingCaps TrackingCaps) error {
	return notAvailableErr
}

func (hmd *Session) RecenterPose() {
}

func (hmd *Session) GetTrackingState(absTime float64, latencyMarker bool) TrackingState {
	return TrackingState{}
}

func (hmd *Session) GetInputState(controllerTypeMask uint) (InputState, error) {
	return InputState{}, nil
}

func (hmd *Session) SetControllerVibration(controllerTypeMask uint, frequency, amplitude float32) error {
	return nil
}

type LayerType int32

const (
	LayerType_Disabled LayerType = iota
	LayerType_EyeFov
	LayerType_EyeFovDepth
	LayerType_Quad
	LayerType_EyeMatrix
	LayerType_Direct
)

type LayerFlags int32

const (
	LayerFlag_HighQuality LayerFlags = iota
	LayerFlag_TextureOriginAtBottomLeft
	LayerFlag_HeadLocked
)

type LayerHeader struct {
	Type  LayerType
	Flags LayerFlags
}

type LayerInterface interface {
	ptr() unsafe.Pointer
}

type LayerEyeFov struct {
	Header           LayerHeader
	ColorTexture     [Eye_Count]*SwapTextureSet
	Viewport         [Eye_Count]Recti
	Fov              [Eye_Count]FovPort
	RenderPose       [Eye_Count]Posef
	SensorSampleTime float64
}

func (l *LayerEyeFov) ptr() unsafe.Pointer {
	return nil
}

type LayerEyeFovDepth struct {
	Header           LayerHeader
	ColorTexture     [Eye_Count]*SwapTextureSet
	Viewport         [Eye_Count]Recti
	Fov              [Eye_Count]FovPort
	RenderPose       [Eye_Count]Posef
	SensorSampleTime float64
	DepthTexture     [Eye_Count]*SwapTextureSet
	ProjectionDesc   TimewarpProjectionDesc
}

func (l *LayerEyeFovDepth) ptr() unsafe.Pointer {
	return nil
}

type LayerEyeMatrix struct {
	Header           LayerHeader
	ColorTexture     [Eye_Count]*SwapTextureSet
	Viewport         [Eye_Count]Recti
	RenderPose       [Eye_Count]Posef
	Matrix           [Eye_Count]Matrix4f
	SensorSampleTime float64
}

func (l *LayerEyeMatrix) ptr() unsafe.Pointer {
	return nil
}

type LayerQuad struct {
	Header         LayerHeader
	ColorTexture   *SwapTextureSet
	Viewport       Recti
	QuadPoseCenter Posef
	QuadSize       Vector2f
}

func (l *LayerQuad) ptr() unsafe.Pointer {
	return nil
}

type LayerDirect struct {
	Header       LayerHeader
	ColorTexture [Eye_Count]*SwapTextureSet
	Viewport     [Eye_Count]Recti
}

func (l *LayerDirect) ptr() unsafe.Pointer {
	return nil
}

func (hmd *Session) DestroySwapTextureSet(textureSet *SwapTextureSet) {
}

func (hmd *Session) DestroyMirrorTexture(mirrorTexture *Texture) {
}

func (hmd *Session) GetFovTextureSize(eye EyeType, fov FovPort, pixelsPerDisplayPixel float32) Sizei {
	return Sizei{}
}

func (hmd *Session) GetRenderDesc(eye EyeType, fov FovPort) EyeRenderDesc {
	return EyeRenderDesc{}
}

func (hmd *Session) SubmitFrame(frameIndex uint, viewScaleDesc *ViewScaleDesc, layers []LayerInterface) (bool, error) {
	return false, notAvailableErr
}

func (hmd *Session) GetPredictedDisplayTime(frameIndex uint64) float64 {
	return 0
}

func (hmd *Session) ResetFrameTiming(frameIndex uint) {
}

func GetTimeInSeconds() float64 {
	return float64(time.Now().UnixNano()) / 1000000000
}

type PerfHudMode int

const (
	PerfHud_Off PerfHudMode = iota
	PerfHud_LatencyTiming
	PerfHud_RenderTiming
	PerfHud_PerfHeadroom
	PerfHud_VersionInfo
)

type LayerHudMode int

const (
	LayerHud_Off LayerHudMode = iota
	LayerHud_Info
)

type DebugHudStereoMode int

const (
	DebugHudStereo_Off DebugHudStereoMode = iota
	DebugHudStereo_Quad
	DebugHudStereo_QuadWithCrosshair
	DebugHudStereo_CrosshairAtInfinity
)

func (hmd *Session) ResetBackOfHeadTracking() {
}

func (hmd *Session) ResetMulticameraTracking() {
}

func (hmd *Session) GetBool(propertyName string, defaultVal bool) bool {
	return false
}

func (hmd *Session) SetBool(propertyName string, value bool) error {
	return notAvailableErr
}

func (hmd *Session) GetInt(propertyName string, defaultVal int) int {
	return 0
}

func (hmd *Session) SetInt(propertyName string, value int) error {
	return notAvailableErr
}

func (hmd *Session) GetFloat(propertyName string, defaultVal float32) float32 {
	return 0
}

func (hmd *Session) SetFloat(propertyName string, value float32) error {
	return notAvailableErr
}

func (hmd *Session) GetFloatArray(propertyName string, valuesCapacity uint) []float32 {
	return []float32{}
}

func (hmd *Session) SetFloatArray(propertyName string, values []float32) error {
	return notAvailableErr
}

func (hmd *Session) GetString(propertyName string, defaultVal string) string {
	return ""
}

func (hmd *Session) SetString(propertyName string, value string) error {
	return notAvailableErr
}
