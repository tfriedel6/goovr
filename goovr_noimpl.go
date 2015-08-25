// +build !windows

package goovr

import (
	"unsafe"
	"errors"
	"time"
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
	Hmd_None      HmdType = iota
	Hmd_DK1
	Hmd_DKHD
	Hmd_DK2
	Hmd_BlackStar
	Hmd_CB
	Hmd_Other
)

type HmdCaps int32

const (
	HmdCap_DebugDevice       HmdCaps = iota
	HmdCap_LowPersistence
	HmdCap_DynamicPrediction
	HmdCap_NoVSync
	HmdCap_Writable_Mask
	HmdCap_Service_Mask
)

// Tracking capability bits reported by the device.
// Used with ovrHmd_ConfigureTracking.
type TrackingCaps int32

const (
	TrackingCap_Orientation      TrackingCaps = iota
	TrackingCap_MagYawCorrection
	TrackingCap_Position
	TrackingCap_Idle
)

// Specifies which eye is being used for rendering.
// This type explicitly does not include a third "NoStereo" monoscopic option, as such is
// not required for an HMD-centered API.
type EyeType int32

const (
	Eye_Left  EyeType = iota
	Eye_Right
	Eye_Count
)

// This is a complete descriptor of the HMD.
type Hmd struct {
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
	RawSensorData          SensorData
	StatusFlags            StatusBits
	LastCameraFrameCounter uint32
}

type FrameTiming struct {
	DisplayMidpointSeconds float64
	FrameIntervalSeconds   float64
	AppFrameIndex          float64
	DisplayFrameIndex      float64
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
	RenderAPI_None           RenderAPIType = iota
	RenderAPI_OpenGL
	RenderAPI_Android_GLES
	RenderAPI_D3D9_Obsolete
	RenderAPI_D3D10_Obsolete
	RenderAPI_D3D11
	RenderAPI_Count
)

type TextureHeader struct {
	API         RenderAPIType
	TextureSize Sizei
}

type Texture struct {
	Header   TextureHeader
	OGL      *GLTextureData
	D3D11    *D3D11TextureData
}

type SwapTextureSet struct {
	Textures        []Texture
	TextureCount    int
}

func (s *SwapTextureSet) SetCurrentIndex(value int) {
}

func (s *SwapTextureSet) CurrentIndex() int {
	return 0
}

type InitFlags int32

const (
	Init_Debug          InitFlags = iota
	Init_ServerOptional
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

func Hmd_Detect() (int, error) {
	return 0, nil
}

func Hmd_Create(index int) (*Hmd, error) {
	return nil, notAvailableErr
}

func Hmd_CreateDebug(typ HmdType) (*Hmd, error) {
	return nil, notAvailableErr
}

func (hmd *Hmd) Destroy() {
}

func (hmd *Hmd) GetEnabledCaps() HmdCaps {
	return 0
}

func (hmd *Hmd) SetEnabledCaps(hmdCaps HmdCaps) {
}

func (hmd *Hmd) ConfigureTracking(supportedTrackingCaps, requiredTrackingCaps TrackingCaps) error {
	return notAvailableErr
}

func (hmd *Hmd) RecenterPose() {
}

func (hmd *Hmd) GetTrackingState(absTime float64) TrackingState {
	return TrackingState{}
}

type LayerType int32

const (
	LayerType_Disabled       LayerType = iota
	LayerType_EyeFov
	LayerType_EyeFovDepth
	LayerType_QuadInWorld
	LayerType_QuadHeadLocked
	LayerType_Direct
)

type LayerFlags int32

const (
	LayerFlag_HighQuality               LayerFlags = iota
	LayerFlag_TextureOriginAtBottomLeft
)

type LayerHeader struct {
	Type  LayerType
	Flags LayerFlags
}

type LayerInterface interface {
	ptr() unsafe.Pointer
}

type LayerEyeFov struct {
	Header       LayerHeader
	ColorTexture [Eye_Count]*SwapTextureSet
	Viewport     [Eye_Count]Recti
	Fov          [Eye_Count]FovPort
	RenderPose   [Eye_Count]Posef
}

func (l *LayerEyeFov) ptr() unsafe.Pointer {
	return nil
}

type LayerEyeFovDepth struct {
	Header         LayerHeader
	ColorTexture   [Eye_Count]*SwapTextureSet
	Viewport       [Eye_Count]Recti
	Fov            [Eye_Count]FovPort
	RenderPose     [Eye_Count]Posef
	DepthTexture   [Eye_Count]*SwapTextureSet
	ProjectionDesc TimewarpProjectionDesc
}

func (l *LayerEyeFovDepth) ptr() unsafe.Pointer {
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

func (hmd *Hmd) DestroySwapTextureSet(textureSet *SwapTextureSet) {
}

func (hmd *Hmd) DestroyMirrorTexture(mirrorTexture *Texture) {
}

func (hmd *Hmd) GetFovTextureSize(eye EyeType, fov FovPort, pixelsPerDisplayPixel float32) Sizei {
	return Sizei{}
}

func (hmd *Hmd) GetRenderDesc(eye EyeType, fov FovPort) EyeRenderDesc {
	return EyeRenderDesc{}
}

func (hmd *Hmd) SubmitFrame(frameIndex uint, viewScaleDesc *ViewScaleDesc, layers []LayerInterface) (bool, error) {
	return false, notAvailableErr
}

func (hmd *Hmd) GetFrameTiming(frameIndex uint) FrameTiming {
	return FrameTiming{}
}

func (hmd *Hmd) ResetFrameTiming(frameIndex uint) {
}

func GetTimeInSeconds() float64 {
	return float64(time.Now().UnixNano()) / 1000000000
}

func (hmd *Hmd) GetBool(propertyName string, defaultVal bool) bool {
	return false
}

func (hmd *Hmd) SetBool(propertyName string, value bool) error {
	return notAvailableErr
}

func (hmd *Hmd) GetInt(propertyName string, defaultVal int) int {
	return 0
}

func (hmd *Hmd) SetInt(propertyName string, value int) error {
	return notAvailableErr
}

func (hmd *Hmd) GetFloat(propertyName string, defaultVal float32) float32 {
	return 0
}

func (hmd *Hmd) SetFloat(propertyName string, value float32) error {
	return notAvailableErr
}

func (hmd *Hmd) GetFloatArray(propertyName string, valuesCapacity uint) []float32 {
	return []float32{}
}

func (hmd *Hmd) SetFloatArray(propertyName string, values []float32) error {
	return notAvailableErr
}

func (hmd *Hmd) GetString(propertyName string, defaultVal string) string {
	return ""
}

func (hmd *Hmd) SetString(propertyName string, value string) error {
	return notAvailableErr
}
