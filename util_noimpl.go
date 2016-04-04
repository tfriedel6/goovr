// +build !windows

package goovr

/// Enumerates modifications to the projection matrix based on the application's needs.
///
/// \see ovrMatrix4f_Projection
type ProjectionModifier int32

const (
	/// Use for generating a default projection matrix that is:
	/// * Right-handed.
	/// * Near depth values stored in the depth buffer are smaller than far depth values.
	/// * Both near and far are explicitly defined.
	/// * With a clipping range that is (0 to w).
	Projection_None ProjectionModifier = 0

	/// Enable if using left-handed transformations in your application.
	Projection_LeftHanded

	/// After the projection transform is applied, far values stored in the depth buffer will be less than closer depth values.
	/// NOTE: Enable only if the application is using a floating-point depth buffer for proper precision.
	Projection_FarLessThanNear

	/// When this flag is used, the zfar value pushed into ovrMatrix4f_Projection() will be ignored
	/// NOTE: Enable only if ovrProjection_FarLessThanNear is also enabled where the far clipping plane will be pushed to infinity.
	Projection_FarClipAtInfinity

	/// Enable if the application is rendering with OpenGL and expects a projection matrix with a clipping range of (-w to w).
	/// Ignore this flag if your application already handles the conversion from D3D range (0 to w) to OpenGL.
	Projection_ClipRangeOpenGL
)

/// Return values for ovr_Detect.
///
/// \see ovr_Detect
type DetectResult struct {
	/// Is ovrFalse when the Oculus Service is not running.
	///   This means that the Oculus Service is either uninstalled or stopped.
	///   IsOculusHMDConnected will be ovrFalse in this case.
	/// Is ovrTrue when the Oculus Service is running.
	///   This means that the Oculus Service is installed and running.
	///   IsOculusHMDConnected will reflect the state of the HMD.
	IsOculusServiceRunning bool

	/// Is ovrFalse when an Oculus HMD is not detected.
	///   If the Oculus Service is not running, this will be ovrFalse.
	/// Is ovrTrue when an Oculus HMD is detected.
	///   This implies that the Oculus Service is also installed and running.
	IsOculusHMDConnected bool
}

/// Detects Oculus Runtime and Device Status
///
/// Checks for Oculus Runtime and Oculus HMD device status without loading the LibOVRRT
/// shared library.  This may be called before ovr_Initialize() to help decide whether or
/// not to initialize LibOVR.
///
/// \param[in] timeoutMilliseconds Specifies a timeout to wait for HMD to be attached or 0 to poll.
///
/// \return Returns an ovrDetectResult object indicating the result of detection.
///
/// \see ovrDetectResult
func Detect(timeoutMsec int) DetectResult {
	return DetectResult{}
}

/// Used to generate projection from ovrEyeDesc::Fov.
///
/// \param[in] fov Specifies the ovrFovPort to use.
/// \param[in] znear Distance to near Z limit.
/// \param[in] zfar Distance to far Z limit.
/// \param[in] projectionModFlags A combination of the ovrProjectionModifier flags.
///
/// \return Returns the calculated projection matrix.
///
/// \see ovrProjectionModifier
func Matrix4f_Projection(fov FovPort, znear, zfar float32, projectionModFlags ProjectionModifier) Matrix4f {
	return Matrix4f{}
}

/// Extracts the required data from the result of ovrMatrix4f_Projection.
///
/// \param[in] projection Specifies the project matrix from which to extract ovrTimewarpProjectionDesc.
/// \param[in] projectionModFlags A combination of the ovrProjectionModifier flags.
/// \return Returns the extracted ovrTimewarpProjectionDesc.
/// \see ovrTimewarpProjectionDesc
func TimewarpProjectionDesc_FromProjection(projection Matrix4f, projectionModFlags ProjectionModifier) TimewarpProjectionDesc {
	return TimewarpProjectionDesc{}
}

/// Generates an orthographic sub-projection.
///
/// Used for 2D rendering, Y is down.
///
/// \param[in] projection The perspective matrix that the orthographic matrix is derived from.
/// \param[in] orthoScale Equal to 1.0f / pixelsPerTanAngleAtCenter.
/// \param[in] orthoDistance Equal to the distance from the camera in meters, such as 0.8m.
/// \param[in] HmdToEyeOffsetX Specifies the offset of the eye from the center.
///
/// \return Returns the calculated projection matrix.
func Matrix4f_OrthoSubProjection(projection Matrix4f, orthoScale Vector2f, orthoDistance, hmdToEyeViewOffsetX float32) Matrix4f {
	return Matrix4f{}
}

/// Computes offset eye poses based on headPose returned by ovrTrackingState.
///
/// \param[in] headPose Indicates the HMD position and orientation to use for the calculation.
/// \param[in] HmdToEyeOffset Can be ovrEyeRenderDesc.HmdToEyeOffset returned from
///            ovr_GetRenderDesc. For monoscopic rendering, use a vector that is the average
///            of the two vectors for both eyes.
/// \param[out] outEyePoses If outEyePoses are used for rendering, they should be passed to
///             ovr_SubmitFrame in ovrLayerEyeFov::RenderPose or ovrLayerEyeFovDepth::RenderPose.
func CalcEyePoses(headPose Posef, hmdToEyeOffset [2]Vector3f) [2]Posef {
	return [2]Posef{}
}

/// Returns the predicted head pose in outHmdTrackingState and offset eye poses in outEyePoses.
///
/// This is a thread-safe function where caller should increment frameIndex with every frame
/// and pass that index where applicable to functions called on the rendering thread.
/// Assuming outEyePoses are used for rendering, it should be passed as a part of ovrLayerEyeFov.
/// The caller does not need to worry about applying HmdToEyeOffset to the returned outEyePoses variables.
///
/// \param[in]  hmd Specifies an ovrSession previously returned by ovr_Create.
/// \param[in]  frameIndex Specifies the targeted frame index, or 0 to refer to one frame after
///             the last time ovr_SubmitFrame was called.
/// \param[in]  HmdToEyeOffset Can be ovrEyeRenderDesc.HmdToEyeOffset returned from
///             ovr_GetRenderDesc. For monoscopic rendering, use a vector that is the average
///             of the two vectors for both eyes.
/// \param[in]  latencyMarker Specifies that this call is the point in time where
///             the "App-to-Mid-Photon" latency timer starts from. If a given ovrLayer
///             provides "SensorSampleTimestamp", that will override the value stored here.
/// \param[out] outEyePoses The predicted eye poses.
/// \param[out] outSensorSampleTime The time when this function was called. May be NULL, in which case it is ignored.
func (s *Session) GetEyePoses(frameIndex uint64, latencyMarker bool, hmdToEyeOffset [2]Vector3f) ([2]Posef, float64) {
	return [2]Posef{}, 0
}

/// Tracking poses provided by the SDK come in a right-handed coordinate system. If an application
/// is passing in ovrProjection_LeftHanded into ovrMatrix4f_Projection, then it should also use
/// this function to flip the HMD tracking poses to be left-handed.
///
/// While this utility function is intended to convert a left-handed ovrPosef into a right-handed
/// coordinate system, it will also work for converting right-handed to left-handed since the
/// flip operation is the same for both cases.
///
/// \param[in]  inPose that is right-handed
/// \param[out] outPose that is requested to be left-handed (can be the same pointer to inPose)
func Posef_FlipHandedness(v Posef) Posef {
	return Posef{}
}
