package goovr

// #include "OVR_CAPI_Util.h"
import "C"

/// Enumerates modifications to the projection matrix based on the application's needs.
///
/// \see ovrMatrix4f_Projection
///
type ProjectionModifier int32

const (
	Projection_None              ProjectionModifier = C.ovrProjection_None
	Projection_RightHanded                          = C.ovrProjection_RightHanded
	Projection_FarLessThanNear                      = C.ovrProjection_FarLessThanNear
	Projection_FarClipAtInfinity                    = C.ovrProjection_FarClipAtInfinity
	Projection_ClipRangeOpenGL                      = C.ovrProjection_ClipRangeOpenGL
)

// Used to generate projection from ovrEyeDesc::Fov.
//
// \param[in] fov Specifies the ovrFovPort to use.
// \param[in] znear Distance to near Z limit.
// \param[in] zfar Distance to far Z limit.
// \param[in] projectionModFlags A combination of the ovrProjectionModifier flags.
//
// \return Returns the calculated projection matrix.
//
// \see ovrProjectionModifier
func Matrix4f_Projection(fov FovPort, znear, zfar float32, projectionModFlags ProjectionModifier) Matrix4f {
	return goMatrix4f(C.ovrMatrix4f_Projection(cFovPort(fov), C.float(znear), C.float(zfar), C.uint(projectionModFlags)))
}

// Extracts the required data from the result of ovrMatrix4f_Projection.
//
// \param[in] projection Specifies the project matrix from which to extract ovrTimewarpProjectionDesc.
// \return Returns the extracted ovrTimewarpProjectionDesc.
// \see ovrTimewarpProjectionDesc
func TimewarpProjectionDesc_FromProjection(projection Matrix4f) TimewarpProjectionDesc {
	result := C.ovrTimewarpProjectionDesc_FromProjection(cMatrix4f(projection))
	return TimewarpProjectionDesc{
		Projection22: float32(result.Projection22),
		Projection23: float32(result.Projection23),
		Projection32: float32(result.Projection32)}
}

// Generates an orthographic sub-projection.
//
// Used for 2D rendering, Y is down.
//
// \param[in] projection The perspective matrix that the orthographic matrix is derived from.
// \param[in] orthoScale Equal to 1.0f / pixelsPerTanAngleAtCenter.
// \param[in] orthoDistance Equal to the distance from the camera in meters, such as 0.8m.
// \param[in] hmdToEyeViewOffsetX Specifies the offset of the eye from the center.
//
// \return Returns the calculated projection matrix.
func Matrix4f_OrthoSubProjection(projection Matrix4f, orthoScale Vector2f, orthoDistance, hmdToEyeViewOffsetX float32) Matrix4f {
	return goMatrix4f(C.ovrMatrix4f_OrthoSubProjection(cMatrix4f(projection), cVector2f(orthoScale), C.float(orthoDistance), C.float(hmdToEyeViewOffsetX)))
}

// Computes offset eye poses based on headPose returned by ovrTrackingState.
//
// \param[in] headPose Indicates the HMD position and orientation to use for the calculation.
// \param[in] hmdToEyeViewOffset Can be ovrEyeRenderDesc.HmdToEyeViewOffset returned from
//            ovrHmd_GetRenderDesc. For monoscopic rendering, use a vector that is the average
//            of the two vectors for both eyes.
// \param[out] outEyePoses If outEyePoses are used for rendering, they should be passed to
//             ovrHmd_SubmitFrame in ovrLayerEyeFov::RenderPose or ovrLayerEyeFovDepth::RenderPose.
func CalcEyePoses(headPose Posef, hmdToEyeViewOffset [2]Vector3f) [2]Posef {
	cHeadPose := C.ovrPosef{Orientation: cQuatf(headPose.Orientation), Position: cVector3f(headPose.Position)}
	var cHmdToEyeViewOffset [2]C.ovrVector3f
	cHmdToEyeViewOffset[0] = cVector3f(hmdToEyeViewOffset[0])
	cHmdToEyeViewOffset[1] = cVector3f(hmdToEyeViewOffset[1])
	var cEyePoses [2]C.ovrPosef
	C.ovr_CalcEyePoses(cHeadPose, &cHmdToEyeViewOffset[0], &cEyePoses[0])
	return [2]Posef{goPosef(cEyePoses[0]), goPosef(cEyePoses[1])}
}

// Returns the predicted head pose in outHmdTrackingState and offset eye poses in outEyePoses.
//
// This is a thread-safe function where caller should increment frameIndex with every frame
// and pass that index where applicable to functions called on the rendering thread.
// Assuming outEyePoses are used for rendering, it should be passed as a part of ovrLayerEyeFov.
// The caller does not need to worry about applying HmdToEyeViewOffset to the returned outEyePoses variables.
//
// \param[in]  hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in]  frameIndex Specifies the targeted frame index, or 0 to refer to one frame after
//             the last time ovrHmd_SubmitFrame was called.
// \param[in]  hmdToEyeViewOffset Can be ovrEyeRenderDesc.HmdToEyeViewOffset returned from
//             ovrHmd_GetRenderDesc. For monoscopic rendering, use a vector that is the average
//             of the two vectors for both eyes.
// \param[out] outEyePoses The predicted eye poses.
// \param[out] outHmdTrackingState The predicted ovrTrackingState. May be NULL, in which case it is ignored.
func (hmd *Hmd) GetEyePoses(frameIndex uint, hmdToEyeViewOffset [2]Vector3f) ([2]Posef, TrackingState) {
	var cHmdToEyeViewOffset [2]C.ovrVector3f
	cHmdToEyeViewOffset[0] = cVector3f(hmdToEyeViewOffset[0])
	cHmdToEyeViewOffset[1] = cVector3f(hmdToEyeViewOffset[1])
	var cEyePoses [2]C.ovrPosef
	var cTrackingState C.ovrTrackingState
	C.ovrHmd_GetEyePoses(hmd.cHmd, C.uint(frameIndex), &cHmdToEyeViewOffset[0], &cEyePoses[0], &cTrackingState)
	return [2]Posef{goPosef(cEyePoses[0]), goPosef(cEyePoses[1])}, goTrackingState(cTrackingState)
}

// Waits until the specified absolute time.
//
// \deprecated This function may be removed in a future version.
//
// \param[in] absTime Specifies the absolute future time to wait until.
//
// \see ovr_GetTimeInSeconds
func WaitTillTime(absTime float64) float64 {
	return float64(C.ovr_WaitTillTime(C.double(absTime)))
}
