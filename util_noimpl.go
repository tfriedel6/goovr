// +build !windows

package goovr

type ProjectionModifier int32

const (
	Projection_None ProjectionModifier = iota
	Projection_RightHanded
	Projection_FarLessThanNear
	Projection_FarClipAtInfinity
	Projection_ClipRangeOpenGL
)

func Matrix4f_Projection(fov FovPort, znear, zfar float32, projectionModFlags ProjectionModifier) Matrix4f {
	return Matrix4f{}
}

func TimewarpProjectionDesc_FromProjection(projection Matrix4f, projectionModFlags ProjectionModifier) TimewarpProjectionDesc {
	return TimewarpProjectionDesc{}
}

func Matrix4f_OrthoSubProjection(projection Matrix4f, orthoScale Vector2f, orthoDistance, hmdToEyeViewOffsetX float32) Matrix4f {
	return Matrix4f{}
}

func CalcEyePoses(headPose Posef, hmdToEyeViewOffset [2]Vector3f) [2]Posef {
	return [2]Posef{Posef{}, Posef{}}
}

func (hmd *Hmd) GetEyePoses(frameIndex uint, hmdToEyeViewOffset [2]Vector3f) ([2]Posef, TrackingState) {
	return [2]Posef{Posef{}, Posef{}}, TrackingState{}
}
