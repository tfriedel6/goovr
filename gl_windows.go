package goovr

// #include "OVR_CAPI_GL.h"
// ovrTexture* getTexturePointerFromArray(ovrTexture* textures, int index);
import "C"
import (
	"unsafe"
)

type GLTextureData struct {
	TexId C.GLuint
}

// Creates a Texture Set suitable for use with OpenGL.
//
// \param[in]  hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in]  format Specifies the texture format.
// \param[in]  width Specifies the requested texture width.
// \param[in]  height Specifies the requested texture height.
// \param[out] outTextureSet Specifies the created ovrSwapTextureSet, which will be valid only upon a successful return value.
//             This texture set must be eventually destroyed via ovrHmd_DestroySwapTextureSet before destroying the HMD with ovrHmd_Destroy.
//
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information.
//
// \see ovrHmd_DestroySwapTextureSet
func (hmd *Hmd) CreateSwapTextureSetGL(format uint, width, height int) (*SwapTextureSet, error) {
	var cSet *C.ovrSwapTextureSet
	result := C.ovrHmd_CreateSwapTextureSetGL(hmd.cHmd, C.GLuint(format), C.int(width), C.int(height), &cSet)
	err := errorForResult(result)
	if err != nil {
		return nil, err
	}

	textureCount := int(cSet.TextureCount)
	textures := make([]Texture, textureCount)
	for i := 0; i < textureCount; i++ {
		var texture *C.ovrTexture = C.getTexturePointerFromArray(cSet.Textures, C.int(i))
		textures[i] = Texture{
			Header: TextureHeader{
				API:         RenderAPIType(texture.Header.API),
				TextureSize: goSizei(texture.Header.TextureSize)},
			cTexture: texture,
			OGL: &GLTextureData{TexId: (*C.ovrGLTextureData)(unsafe.Pointer(texture)).TexId}}
	}
	return &SwapTextureSet{
		cSwapTextureSet: cSet,
		Textures:        textures,
		TextureCount:    textureCount}, nil
}

// Creates a Mirror Texture which is auto-refreshed to mirror Rift contents produced by this application.
//
// \param[in]  hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in]  format Specifies the texture format.
// \param[in]  width Specifies the requested texture width.
// \param[in]  height Specifies the requested texture height.
// \param[out] outMirrorTexture Specifies the created ovrTexture, which will be valid only upon a successful return value.
//             This texture must be eventually destroyed via ovrHmd_DestroyMirrorTexture before destroying the HMD with ovrHmd_Destroy.
//
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information.
//
// \see ovrHmd_DestroyMirrorTexture
func (hmd *Hmd) CreateMirrorTextureGL(format uint, width, height int) (*Texture, error) {
	var cTexture *C.union_ovrGLTexture
	result := C.ovrHmd_CreateMirrorTextureGL(hmd.cHmd, C.GLuint(format), C.int(width), C.int(height), (**C.ovrTexture)(unsafe.Pointer(&cTexture)))
	err := errorForResult(result)
	if err != nil {
		return nil, err
	}
	return &Texture{
		Header: TextureHeader{
			API:         RenderAPIType((*C.ovrTexture)(unsafe.Pointer(cTexture)).Header.API),
			TextureSize: goSizei((*C.ovrTexture)(unsafe.Pointer(cTexture)).Header.TextureSize)},
		cTexture: (*C.ovrTexture)(unsafe.Pointer(cTexture)),
		OGL:      &GLTextureData{TexId: (*C.ovrGLTextureData)(unsafe.Pointer(cTexture)).TexId}}, nil
}
