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
// Multiple calls to ovr_CreateSwapTextureSetD3D11 for the same ovrHmd are supported, but applications
// cannot rely on switching between ovrSwapTextureSets at runtime without a performance penalty.
//
// \param[in]  session Specifies an ovrSession previously returned by ovr_Create.
// \param[in]  format Specifies the texture format.
// \param[in]  width Specifies the requested texture width.
// \param[in]  height Specifies the requested texture height.
// \param[out] outTextureSet Specifies the created ovrSwapTextureSet, which will be valid upon a successful return value, else it will be NULL.
//             This texture set must be eventually destroyed via ovr_DestroySwapTextureSet before destroying the HMD with ovr_Destroy.
//
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information.
//
// \note The \a format provided should be thought of as the format the distortion compositor will use when reading the contents of the
// texture. To that end, it is highly recommended that the application requests swap-texture-set formats that are in sRGB-space (e.g. GL_SRGB_ALPHA8)
// as the distortion compositor does sRGB-correct rendering. Furthermore, the app should then make sure "glEnable(GL_FRAMEBUFFER_SRGB);"
// is called before rendering into these textures. Even though it is not recommended, if the application would like to treat the
// texture as a linear format and do linear-to-gamma conversion in GLSL, then the application can avoid calling "glEnable(GL_FRAMEBUFFER_SRGB);",
// but should still pass in GL_SRGB_ALPHA8 (not GL_RGBA) for the \a format. Failure to do so will cause the distortion compositor
// to apply incorrect gamma conversions leading to gamma-curve artifacts.
//
// \see ovr_DestroySwapTextureSet
func (hmd *Session) CreateSwapTextureSetGL(format uint, width, height int) (*SwapTextureSet, error) {
	var cSet *C.ovrSwapTextureSet
	result := C.ovr_CreateSwapTextureSetGL(hmd.cSession, C.GLuint(format), C.int(width), C.int(height), &cSet)
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
// A second call to ovr_CreateMirrorTextureGL for a given ovrHmd before destroying the first one
// is not supported and will result in an error return.
//
// \param[in]  session Specifies an ovrSession previously returned by ovr_Create.
// \param[in]  format Specifies the texture format.
// \param[in]  width Specifies the requested texture width.
// \param[in]  height Specifies the requested texture height.
// \param[out] outMirrorTexture Specifies the created ovrSwapTexture, which will be valid upon a successful return value, else it will be NULL.
//             This texture must be eventually destroyed via ovr_DestroyMirrorTexture before destroying the HMD with ovr_Destroy.
//
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information.
//
// \note The \a format provided should be thought of as the format the distortion compositor will use when writing into the mirror
// texture. It is highly recommended that mirror textures are requested as GL_SRGB_ALPHA8 because the distortion compositor
// does sRGB-correct rendering. If the application requests a non-sRGB format (e.g. GL_RGBA) as the mirror texture,
// then the application might have to apply a manual linear-to-gamma conversion when reading from the mirror texture.
// Failure to do so can result in incorrect gamma conversions leading to gamma-curve artifacts and color banding.
//
// \see ovr_DestroyMirrorTexture
func (hmd *Session) CreateMirrorTextureGL(format uint, width, height int) (*Texture, error) {
	var cTexture *C.union_ovrGLTexture
	result := C.ovr_CreateMirrorTextureGL(hmd.cSession, C.GLuint(format), C.int(width), C.int(height), (**C.ovrTexture)(unsafe.Pointer(&cTexture)))
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
