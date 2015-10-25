package goovr

// #include "OVR_CAPI_D3D.h"
// ovrTexture* getTexturePointerFromArray(ovrTexture* textures, int index);
import "C"
import (
	"unsafe"
)

type D3D11TextureData struct {
	PTexture unsafe.Pointer
	PSRView  unsafe.Pointer
}

// Create Texture Set suitable for use with D3D11.
//
// Multiple calls to ovr_CreateSwapTextureSetD3D11 for the same ovrHmd are supported, but applications
// cannot rely on switching between ovrSwapTextureSets at runtime without a performance penalty.
//
// \param[in]  session Specifies an ovrSession previously returned by ovr_Create.
// \param[in]  device Specifies the associated ID3D11Device, which must be the one that the textures will be used with in the application's process.
// \param[in]  desc Specifies requested texture properties. See notes for more info about texture format.
// \param[in]  miscFlags Specifies misc bit flags of type \a ovrSwapTextureSetD3D11Flags used when creating the swap textures
// \param[out] outTextureSet Specifies the created ovrSwapTextureSet, which will be valid upon a successful return value, else it will be NULL.
//             This texture set must be eventually destroyed via ovr_DestroySwapTextureSet before destroying the HMD with ovr_Destroy.
//
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information.
//
// \note The texture format provided in \a desc should be thought of as the format the distortion-compositor will use for the
// ShaderResourceView when reading the contents of the texture. To that end, it is highly recommended that the application
// requests swap-texture-set formats that are in sRGB-space (e.g. DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) as the compositor
// does sRGB-correct rendering. As such, the compositor relies on the GPU's hardware sampler to do the sRGB-to-linear
// conversion. If the application still prefers to render to a linear format (e.g. DXGI_FORMAT_R8G8B8A8_UNORM) while handling the
// linear-to-gamma conversion via HLSL code, then the application must still request the corresponding sRGB format and also use
// the \a ovrSwapTextureSetD3D11_Typeless flag. This will allow the application to create a RenderTargetView that is the desired
// linear format while the compositor continues to treat it as sRGB. Failure to do so will cause the compositor to apply
// unexpected gamma conversions leading to gamma-curve artifacts. The \a ovrSwapTextureSetD3D11_Typeless flag for depth buffer
// formats (e.g. DXGI_FORMAT_D32) are ignored as they are always converted to be typeless.
//
// \see ovr_DestroySwapTextureSet
func (hmd *Session) CreateSwapTextureSetD3D11(device unsafe.Pointer, desc unsafe.Pointer, miscFlags uint) (*SwapTextureSet, error) {
	var cSet *C.ovrSwapTextureSet
	result := C.ovr_CreateSwapTextureSetD3D11(hmd.cSession, (*C.ID3D11Device)(device), (*C.D3D11_TEXTURE2D_DESC)(desc), C.uint(miscFlags), &cSet)
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
			D3D11: &D3D11TextureData{
				PTexture: unsafe.Pointer((*C.ovrD3D11TextureData)(unsafe.Pointer(texture)).pTexture),
				PSRView:  unsafe.Pointer((*C.ovrD3D11TextureData)(unsafe.Pointer(texture)).pSRView)}}
	}
	return &SwapTextureSet{
		cSwapTextureSet: cSet,
		Textures:        textures,
		TextureCount:    textureCount}, nil
}

// Create Mirror Texture which is auto-refreshed to mirror Rift contents produced by this application.
//
// A second call to ovr_CreateMirrorTextureD3D11 for a given ovrHmd before destroying the first one
// is not supported and will result in an error return.
//
// \param[in]  session Specifies an ovrSession previously returned by ovr_Create.
// \param[in]  device Specifies the associated ID3D11Device, which must be the one that the textures will be used with in the application's process.
// \param[in]  desc Specifies requested texture properties. See notes for info about texture format.
// \param[in]  miscFlags Specifies misc bit flags of type \a ovrSwapTextureSetD3D11Flags used when creating the swap textures
// \param[out] outMirrorTexture Specifies the created ovrTexture, which will be valid upon a successful return value, else it will be NULL.
//             This texture must be eventually destroyed via ovr_DestroyMirrorTexture before destroying the HMD with ovr_Destroy.
//
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information.
//
// \note The texture format provided in \a desc should be thought of as the format the compositor will use for the RenderTargetView when
// writing into mirror texture. To that end, it is highly recommended that the application requests a mirror texture format that is
// in sRGB-space (e.g. DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) as the compositor does sRGB-correct rendering. If however the application wants
// to still read the mirror texture as a linear format (e.g. DXGI_FORMAT_R8G8B8A8_UNORM) and handle the sRGB-to-linear conversion in
// HLSL code, then it is recommended the application still requests an sRGB format and also use the \a ovrSwapTextureSetD3D11_Typeless
// flag. This will allow the application to bind a ShaderResourceView that is a linear format while the compositor continues
// to treat is as sRGB. Failure to do so will cause the compositor to apply unexpected gamma conversions leading to
// gamma-curve artifacts.
//
// \see ovr_DestroyMirrorTexture
func (hmd *Session) CreateMirrorTextureD3D11(device unsafe.Pointer, desc unsafe.Pointer, miscFlags uint) (*Texture, error) {
	var cTexture *C.union_ovrD3D11Texture
	result := C.ovr_CreateMirrorTextureD3D11(hmd.cSession, (*C.ID3D11Device)(device), (*C.D3D11_TEXTURE2D_DESC)(desc), C.uint(miscFlags), (**C.ovrTexture)(unsafe.Pointer(&cTexture)))
	err := errorForResult(result)
	if err != nil {
		return nil, err
	}
	return &Texture{
		Header: TextureHeader{
			API:         RenderAPIType((*C.ovrTexture)(unsafe.Pointer(cTexture)).Header.API),
			TextureSize: goSizei((*C.ovrTexture)(unsafe.Pointer(cTexture)).Header.TextureSize)},
		cTexture: (*C.ovrTexture)(unsafe.Pointer(cTexture)),
		D3D11: &D3D11TextureData{
			PTexture: unsafe.Pointer((*C.ovrD3D11TextureData)(unsafe.Pointer(cTexture)).pTexture),
			PSRView:  unsafe.Pointer((*C.ovrD3D11TextureData)(unsafe.Pointer(cTexture)).pSRView)}}, nil
}
