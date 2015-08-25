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
// \param[in]  hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in]  device Specifies the associated ID3D11Device, which must be the one that the textures will be used with in the application's process.
// \param[in]  desc Specifies requested texture properties.
// \param[out] outTextureSet Specifies the created ovrSwapTextureSet, which will be valid only upon a successful return value.
//             This texture set must be eventually destroyed via ovrHmd_DestroySwapTextureSet before destroying the HMD with ovrHmd_Destroy.
//
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information.
//
// \see ovrHmd_DestroySwapTextureSet
func (hmd *Hmd) CreateSwapTextureSetD3D11(device unsafe.Pointer, desc unsafe.Pointer) (*SwapTextureSet, error) {
	var cSet *C.ovrSwapTextureSet
	result := C.ovrHmd_CreateSwapTextureSetD3D11(hmd.cHmd, (*C.ID3D11Device)(device), (*C.D3D11_TEXTURE2D_DESC)(desc), &cSet)
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
// \param[in]  hmd Specifies an ovrHmd previously returned by ovrHmd_Create.
// \param[in]  device Specifies the associated ID3D11Device, which must be the one that the textures will be used with in the application's process.
// \param[in]  desc Specifies requested texture properties.
// \param[out] outMirrorTexture Specifies the created ovrTexture, which will be valid only upon a successful return value.
//             This texture must be eventually destroyed via ovrHmd_DestroyMirrorTexture before destroying the HMD with ovrHmd_Destroy.
//
// \return Returns an ovrResult indicating success or failure. In the case of failure, use
//         ovr_GetLastErrorInfo to get more information.
//
// \see ovrHmd_DestroyMirrorTexture
func (hmd *Hmd) CreateMirrorTextureD3D11(device unsafe.Pointer, desc unsafe.Pointer) (*Texture, error) {
	var cTexture *C.union_ovrD3D11Texture
	result := C.ovrHmd_CreateMirrorTextureD3D11(hmd.cHmd, (*C.ID3D11Device)(device), (*C.D3D11_TEXTURE2D_DESC)(desc), (**C.ovrTexture)(unsafe.Pointer(&cTexture)))
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
