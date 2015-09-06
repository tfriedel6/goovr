// +build !windows

package goovr

import "unsafe"

type D3D11TextureData struct {
	PTexture unsafe.Pointer
	PSRView  unsafe.Pointer
}

func (hmd *Hmd) CreateSwapTextureSetD3D11(device unsafe.Pointer, desc unsafe.Pointer, miscFlags uint) (*SwapTextureSet, error) {
	return nil, notAvailableErr
}

func (hmd *Hmd) CreateMirrorTextureD3D11(device unsafe.Pointer, desc unsafe.Pointer, miscFlags uint) (*Texture, error) {
	return nil, notAvailableErr
}
