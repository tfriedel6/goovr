// +build !windows

package goovr

type GLTextureData struct {
	TexId int
}

func (hmd *Hmd) CreateSwapTextureSetGL(format uint, width, height int) (*SwapTextureSet, error) {
	return nil, notAvailableErr
}

func (hmd *Hmd) CreateMirrorTextureGL(format uint, width, height int) (*Texture, error) {
	return nil, notAvailableErr
}
