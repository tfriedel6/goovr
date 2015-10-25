// +build !windows

package goovr

type GLTextureData struct {
	TexId int
}

func (hmd *Session) CreateSwapTextureSetGL(format uint, width, height int) (*SwapTextureSet, error) {
	return nil, notAvailableErr
}

func (hmd *Session) CreateMirrorTextureGL(format uint, width, height int) (*Texture, error) {
	return nil, notAvailableErr
}
