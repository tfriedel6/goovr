#include "OVR_CAPI_0_6_0.h"

void logCallback(int level, const char *message);

int logCallback_cgo(int level, const char* message) {
    logCallback(level, message);
    return 0;
}

ovrTexture* getTexturePointerFromArray(ovrTexture* textures, int index) {
    return &textures[index];
}