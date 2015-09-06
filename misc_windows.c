#include "OVR_CAPI_0_7_0.h"

void logCallback(int level, const char *message);

int logCallback_cgo(uintptr_t userData, int level, const char* message) {
    logCallback(level, message);
    return 0;
}

ovrTexture* getTexturePointerFromArray(ovrTexture* textures, int index) {
    return &textures[index];
}