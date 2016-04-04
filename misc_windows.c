#include "OVR_CAPI.h"

void logCallback(int level, const char *message);

int logCallback_cgo(uintptr_t userData, int level, const char* message) {
    logCallback(level, message);
    return 0;
}
