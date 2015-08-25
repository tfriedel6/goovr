package goovr

// #include "OVR_ErrorCode.h"
import "C"
import (
	"errors"
	"fmt"
)

func errorForResult(result C.ovrResult) error {
	if result >= 0 {
		return nil
	}
	switch result {
	case C.ovrError_MemoryAllocationFailure:
		return errors.New("ovrError_MemoryAllocationFailure: Failure to allocate memory")
	case C.ovrError_SocketCreationFailure:
		return errors.New("ovrError_SocketCreationFailure: Failure to create a socket")
	case C.ovrError_InvalidHmd:
		return errors.New("ovrError_InvalidHmd: Invalid HMD parameter provided")
	case C.ovrError_Timeout:
		return errors.New("ovrError_Timeout: The operation timed out")
	case C.ovrError_NotInitialized:
		return errors.New("ovrError_NotInitialized: The system or component has not been initialized")
	case C.ovrError_InvalidParameter:
		return errors.New("ovrError_InvalidParameter: Invalid parameter provided. See error info or log for details")
	case C.ovrError_ServiceError:
		return errors.New("ovrError_ServiceError: Generic service error. See error info or log for details")
	case C.ovrError_NoHmd:
		return errors.New("ovrError_NoHmd: The given HMD doesn't exist")
	case C.ovrError_Initialize:
		return errors.New("ovrError_Initialize: Generic initialization error")
	case C.ovrError_LibLoad:
		return errors.New("ovrError_LibLoad: Couldn't load LibOVRRT")
	case C.ovrError_LibVersion:
		return errors.New("ovrError_LibVersion: LibOVRRT version incompatibility")
	case C.ovrError_ServiceConnection:
		return errors.New("ovrError_ServiceConnection: Couldn't connect to the OVR Service")
	case C.ovrError_ServiceVersion:
		return errors.New("ovrError_ServiceVersion: OVR Service version incompatibility")
	case C.ovrError_IncompatibleOS:
		return errors.New("ovrError_IncompatibleOS: The operating system version is incompatible")
	case C.ovrError_DisplayInit:
		return errors.New("ovrError_DisplayInit: Unable to initialize the HMD display")
	case C.ovrError_ServerStart:
		return errors.New("ovrError_ServerStart: Unable to start the server. Is it already running")
	case C.ovrError_Reinitialization:
		return errors.New("ovrError_Reinitialization: Attempting to re-initialize with a different version")
	case C.ovrError_InvalidBundleAdjustment:
		return errors.New("ovrError_InvalidBundleAdjustment: Headset has no bundle adjustment data")
	case C.ovrError_USBBandwidth:
		return errors.New("ovrError_USBBandwidth: The USB hub cannot handle the camera frame bandwidth")
	}
	return fmt.Errorf("ovr error with unknown error code %d", result)
}
