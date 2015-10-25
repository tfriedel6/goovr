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
	case C.ovrError_InvalidSession:
		return errors.New("ovrError_InvalidSession: Invalid ovrSession parameter provided")
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
	case C.ovrError_AudioDeviceNotFound:
		return errors.New("ovrError_AudioDeviceNotFound: Failure to find the specified audio device")
	case C.ovrError_AudioComError:
		return errors.New("ovrError_AudioComError: Generic COM error")
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
	case C.ovrError_MismatchedAdapters:
		return errors.New("ovrError_MismatchedAdapters: Chosen rendering adapters between client and service do not match")
	case C.ovrError_LeakingResources:
		return errors.New("ovrError_LeakingResources: Calling application has leaked resources")
	case C.ovrError_ClientVersion:
		return errors.New("ovrError_ClientVersion: Client version too old to connect to service")
	case C.ovrError_OutOfDateOS:
		return errors.New("ovrError_OutOfDateOS: The operating system is out of date")
	case C.ovrError_OutOfDateGfxDriver:
		return errors.New("ovrError_OutOfDateGfxDriver: The graphics driver is out of date")
	case C.ovrError_IncompatibleGPU:
		return errors.New("ovrError_IncompatibleGPU: The graphics hardware is not supported")
	case C.ovrError_NoValidVRDisplaySystem:
		return errors.New("ovrError_NoValidVRDisplaySystem: No valid VR display system found")
	case C.ovrError_InvalidBundleAdjustment:
		return errors.New("ovrError_InvalidBundleAdjustment: Headset has no bundle adjustment data")
	case C.ovrError_USBBandwidth:
		return errors.New("ovrError_USBBandwidth: The USB hub cannot handle the camera frame bandwidth")
	case C.ovrError_USBEnumeratedSpeed:
		return errors.New("ovrError_USBEnumeratedSpeed: The USB camera is not enumerating at the correct device speed")
	case C.ovrError_ImageSensorCommError:
		return errors.New("ovrError_ImageSensorCommError: Unable to communicate with the image sensor")
	case C.ovrError_GeneralTrackerFailure:
		return errors.New("ovrError_GeneralTrackerFailure: GeneralTrackerFailure. We use this to report various tracker issues that don't fit in an easily classifiable bucket")
	case C.ovrError_ExcessiveFrameTruncation:
		return errors.New("ovrError_ExcessiveFrameTruncation: A more than acceptable number of frames are coming back truncated")
	case C.ovrError_ExcessiveFrameSkipping:
		return errors.New("ovrError_ExcessiveFrameSkipping: A more than acceptable number of frames have been skipped")
	case C.ovrError_SyncDisconnected:
		return errors.New("ovrError_SyncDisconnected: The tracker is not receiving the sync signal (cable disconnected?)")
	case C.ovrError_TrackerMemoryReadFailure:
		return errors.New("ovrError_TrackerMemoryReadFailure: Failed to read memory from the tracker")
	case C.ovrError_TrackerMemoryWriteFailure:
		return errors.New("ovrError_TrackerMemoryWriteFailure: Failed to write memory from the tracker")
	case C.ovrError_TrackerFrameTimeout:
		return errors.New("ovrError_TrackerFrameTimeout: Timed out waiting for a camera frame")
	case C.ovrError_TrackerTruncatedFrame:
		return errors.New("ovrError_TrackerTruncatedFrame: Truncated frame returned from tracker")
	case C.ovrError_HMDFirmwareMismatch:
		return errors.New("ovrError_HMDFirmwareMismatch: The HMD Firmware is out of date and is unacceptable")
	case C.ovrError_TrackerFirmwareMismatch:
		return errors.New("ovrError_TrackerFirmwareMismatch: The Tracker Firmware is out of date and is unacceptable")
	case C.ovrError_BootloaderDeviceDetected:
		return errors.New("ovrError_BootloaderDeviceDetected: A bootloader HMD is detected by the service")
	case C.ovrError_TrackerCalibrationError:
		return errors.New("ovrError_TrackerCalibrationError: The tracker calibration is missing or incorrect")
	case C.ovrError_ControllerFirmwareMismatch:
		return errors.New("ovrError_ControllerFirmwareMismatch: The controller firmware is out of date and is unacceptable")
	case C.ovrError_Incomplete:
		return errors.New("ovrError_Incomplete: Requested async work not yet complete")
	case C.ovrError_Abandoned:
		return errors.New("ovrError_Abandoned: Requested async work was abandoned and result is incomplete")
	case C.ovrError_DisplayLost:
		return errors.New("ovrError_DisplayLost: In the event of a system-wide graphics reset or cable unplug this is returned to the app")
	case C.ovrError_RuntimeException:
		return errors.New("ovrError_RuntimeException: A runtime exception occurred. The application is required to shutdown LibOVR and re-initialize it before this error state will be cleared")
	}
	return fmt.Errorf("ovr error with unknown error code %d", result)
}
