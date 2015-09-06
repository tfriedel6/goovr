package goovr_test

import (
	"testing"
	"github.com/void6/goovr"
	"fmt"
	"time"
)

func ovrInit(t *testing.T) {
	params := goovr.InitParams{
		Flags: 0,
		RequestedMinorVersion: 0,
		LogCallback: nil,
		ConnectionTimeoutMS: 0}
	err := goovr.Initialize(&params)
	if err != nil {
		t.Fatal(err)
	}
}

func ovrShutdown() {
	goovr.Shutdown()
}

func TestInitialize(t *testing.T) {
	ovrInit(t)
	ovrShutdown()
}

func TestCreateFirstHMD(t *testing.T) {
	ovrInit(t)
	defer ovrShutdown()

	hmd, err := goovr.Hmd_Create(nil)
	if err != nil {
		t.Fatal(err)
	}
	fmt.Printf("%#v\n", hmd)
	hmd.Destroy()
}

func TestTracking(t *testing.T) {
	ovrInit(t)
	defer ovrShutdown()

	hmd, err := goovr.Hmd_Create(nil)
	if err != nil {
		t.Fatal(err)
	}
	defer hmd.Destroy()

	allCaps := goovr.TrackingCap_Orientation | goovr.TrackingCap_MagYawCorrection | goovr.TrackingCap_Position
	err = hmd.ConfigureTracking(allCaps, allCaps)
	if err != nil {
		t.Fatal(err)
	}

	time.Sleep(100 * time.Millisecond)
	state := hmd.GetTrackingState(0)
	fmt.Printf("%#v\n", state)
	time.Sleep(100 * time.Millisecond)
	state = hmd.GetTrackingState(0)
	fmt.Printf("%#v\n", state)
	time.Sleep(100 * time.Millisecond)
	state = hmd.GetTrackingState(0)
	fmt.Printf("%#v\n", state)
	time.Sleep(100 * time.Millisecond)
	state = hmd.GetTrackingState(0)
	fmt.Printf("%#v\n", state)
	time.Sleep(100 * time.Millisecond)
	state = hmd.GetTrackingState(0)
	fmt.Printf("%#v\n", state)
	time.Sleep(100 * time.Millisecond)
	state = hmd.GetTrackingState(0)
	fmt.Printf("%#v\n", state)
	time.Sleep(100 * time.Millisecond)
	state = hmd.GetTrackingState(0)
	fmt.Printf("%#v\n", state)
}

/*
func TestProperties(t *testing.T) {
	ovrInit(t)
	defer ovrShutdown()

	hmd, err := goovr.Hmd_Create(0)
	if err != nil {
		t.Fatal(err)
	}
	defer hmd.Destroy()

	err = hmd.SetBool("test_bool", true)
	if err != nil {
		t.Fatal(err)
	}
	if hmd.GetBool("test_bool", false) != true {
		t.Fatal("Incorrect boolean value")
	}

	err = hmd.SetInt("test_int", 42)
	if err != nil {
		t.Fatal(err)
	}
	if hmd.GetInt("test_int", 0) != 42 {
		t.Fatal("Incorrect int value")
	}

	err = hmd.SetFloat("test_float", 3.14)
	if err != nil {
		t.Fatal(err)
	}
	if value := hmd.GetFloat("test_float", 0); value < 3.13 || value > 3.15 {
		t.Fatal("Incorrect float value")
	}

	err = hmd.SetFloatArray("test_float_array", []float32{2.718, 3.14})
	if err != nil {
		t.Fatal(err)
	}
	if values := hmd.GetFloatArray("test_float_array", 2); values[0] < 2.717 || values[0] > 2.719 || values[1] < 3.13 || values[1] > 3.15 {
		t.Fatal("Incorrect float array value")
	}

	err = hmd.SetString("test_string", "test")
	if err != nil {
		t.Fatal(err)
	}
	if hmd.GetString("test_string", "") != "test" {
		t.Fatal("Incorrect string value")
	}
}
*/
