# goovr
Go bindings for the Oculus SDK 1.3.0

No external libraries are required for this to compile, all code is included and builds with CGO.
Under platforms other than windows the goovr.Inititalize and goovr.Create functions return an error,
and none of the functions actually do anything.
