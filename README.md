# A Simple PID Controller Written in GO

## Overview

I created this PID package because I needed to tune a PID controller for a heating stage that triggers an integrating sphere to take a measurement when the monitored temperature reaches the target and is stabilized for a set duration. 
It is modeled on the basic PID controller with output and integral clamping. It comes with a integrated stabilization detection using moving average. It also supports logging for parameter training and diagnostic purposes.

Rather than having the external process call the PID to update, which is the approach adopted by many packages, this PID controller has an internal ticker that calls two function periodically, one to get the input from the sensor and the other to apply the output to the instrument. 

## Features

- Basic PID controller model that just works
- Stabilization detection using moving average of absolute error
- Output and integral clamping
- Logging for parameter training
- Diagnostic logging

## Installation

To get this package, run
```shell
    go get "github.com/sfyfq/pid"
```

## How to Use
Using a PID for a waterboiler as an example.

Three function calls are needed to create and run a PID controller.

First, create a new PID controller instance by calling `NewPID(Kp, Kd, Ki, intervalSecond, sampleFunc, actionFunc, errorMap)`

```go
pid,err:= NewPID(
	100, 0.2, 0.1,                      // set Kp, Ki, Kd to 100, 0.2 and 0.1 respectively
	1 ,                                 // sample once per second
	boiler.GetTemperature(),            // function to get the input variable
	boiler.SetPower(f),                 // function to set the output variable
	nil,                                // optional function to transform error value, set to nil in this example
)
```

Next, set the output limits by calling `SetOutputLimits(lower, upper)` on the pid instance


```go
pid.SetOutputLimits(0, 1800)            // limit the output power of the boiler to [0, 1800]
```

Start the PID controller by calling `Start(setpoint, stableForSeconds, absTol, oneshot)`. An optional timeout (time.Duration) can be supplied to limit the duration of the PID loop.
The function returns a readonly channel, which is closed when the loop finishes and can be waited on to keep the main app running.

```go
trigger := pid.Start(80, 30, 1, false)
time.AfterFunc(120*time.Second, pid.Stop)
for range trigger {
    fmt.Println("stablization reached")
}
fmt.Println("The PID has stopped")

```


## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

Special thanks to the developers of existing PID libraries that provided inspirations in creating this package.