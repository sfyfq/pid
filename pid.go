// This PID controller package took inspiration from the following modules
// simple-pid (https://pypi.org/project/simple-pid/)

package pid

import (
	"errors"
	"fmt"
	"log"
	"math"
	"os"
	"sync"
	"time"
)

type OutputLimit struct {
	lower, upper float64
}
type LSArray struct {
	array []float64
	head  int
}

func NewLSArray(size int) *LSArray {

	array := &LSArray{
		array: make([]float64, size),
		head:  -1,
	}
	array.Reset()

	return array
}

func (l *LSArray) Resize(size int) {
	if size > len(l.array) {
		l.array = make([]float64, size)
		fmt.Printf("array resized to %d", size)
	}
	l.Reset()
}

func (l *LSArray) Reset() {
	for i := range l.array {
		l.array[i] = math.Inf(1)
	}
}

func (l *LSArray) AddValue(v float64) {
	l.head += 1
	if l.head == len(l.array) {
		l.head = 0
	}
	l.array[l.head] = v
}

func (l *LSArray) Average() float64 {
	sum := 0.0
	for _, v := range l.array {
		sum += v
	}
	return sum / float64(len(l.array))
}

type pidLogging struct {
	logFilename string
	logger      *log.Logger
	logFile     *os.File
}

// PID represents a simple PID controller
type PID struct {
	kp, ki, kd                          float64
	setpoint                            float64
	limits                              OutputLimit
	last_time                           time.Time
	interval_s                          float64
	ticker                              time.Ticker
	sample_func                         func() float64
	action_func                         func(float64)
	error_map                           func(float64) float64
	proportional, integral, derivative  float64
	last_output, last_error, last_input float64
	cancel                              chan struct{}
	movingAverage                       LSArray
	done                                chan struct{}
	startTime                           time.Time
	mu                                  sync.Mutex

	pidLogging
}

// NewPID initialize a new PID controller

var OUTPUT_LIMIT_NONE = math.MaxFloat64

func NewPID(
	Kp, Ki, Kd float64,
	interval_s float64,
	sample_func func() float64,
	action_func func(float64),
	error_map func(float64) float64,

) (*PID, error) {
	if sample_func == nil || action_func == nil {
		return nil, errors.New("both sample_func and action_func must be provided")
	}
	pid := &PID{
		kp:         Kp,
		ki:         Ki,
		kd:         Kd,
		interval_s: interval_s,
		limits: OutputLimit{
			lower: -OUTPUT_LIMIT_NONE,
			upper: OUTPUT_LIMIT_NONE},
		sample_func:   sample_func,
		action_func:   action_func,
		error_map:     error_map,
		movingAverage: *NewLSArray(10),
		ticker:        *time.NewTicker(time.Duration(float64(time.Second) * interval_s)),
	}

	pid.ticker.Stop()
	pid.Reset()
	pid.integral = clamp(pid.integral, pid.limits)
	return pid, nil
}

func (p *PID) EnableLog(logFilename string) error {
	logFile, err := os.OpenFile(logFilename, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		return err
	}
	defer logFile.Close()
	p.mu.Lock()
	defer p.mu.Unlock()
	p.logFilename = logFilename
	return nil

}

func (p *PID) DisableLog() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.logFilename = ""
}

// SetOutputLimits sets lower and upper bounds on the control variable.
// To leave one limit unchanged, set it to pid.OUTPUT_LIMIT_NONE
func (p *PID) SetOutputLimits(lower, upper float64) {
	if math.IsNaN(lower) || math.IsInf(lower, 0) {
		panic("the lower limit is NaN or Inf")
	}
	if math.IsNaN(upper) || math.IsInf(upper, 0) {
		panic("the upper limit is NaN or Inf")
	}
	if lower > upper {
		panic("the lower limit cannot be greater than the upper limit")
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if lower != OUTPUT_LIMIT_NONE {
		p.limits.lower = lower
		if p.logger != nil {
			p.logger.Printf("the lower output limit has been set to %f", lower)
		}
	}

	if upper != OUTPUT_LIMIT_NONE {
		p.limits.upper = upper
		if p.logger != nil {
			p.logger.Printf("the upper output limit has been set to %f", upper)
		}
	}

}

// Starts the PID loop control for `stableForSeconds` seconds and stops the loop
// automatically when the moving average error within the last `stableForSeconds` seconds is no greater than absTol. An optional timeout can be supplied to stop the PID after a certain time period.
// It returns a channel that is closed when the PID is stopped.
func (p *PID) Start(setpoint float64, stableForSeconds float64, absTol float64, timeout ...time.Duration) <-chan struct{} {
	p.Stop() // make sure at most one active PID control loop at all time.
	if p.logFilename != "" {
		logFile, err := os.OpenFile(p.logFilename, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
		if err == nil {
			p.logger = log.New(logFile, "", 0)
		}
	}

	if p.logger != nil {
		p.logger.Printf("New PID Session")
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	p.setpoint = setpoint
	if p.logger != nil {
		p.logger.Printf("Setpoint set to %f", setpoint)
	}
	p.cancel = make(chan struct{})
	p.done = make(chan struct{})
	size := int(math.Min(1024, math.Max(1, math.Ceil(stableForSeconds/p.interval_s))))
	p.movingAverage.Resize(size)

	// set up timeout timer if specified and restart the sampling ticker
	var timeoutTimer *time.Timer
	var timeoutChan <-chan time.Time
	if len(timeout) > 0 {
		timeoutTimer = time.NewTimer(timeout[0])
		timeoutChan = timeoutTimer.C
	}
	p.ticker.Reset(time.Duration(float64(time.Second) * p.interval_s))

	go func() {
		defer func() {
			if p.logger != nil {
				elapsed := time.Since(p.startTime).Seconds()
				p.logger.Printf("the PID loop has stopped and took %f seconds", elapsed)
				p.logFile.Close()
				p.logFile = nil
			}
			if timeoutTimer != nil && !timeoutTimer.Stop() {
				<-timeoutTimer.C
			}
			p.ticker.Stop()
			close(p.done)
		}()
		if p.logger != nil {
			p.logger.Printf("PID regulation started...")
			p.logger.Printf("time,sensedValue,outputValue,Kp,Ki,Kd,proportional,integral,derivative")
		}
		p.startTime = time.Now()
		for {
			select {
			case <-timeoutChan:
				return
			case now := <-p.ticker.C:
				newValue := p.sample_func()
				deltaTime := now.Sub(p.last_time)
				elapsedTime := now.Sub(p.startTime)
				p.last_time = now
				p.last_input = newValue
				newControlValue := p.compute(newValue, deltaTime)
				p.action_func(newControlValue)
				p.movingAverage.AddValue(math.Abs(newValue - p.setpoint))
				if p.logger != nil {
					p.logger.Println(elapsedTime.Seconds(), newValue, newControlValue, p.kp, p.ki, p.kd, p.proportional, p.integral, p.derivative)
				}
				if p.movingAverage.Average() <= math.Abs(absTol) {
					return
				}
			case <-p.cancel:
				return
			}
		}
	}()
	return p.done
}

// Stops the PID loop control
func (p *PID) Stop() {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.cancel != nil {
		close(p.cancel)
		p.cancel = nil
		if p.logger != nil {
			p.logger.Printf("the PID loop was canceled")
		}
	}
	if p.done != nil {
		<-p.done // wait for the goroutine to complete
	}

}

// GetTuning returns the tuning constants
func (p *PID) GetTuning() (Kp, Ki, Kd float64) {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.kp, p.ki, p.kd
}

// SetTuning sets the tuning constants. It panics if any one of them is negative
func (p *PID) SetTuning(Kp, Ki, Kd float64) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if Kp < 0 {
		panic("Kp is negative")
	}
	if Ki < 0 {
		panic("Ki is negative")
	}
	if Kd < 0 {
		panic("Kd is negative")
	}

	p.kp, p.ki, p.kd = Kp, Ki, Kd
}

// compute calculates internal states based on the new input value and the elapsed time since the sample point
func (p *PID) compute(newValue float64, deltaTime time.Duration) float64 {
	p.mu.Lock()
	defer p.mu.Unlock()

	dt := deltaTime.Seconds()
	dt = math.Max(dt, 1e-16)
	// compute error terms
	e := p.setpoint - newValue
	var d_error float64

	if math.IsNaN(p.last_error) {
		d_error = e
	} else {
		d_error = e - p.last_error
	}

	// apply error transformation if needed
	if p.error_map != nil {
		d_error = p.error_map(d_error)
	}

	// compute the proportional term
	p.proportional = p.kp * e

	// compute the integral and derivative terms
	p.integral += p.ki * e * dt
	p.integral = clamp(p.integral, p.limits)

	p.derivative = p.kd * d_error / dt

	controlValue := p.proportional + p.integral + p.derivative
	controlValue = clamp(controlValue, p.limits)

	p.last_output = controlValue
	p.last_error = e
	return controlValue

}

// Reset initializes the internal states of the PID
func (p *PID) Reset() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.proportional = 0
	p.integral = 0
	p.derivative = 0
	p.last_output = math.NaN()
	p.last_error = math.NaN()
	p.last_input = math.NaN()
	p.last_time = time.Now()
}

// clamp applies limits on the output value
func clamp(in float64, limits OutputLimit) (out float64) {
	if math.IsNaN(in) {
		return math.NaN()
	}
	if !math.IsNaN(limits.upper) && in > limits.upper {
		return limits.upper
	}
	if !math.IsNaN(limits.lower) && in < limits.lower {
		return limits.lower
	}
	return in
}
