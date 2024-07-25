// This PID controller package took inspiration from the following modules
// simple-pid (https://pypi.org/project/simple-pid/)

package pid

import (
	"errors"
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

func (l *LSArray) Mean() float64 {
	sum := 0.0
	for _, v := range l.array {
		sum += v
	}
	return sum / float64(len(l.array))
}

func (l *LSArray) Var() float64 {
	variance := 0.0
	mean := l.Mean()
	for _, value := range l.array {
		variance += (value - mean) * (value - mean)
	}
	return variance / float64(len(l.array))
}

type pidTarget struct {
	movingAverage *LSArray
	setpoint      float64
	absTol        float64
	errorReversed bool
	stabilized    bool
}

type pidLogging struct {
	logFilename string
	logger      *log.Logger
	logFile     *os.File
}

// PID represents a simple PID controller
type PID struct {
	pidLogging
	pidTarget
	limits                             OutputLimit
	lastTime                           time.Time
	interval                           time.Duration
	ticker                             time.Ticker
	sampleFunc                         func() float64
	actionFunc                         func(float64)
	errorMap                           func(float64) float64
	kp, ki, kd                         float64
	proportional, integral, derivative float64
	lastOutput, lastError, lastInput   float64
	cancel                             chan struct{}
	trigger                            chan struct{}
	startTime                          time.Time
	mu                                 sync.Mutex
}

var OUTPUT_LIMIT_NONE = math.MaxFloat64

// NewPID initialize a new PID controller
func NewPID(
	Kp, Ki, Kd float64,
	intervalSecond float64,
	sampleFunc func() float64,
	actionFunc func(float64),
	errorMap func(float64) float64,

) (*PID, error) {
	if sampleFunc == nil || actionFunc == nil {
		return nil, errors.New("both sample_func and action_func must be provided")
	}
	pid := &PID{
		kp:       Kp,
		ki:       Ki,
		kd:       Kd,
		interval: time.Duration(float64(time.Second) * intervalSecond),
		limits: OutputLimit{
			lower: -OUTPUT_LIMIT_NONE,
			upper: OUTPUT_LIMIT_NONE},
		sampleFunc: sampleFunc,
		actionFunc: actionFunc,
		errorMap:   errorMap,
		ticker:     *time.NewTicker(time.Duration(float64(time.Second) * intervalSecond)),
		pidTarget: pidTarget{
			movingAverage: NewLSArray(10),
		},
	}

	pid.ticker.Stop()
	pid.Reset()
	pid.integral = clamp(pid.integral, pid.limits)
	return pid, nil
}

// EnableLog enables logging to a file. Logging is started automatically by Start() and stops when the loop finishes.
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

// DisableLog disables file logging.
func (p *PID) DisableLog() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.logFilename = ""
	if p.logFile != nil {
		p.logFile.Close()
		p.logFile = nil
	}
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
func (p *PID) Start(setpoint float64, stableForSeconds float64, absTol float64, oneshot bool, timeout ...time.Duration) (trigger <-chan struct{}) {
	p.Stop() // make sure at most one active PID control loop at all time.
	if p.logFilename != "" {
		logFile, err := os.OpenFile(p.logFilename, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
		if err == nil {
			p.logger = log.New(logFile, "", 0)
			p.logFile = logFile
		}
	}

	if p.logger != nil {
		p.logger.Printf("New PID Session")
	}
	p.mu.Lock()
	defer p.mu.Unlock()

	p.setpoint = setpoint
	p.absTol = absTol
	p.stabilized = false
	p.errorReversed = false

	p.cancel = make(chan struct{}, 1)
	p.trigger = make(chan struct{}, 1)
	size := int(math.Min(1024, math.Max(1, math.Ceil(stableForSeconds/p.interval.Seconds()))))
	p.movingAverage.Resize(size)

	// set up timeout timer if specified and restart the sampling ticker
	var timeoutTimer *time.Timer
	var timeoutChan <-chan time.Time
	if len(timeout) > 0 {
		timeoutTimer = time.NewTimer(timeout[0])
		timeoutChan = timeoutTimer.C
	}
	p.ticker.Reset(p.interval)

	go func() {
		defer func() {
			if p.logger != nil {
				elapsed := time.Since(p.startTime).Seconds()
				p.logger.Printf("the PID loop has stopped and took %f seconds", elapsed)
				p.logFile.Close()
				p.logFile = nil
				p.logger = nil
			}
			if timeoutTimer != nil && !timeoutTimer.Stop() {
				<-timeoutTimer.C
			}
			p.ticker.Stop()
			close(p.trigger)
			p.cancel = nil
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
				newValue := p.sampleFunc()
				deltaTime := now.Sub(p.lastTime)
				elapsedTime := now.Sub(p.startTime)
				p.lastTime = now
				p.lastInput = newValue
				newControlValue := p.compute(newValue, deltaTime)
				p.actionFunc(newControlValue)
				p.movingAverage.AddValue(math.Abs(newValue - p.setpoint))
				if p.logger != nil {
					p.logger.Printf("%f,%f,%f,%f,%f,%f,%f,%f,%f", elapsedTime.Seconds(), newValue, newControlValue, p.kp, p.ki, p.kd, p.proportional, p.integral, p.derivative)
				}
				if p.isStabilzied() {
					if oneshot {
						return
					} else if !p.stabilized {
						p.stabilized = true
						select {
						case p.trigger <- struct{}{}:
						default:
						}
					}
				} else {
					p.stabilized = false
				}
			case <-p.cancel:
				return
			}
		}
	}()
	return p.trigger
}

// Stops the PID loop control
func (p *PID) Stop() {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.cancel != nil {
		close(p.cancel)
		if p.logger != nil {
			p.logger.Printf("received signal to cancel the PID loop")
		}
	}
}

// GetSetpoint returns the current setpoint
func (p *PID) GetSetpoint() float64 {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.setpoint
}

// SetSetpoint sets a new setpoint
func (p *PID) SetSetpoint(newValue float64) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.logger != nil {
		p.logger.Printf("new setpoint %f", newValue)
	}
	p.setpoint = newValue
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
	if p.logger != nil {
		p.logger.Printf("new tuning parameters Kp=%f, Ki=%f, Kd=%f", Kp, Ki, Kd)
	}
}

// compute calculates internal states based on the new input value and the elapsed time since the sample point
func (p *PID) compute(newValue float64, deltaTime time.Duration) float64 {
	p.mu.Lock()
	defer p.mu.Unlock()

	dt := deltaTime.Seconds()
	dt = math.Max(dt, 1e-16)
	// compute error terms
	e := p.setpoint - newValue
	if math.Signbit(e) != math.Signbit(p.lastError) {
		p.errorReversed = true
	}
	var d_error float64

	if math.IsNaN(p.lastError) {
		d_error = e
	} else {
		d_error = e - p.lastError

	}

	// apply error transformation if needed
	if p.errorMap != nil {
		d_error = p.errorMap(d_error)
	}

	// compute the proportional term
	p.proportional = p.kp * e

	// compute the integral and derivative terms
	p.integral += p.ki * e * dt
	p.integral = clamp(p.integral, p.limits)

	p.derivative = p.kd * d_error / dt

	controlValue := p.proportional + p.integral + p.derivative
	controlValue = clamp(controlValue, p.limits)

	p.lastOutput = controlValue
	p.lastError = e
	return controlValue

}

// Reset initializes the internal states of the PID
func (p *PID) Reset() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.proportional = 0
	p.integral = 0
	p.derivative = 0
	p.lastOutput = math.NaN()
	p.lastError = math.NaN()
	p.lastInput = math.NaN()
	p.lastTime = time.Now()
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

// isStabilzied encapsulate the algorithm to determine whether the input variable has stabilized.
func (p *PID) isStabilzied() bool {
	if p.movingAverage.Mean() <= math.Abs(p.absTol) {
		if !p.errorReversed {
			return false
		} else {

			return true
		}

	} else {
		p.errorReversed = false
		return false
	}

}
