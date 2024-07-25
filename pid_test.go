package pid

import (
	"fmt"
	"math"
	"testing"
	"time"
)

type WaterBoiler struct {
	temp        float64
	power       float64
	volume      float64
	dissipation float64 // degrees celsius per second
	last_time   time.Time
	speedup     float64
}

func (w *WaterBoiler) GetTemperature() float64 {
	now := time.Now()
	elapsedTime := now.Sub(w.last_time)
	w.last_time = now
	w.temp += (w.power/(4186*w.volume) - w.dissipation) * elapsedTime.Seconds() * w.speedup
	w.temp = math.Min(math.Max(w.temp, 0), 100)
	return w.temp

}

func (w *WaterBoiler) SetPower(power float64) {
	w.GetTemperature() // call once to update temperature
	w.power = math.Max(0, power)
}

func TestPID(t *testing.T) {
	boiler := WaterBoiler{
		temp:        77,
		power:       0,
		volume:      1,
		dissipation: 0.01,
		last_time:   time.Now(),
		speedup:     1,
	}

	pid, err := NewPID(300, 2, 1, 1,
		func() float64 {
			waterTemp := boiler.GetTemperature()
			return waterTemp
		},
		func(f float64) {
			boiler.SetPower(f)
		}, nil)

	if err != nil {
		t.Fatal(err)
	}

	pid.EnableLog("waterboiler.log")
	pid.SetOutputLimits(0, 1800)
	stopped := pid.Start(80, 30, 1, false)
	time.AfterFunc(60*time.Second, pid.DisableLog)
	time.AfterFunc(120*time.Second, pid.Stop)

	for range stopped {
		fmt.Println("stablization reached")
	}
	fmt.Println("The PID has stopped")
}
