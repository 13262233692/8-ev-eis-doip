package signal

import (
	"math"
	"sync"
)

type AdaptiveBilinearKalman struct {
	mu               sync.RWMutex
	x1               float64
	x2               float64
	p11              float64
	p12              float64
	p21              float64
	p22              float64
	q1               float64
	q2               float64
	r                float64
	processNoiseAdapt float64
	measNoiseAdapt    float64
	innovation       float64
	innovationCov    float64
	alpha            float64
	beta             float64
	initialized      bool
	windowSize       int
	residualBuffer   []float64
	residualIdx      int
	residualCount    int
}

type KalmanConfig struct {
	InitialState1     float64
	InitialState2     float64
	InitialCov11      float64
	InitialCov12      float64
	InitialCov22      float64
	ProcessNoise1     float64
	ProcessNoise2     float64
	MeasurementNoise  float64
	Alpha             float64
	Beta              float64
	WindowSize        int
}

func DefaultKalmanConfig() KalmanConfig {
	return KalmanConfig{
		InitialState1:    0.0,
		InitialState2:    0.0,
		InitialCov11:     1.0,
		InitialCov12:     0.0,
		InitialCov22:     1.0,
		ProcessNoise1:    1e-4,
		ProcessNoise2:    1e-6,
		MeasurementNoise: 1e-2,
		Alpha:            0.95,
		Beta:             0.1,
		WindowSize:       50,
	}
}

func NewAdaptiveBilinearKalman(cfg KalmanConfig) *AdaptiveBilinearKalman {
	if cfg.WindowSize <= 0 {
		cfg.WindowSize = 50
	}
	if cfg.Alpha <= 0 || cfg.Alpha >= 1 {
		cfg.Alpha = 0.95
	}
	return &AdaptiveBilinearKalman{
		x1:                cfg.InitialState1,
		x2:                cfg.InitialState2,
		p11:               cfg.InitialCov11,
		p12:               cfg.InitialCov12,
		p21:               cfg.InitialCov12,
		p22:               cfg.InitialCov22,
		q1:                cfg.ProcessNoise1,
		q2:                cfg.ProcessNoise2,
		r:                 cfg.MeasurementNoise,
		processNoiseAdapt: cfg.ProcessNoise1,
		measNoiseAdapt:    cfg.MeasurementNoise,
		alpha:             cfg.Alpha,
		beta:              cfg.Beta,
		windowSize:        cfg.WindowSize,
		residualBuffer:    make([]float64, cfg.WindowSize),
	}
}

func (k *AdaptiveBilinearKalman) Reset() {
	k.mu.Lock()
	defer k.mu.Unlock()
	k.x1 = 0.0
	k.x2 = 0.0
	k.p11 = 1.0
	k.p12 = 0.0
	k.p21 = 0.0
	k.p22 = 1.0
	k.initialized = false
	k.residualIdx = 0
	k.residualCount = 0
	for i := range k.residualBuffer {
		k.residualBuffer[i] = 0
	}
}

func (k *AdaptiveBilinearKalman) Filter(measurement float64) float64 {
	k.mu.Lock()
	defer k.mu.Unlock()

	if !k.initialized {
		k.x1 = measurement
		k.x2 = 0.0
		k.initialized = true
		return measurement
	}

	dt := 1.0
	x1Pred := k.x1 + dt*k.x2
	x2Pred := k.x2

	p11Pred := k.p11 + dt*(k.p12+k.p21) + dt*dt*k.p22 + k.q1
	p12Pred := k.p12 + dt*k.p22
	p21Pred := p12Pred
	p22Pred := k.p22 + k.q2

	h1 := 1.0
	h2 := 0.0
	innovationCov := h1*p11Pred*h1 + h1*p12Pred*h2 + h2*p21Pred*h1 + h2*p22Pred*h2 + k.measNoiseAdapt
	innovation := measurement - (h1*x1Pred + h2*x2Pred)

	k.addResidual(innovation)
	k.adaptNoise(innovation, innovationCov)

	k1 := (p11Pred*h1 + p12Pred*h2) / innovationCov
	k2 := (p21Pred*h1 + p22Pred*h2) / innovationCov

	k.x1 = x1Pred + k1*innovation
	k.x2 = x2Pred + k2*innovation

	kh1 := 1.0 - k1*h1
	kh2 := -k1 * h2
	kh3 := -k2 * h1
	kh4 := 1.0 - k2*h2

	k.p11 = kh1*p11Pred + kh2*p21Pred
	k.p12 = kh1*p12Pred + kh2*p22Pred
	k.p21 = kh3*p11Pred + kh4*p21Pred
	k.p22 = kh3*p12Pred + kh4*p22Pred

	k.innovation = innovation
	k.innovationCov = innovationCov

	return k.x1
}

func (k *AdaptiveBilinearKalman) addResidual(r float64) {
	k.residualBuffer[k.residualIdx] = r
	k.residualIdx = (k.residualIdx + 1) % k.windowSize
	if k.residualCount < k.windowSize {
		k.residualCount++
	}
}

func (k *AdaptiveBilinearKalman) adaptNoise(innovation, innovCov float64) {
	if k.residualCount < 2 {
		return
	}

	var mean, variance float64
	n := k.residualCount
	for i := 0; i < n; i++ {
		mean += k.residualBuffer[i]
	}
	mean /= float64(n)
	for i := 0; i < n; i++ {
		d := k.residualBuffer[i] - mean
		variance += d * d
	}
	variance /= float64(n - 1)

	targetR := math.Max(variance, 1e-8)
	k.measNoiseAdapt = k.alpha*k.measNoiseAdapt + (1.0-k.alpha)*targetR

	innovRatio := innovation * innovation / math.Max(innovCov, 1e-8)
	if innovRatio > 1.0+2.0*k.beta {
		k.q1 = math.Min(k.q1*1.01, 1e-1)
		k.q2 = math.Min(k.q2*1.01, 1e-3)
	} else if innovRatio < 1.0-2.0*k.beta {
		k.q1 = math.Max(k.q1*0.99, 1e-8)
		k.q2 = math.Max(k.q2*0.99, 1e-10)
	}
}

func (k *AdaptiveBilinearKalman) State() (level, slope float64) {
	k.mu.RLock()
	defer k.mu.RUnlock()
	return k.x1, k.x2
}

func (k *AdaptiveBilinearKalman) Covariance() (p11, p12, p22 float64) {
	k.mu.RLock()
	defer k.mu.RUnlock()
	return k.p11, k.p12, k.p22
}

func (k *AdaptiveBilinearKalman) InnovationStats() (innovation, cov, adaptiveR float64) {
	k.mu.RLock()
	defer k.mu.RUnlock()
	return k.innovation, k.innovationCov, k.measNoiseAdapt
}

type DualChannelKalman struct {
	voltageFilter *AdaptiveBilinearKalman
	currentFilter *AdaptiveBilinearKalman
}

func NewDualChannelKalman(cfg KalmanConfig) *DualChannelKalman {
	return &DualChannelKalman{
		voltageFilter: NewAdaptiveBilinearKalman(cfg),
		currentFilter: NewAdaptiveBilinearKalman(cfg),
	}
}

func (d *DualChannelKalman) Filter(voltage, current float64) (float64, float64) {
	v := d.voltageFilter.Filter(voltage)
	i := d.currentFilter.Filter(current)
	return v, i
}

func (d *DualChannelKalman) Reset() {
	d.voltageFilter.Reset()
	d.currentFilter.Reset()
}

func (d *DualChannelKalman) VoltageState() (level, slope float64) {
	return d.voltageFilter.State()
}

func (d *DualChannelKalman) CurrentState() (level, slope float64) {
	return d.currentFilter.State()
}

type ComplexKalman struct {
	mu         sync.RWMutex
	xReal      float64
	xImag      float64
	pReal      float64
	pImag      float64
	pCross     float64
	qReal      float64
	qImag      float64
	rReal      float64
	rImag      float64
	alpha      float64
	initialized bool
}

func NewComplexKalman(alpha float64) *ComplexKalman {
	if alpha <= 0 || alpha >= 1 {
		alpha = 0.9
	}
	return &ComplexKalman{
		pReal: 1.0,
		pImag: 1.0,
		pCross: 0.0,
		qReal: 1e-5,
		qImag: 1e-5,
		rReal: 1e-2,
		rImag: 1e-2,
		alpha: alpha,
	}
}

func (c *ComplexKalman) Filter(real, imag float64) (float64, float64) {
	c.mu.Lock()
	defer c.mu.Unlock()

	if !c.initialized {
		c.xReal = real
		c.xImag = imag
		c.initialized = true
		return real, imag
	}

	pRealPred := c.pReal + c.qReal
	pImagPred := c.pImag + c.qImag
	pCrossPred := c.pCross

	innovReal := real - c.xReal
	innovImag := imag - c.xImag

	det := (pRealPred+c.rReal)*(pImagPred+c.rImag) - pCrossPred*pCrossPred
	if math.Abs(det) < 1e-16 {
		det = 1e-16
	}

	kRealReal := (pImagPred + c.rImag) / det
	kRealImag := -pCrossPred / det
	kImagReal := -pCrossPred / det
	kImagImag := (pRealPred + c.rReal) / det

	gainReal := pRealPred*kRealReal + pCrossPred*kImagReal
	gainImag := pCrossPred*kRealImag + pImagPred*kImagImag

	c.xReal += gainReal*innovReal + gainImag*innovImag
	c.xImag += gainReal*innovImag + gainImag*innovImag

	c.pReal = (1.0 - gainReal) * pRealPred
	c.pImag = (1.0 - gainImag) * pImagPred
	c.pCross = (1.0-gainReal)*(1.0-gainImag)*pCrossPred / 2.0

	innovMag := innovReal*innovReal + innovImag*innovImag
	if innovMag > 0.1 {
		c.rReal = c.alpha*c.rReal + (1.0-c.alpha)*innovMag*0.5
		c.rImag = c.alpha*c.rImag + (1.0-c.alpha)*innovMag*0.5
	}

	return c.xReal, c.xImag
}

func (c *ComplexKalman) Reset() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.xReal = 0
	c.xImag = 0
	c.pReal = 1.0
	c.pImag = 1.0
	c.pCross = 0
	c.initialized = false
}

func (c *ComplexKalman) State() (real, imag float64) {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.xReal, c.xImag
}

func MovingAverage(data []float64, window int) []float64 {
	if window <= 1 || len(data) == 0 {
		result := make([]float64, len(data))
		copy(result, data)
		return result
	}

	n := len(data)
	result := make([]float64, n)
	var sum float64

	for i := 0; i < n; i++ {
		sum += data[i]
		if i >= window {
			sum -= data[i-window]
		}
		if i < window-1 {
			result[i] = data[i]
		} else {
			result[i] = sum / float64(window)
		}
	}

	return result
}

func RMSNoise(data []float64) float64 {
	if len(data) < 2 {
		return 0
	}
	n := float64(len(data))
	var mean, sum float64
	for _, v := range data {
		mean += v
	}
	mean /= n
	for _, v := range data {
		d := v - mean
		sum += d * d
	}
	return math.Sqrt(sum / (n - 1.0))
}
