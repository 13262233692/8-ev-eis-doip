package thermal

import (
	"math"
	"sync"
)

const (
	StateDim = 5
)

type EKFState struct {
	SOC        float64
	EtaAnode   float64
	UPhiSolid  float64
	Rct        float64
	TauDiff    float64
}

type EKFCovariance struct {
	P [StateDim][StateDim]float64
}

type EKFConfig struct {
	ProcessNoiseSOC       float64
	ProcessNoiseEta       float64
	ProcessNoisePhi       float64
	ProcessNoiseRct       float64
	ProcessNoiseTau       float64
	MeasNoiseVoltage      float64
	MeasNoiseCurrent      float64
	InitialSOC            float64
	InitialEta            float64
	InitialPhi            float64
	InitialRct            float64
	InitialTau            float64
	OCVFunc               func(soc float64) float64
	NominalCapacityAh     float64
	DT                    float64
}

func DefaultEKFConfig() EKFConfig {
	return EKFConfig{
		ProcessNoiseSOC:   1e-5,
		ProcessNoiseEta:   1e-4,
		ProcessNoisePhi:   1e-4,
		ProcessNoiseRct:   1e-6,
		ProcessNoiseTau:   1e-6,
		MeasNoiseVoltage:  5e-3,
		MeasNoiseCurrent:  1e-2,
		InitialSOC:        0.5,
		InitialEta:        0.3,
		InitialPhi:        0.8,
		InitialRct:        0.05,
		InitialTau:        1.0,
		NominalCapacityAh: 80.0,
		DT:                1e-4,
		OCVFunc:           DefaultOCVCurve,
	}
}

func DefaultOCVCurve(soc float64) float64 {
	if soc < 0 {
		soc = 0
	}
	if soc > 1 {
		soc = 1
	}
	return 3.0 + 1.2*soc - 0.3*soc*soc + 0.1*soc*soc*soc
}

type ExtendedKalmanFilter struct {
	mu     sync.RWMutex
	x      [StateDim]float64
	P      [StateDim][StateDim]float64
	Q      [StateDim]float64
	R      [2]float64
	cfg    EKFConfig
	init   bool
	stepN  uint64
}

func NewExtendedKalmanFilter(cfg EKFConfig) *ExtendedKalmanFilter {
	ekf := &ExtendedKalmanFilter{
		cfg: cfg,
		Q: [StateDim]float64{
			cfg.ProcessNoiseSOC,
			cfg.ProcessNoiseEta,
			cfg.ProcessNoisePhi,
			cfg.ProcessNoiseRct,
			cfg.ProcessNoiseTau,
		},
		R: [2]float64{
			cfg.MeasNoiseVoltage,
			cfg.MeasNoiseCurrent,
		},
	}

	ekf.x[0] = cfg.InitialSOC
	ekf.x[1] = cfg.InitialEta
	ekf.x[2] = cfg.InitialPhi
	ekf.x[3] = cfg.InitialRct
	ekf.x[4] = cfg.InitialTau

	for i := 0; i < StateDim; i++ {
		ekf.P[i][i] = 1.0
	}

	return ekf
}

func (e *ExtendedKalmanFilter) Predict(current float64) {
	e.mu.Lock()
	defer e.mu.Unlock()

	dt := e.cfg.DT
	capAs := e.cfg.NominalCapacityAh * 3600.0

	soc := e.x[0]
	eta := e.x[1]
	phi := e.x[2]
	rct := e.x[3]
	tau := e.x[4]

	e.x[0] = soc - dt*current/(eta*capAs)
	if e.x[0] < 0 {
		e.x[0] = 0
	}
	if e.x[0] > 1 {
		e.x[0] = 1
	}

	expTerm := math.Exp(-dt / math.Max(tau, 1e-12))
	e.x[2] = phi*expTerm + rct*current*(1-expTerm)

	e.x[1] = eta
	e.x[3] = rct
	e.x[4] = tau

	dSoc_dSoc := 1.0
	dSoc_dEta := dt * current / (eta * eta * capAs)

	dPhi_dPhi := expTerm
	dPhi_dRct := current * (1 - expTerm)
	dPhi_dTau := phi * dt * expTerm / (tau * tau)

	F := [StateDim][StateDim]float64{}
	F[0][0] = dSoc_dSoc
	F[0][1] = dSoc_dEta

	F[1][1] = 1.0

	F[2][2] = dPhi_dPhi
	F[2][3] = dPhi_dRct
	F[2][4] = dPhi_dTau

	F[3][3] = 1.0
	F[4][4] = 1.0

	var Pnew [StateDim][StateDim]float64
	for i := 0; i < StateDim; i++ {
		for j := 0; j < StateDim; j++ {
			for k := 0; k < StateDim; k++ {
				Pnew[i][j] += F[i][k] * e.P[k][j]
			}
		}
	}
	for i := 0; i < StateDim; i++ {
		for j := 0; j < StateDim; j++ {
			var sum float64
			for k := 0; k < StateDim; k++ {
				sum += Pnew[i][k] * F[j][k]
			}
			e.P[i][j] = sum
		}
	}

	for i := 0; i < StateDim; i++ {
		e.P[i][i] += e.Q[i]
	}
}

func (e *ExtendedKalmanFilter) Update(voltage, current float64) {
	e.mu.Lock()
	defer e.mu.Unlock()

	ocv := e.cfg.OCVFunc(e.x[0])
	eta := e.x[1]
	phi := e.x[2]
	rct := e.x[3]

	etaOverpotential := ocv - phi - rct*current

	vPred := ocv - phi - rct*current/eta

	h := [2][StateDim]float64{}

	soc := e.x[0]
	docv := ocvDerivative(e.cfg.OCVFunc, soc)

	h[0][0] = docv
	h[0][2] = -1.0
	h[0][3] = -current / eta
	h[0][1] = rct * current / (eta * eta)

	h[1][0] = 0
	h[1][1] = 0
	h[1][2] = 0
	h[1][3] = 0
	h[1][4] = 0

	innov := [2]float64{voltage - vPred, 0}

	var S [2][2]float64
	for i := 0; i < 2; i++ {
		for j := 0; j < 2; j++ {
			for k := 0; k < StateDim; k++ {
				S[i][j] += h[i][k] * e.P[k][0] * h[j][0] +
					h[i][k]*e.P[k][1]*h[j][1] +
					h[i][k]*e.P[k][2]*h[j][2] +
					h[i][k]*e.P[k][3]*h[j][3] +
					h[i][k]*e.P[k][4]*h[j][4]
			}
		}
		S[i][i] += e.R[i]
	}

	detS := S[0][0]*S[1][1] - S[0][1]*S[1][0]
	if math.Abs(detS) < 1e-20 {
		detS = 1e-20
	}
	invS := [2][2]float64{
		{S[1][1] / detS, -S[0][1] / detS},
		{-S[1][0] / detS, S[0][0] / detS},
	}

	var K [StateDim][2]float64
	for i := 0; i < StateDim; i++ {
		for j := 0; j < 2; j++ {
			for k := 0; k < 2; k++ {
				K[i][j] += e.P[i][0]*h[k][0]*invS[0][j] +
					e.P[i][1]*h[k][1]*invS[0][j] +
					e.P[i][2]*h[k][2]*invS[0][j] +
					e.P[i][3]*h[k][3]*invS[0][j] +
					e.P[i][4]*h[k][4]*invS[0][j]
			}
		}
	}

	for i := 0; i < StateDim; i++ {
		e.x[i] += K[i][0]*innov[0] + K[i][1]*innov[1]
	}

	if e.x[0] < 0 {
		e.x[0] = 0
	}
	if e.x[0] > 1 {
		e.x[0] = 1
	}
	if e.x[1] < 0.01 {
		e.x[1] = 0.01
	}
	if e.x[1] > 1.0 {
		e.x[1] = 1.0
	}
	if e.x[3] < 1e-6 {
		e.x[3] = 1e-6
	}
	if e.x[4] < 1e-6 {
		e.x[4] = 1e-6
	}

	var KH [StateDim][StateDim]float64
	for i := 0; i < StateDim; i++ {
		for j := 0; j < StateDim; j++ {
			KH[i][j] = K[i][0]*h[0][j] + K[i][1]*h[1][j]
		}
	}

	var I_KH [StateDim][StateDim]float64
	for i := 0; i < StateDim; i++ {
		I_KH[i][i] = 1.0
		for j := 0; j < StateDim; j++ {
			I_KH[i][j] -= KH[i][j]
		}
	}

	var Pnew [StateDim][StateDim]float64
	for i := 0; i < StateDim; i++ {
		for j := 0; j < StateDim; j++ {
			for k := 0; k < StateDim; k++ {
				Pnew[i][j] += I_KH[i][k] * e.P[k][j]
			}
		}
	}
	e.P = Pnew

	e.stepN++

	_ = etaOverpotential
}

func (e *ExtendedKalmanFilter) Step(voltage, current float64) EKFState {
	e.Predict(current)
	e.Update(voltage, current)
	return e.State()
}

func (e *ExtendedKalmanFilter) State() EKFState {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return EKFState{
		SOC:       e.x[0],
		EtaAnode:  e.x[1],
		UPhiSolid: e.x[2],
		Rct:       e.x[3],
		TauDiff:   e.x[4],
	}
}

func (e *ExtendedKalmanFilter) Covariance() EKFCovariance {
	e.mu.RLock()
	defer e.mu.RUnlock()
	cov := EKFCovariance{}
	cov.P = e.P
	return cov
}

func (e *ExtendedKalmanFilter) AnodeOverpotential(voltage, current float64) float64 {
	e.mu.RLock()
	defer e.mu.RUnlock()

	ocv := e.cfg.OCVFunc(e.x[0])
	phi := e.x[2]
	rct := e.x[3]
	eta := e.x[1]

	etaAnode := ocv - phi - rct*current/eta
	return etaAnode
}

func (e *ExtendedKalmanFilter) StepCount() uint64 {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.stepN
}

func (e *ExtendedKalmanFilter) Reset() {
	e.mu.Lock()
	defer e.mu.Unlock()

	e.x[0] = e.cfg.InitialSOC
	e.x[1] = e.cfg.InitialEta
	e.x[2] = e.cfg.InitialPhi
	e.x[3] = e.cfg.InitialRct
	e.x[4] = e.cfg.InitialTau

	for i := 0; i < StateDim; i++ {
		for j := 0; j < StateDim; j++ {
			e.P[i][j] = 0
		}
		e.P[i][i] = 1.0
	}

	e.init = false
	e.stepN = 0
}

func (e *ExtendedKalmanFilter) SetState(state EKFState) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.x[0] = state.SOC
	e.x[1] = state.EtaAnode
	e.x[2] = state.UPhiSolid
	e.x[3] = state.Rct
	e.x[4] = state.TauDiff
}

func ocvDerivative(ocvFunc func(float64) float64, soc float64) float64 {
	ds := 1e-6
	if soc < ds {
		ds = soc / 2
	}
	if soc > 1-ds {
		ds = (1 - soc) / 2
	}
	if ds <= 0 {
		return 0
	}
	return (ocvFunc(soc+ds) - ocvFunc(soc-ds)) / (2 * ds)
}

func OverpotentialGradient(history []float64, window int) (gradient float64, secondDeriv float64) {
	n := len(history)
	if n < 3 || window < 3 {
		return 0, 0
	}

	start := n - window
	if start < 0 {
		start = 0
	}
	sub := history[start:]

	m := len(sub)
	if m < 3 {
		return 0, 0
	}

	dx := 1.0

	gradSum := 0.0
	for i := 1; i < m; i++ {
		gradSum += (sub[i] - sub[i-1]) / dx
	}
	gradient = gradSum / float64(m-1)

	secondSum := 0.0
	for i := 1; i < m-1; i++ {
		secondSum += (sub[i+1] - 2*sub[i] + sub[i-1]) / (dx * dx)
	}
	secondDeriv = secondSum / float64(m-2)

	return gradient, secondDeriv
}
