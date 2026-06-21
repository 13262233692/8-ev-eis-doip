package thermal

import (
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"
)

type HazardLevel int

const (
	HazardNormal     HazardLevel = iota
	HazardWatch
	HazardWarning
	HazardCritical
	HazardEmergency
)

type HazardConfig struct {
	OverpotentialThreshold    float64
	OverpotentialWarnThresh   float64
	OverpotentialCritThresh   float64
	GradientThreshold         float64
	GradientCritThreshold     float64
	IntegralWindowSec         float64
	IntegralThreshold         float64
	RecoveryThreshold         float64
	DebounceCount             int
	CooldownSec               float64
	AutoContactorOpen         bool
	RecordAllSamples          bool
	MaxHistoryLen             int
}

func DefaultHazardConfig() HazardConfig {
	return HazardConfig{
		OverpotentialThreshold:  0.0,
		OverpotentialWarnThresh: 0.05,
		OverpotentialCritThresh: -0.02,
		GradientThreshold:       -0.1,
		GradientCritThreshold:   -0.5,
		IntegralWindowSec:       10.0,
		IntegralThreshold:       -0.5,
		RecoveryThreshold:       0.05,
		DebounceCount:           3,
		CooldownSec:             5.0,
		AutoContactorOpen:       true,
		RecordAllSamples:        false,
		MaxHistoryLen:           10000,
	}
}

type HazardAssessment struct {
	Level              HazardLevel
	Overpotential      float64
	Gradient           float64
	SecondDerivative   float64
	IntegralValue      float64
	DebounceCounter    int
	ContactorTriggered bool
	Timestamp          time.Time
}

type ChargeSample struct {
	Voltage     float64
	Current     float64
	Timestamp   time.Time
	Overpotential float64
}

type HazardDetector struct {
	mu          sync.RWMutex
	cfg         HazardConfig
	ekf         *ExtendedKalmanFilter
	contactor   *ContactorManager
	audit       *AuditLog
	running     atomic.Bool
	hazardLevel HazardLevel
	debounceN   int
	lastAssess  *HazardAssessment
	history     []ChargeSample
	integral    float64
	integralT0  time.Time
	cooldownEnd time.Time
	totalChecks uint64
	triggerN    uint64
}

func NewHazardDetector(cfg HazardConfig, ekf *ExtendedKalmanFilter, contactor *ContactorManager, audit *AuditLog) *HazardDetector {
	if cfg.MaxHistoryLen <= 0 {
		cfg.MaxHistoryLen = 10000
	}
	return &HazardDetector{
		cfg:       cfg,
		ekf:       ekf,
		contactor: contactor,
		audit:     audit,
		history:   make([]ChargeSample, 0, cfg.MaxHistoryLen),
	}
}

func (hd *HazardDetector) IngestSample(voltage, current float64, ts time.Time) *HazardAssessment {
	if !hd.running.Load() {
		return nil
	}

	hd.mu.Lock()
	defer hd.mu.Unlock()

	hd.totalChecks++

	state := hd.ekf.Step(voltage, current)

	ocv := hd.ekf.cfg.OCVFunc(state.SOC)
	overpotential := ocv - state.UPhiSolid - state.Rct*current/state.EtaAnode

	sample := ChargeSample{
		Voltage:     voltage,
		Current:     current,
		Timestamp:   ts,
		Overpotential: overpotential,
	}

	hd.history = append(hd.history, sample)
	if len(hd.history) > hd.cfg.MaxHistoryLen {
		hd.history = hd.history[len(hd.history)-hd.cfg.MaxHistoryLen:]
	}

	opHist := make([]float64, len(hd.history))
	for i, s := range hd.history {
		opHist[i] = s.Overpotential
	}

	gradWindow := 50
	if gradWindow > len(opHist) {
		gradWindow = len(opHist)
	}
	gradient, secondDeriv := OverpotentialGradient(opHist, gradWindow)

	if hd.integralT0.IsZero() {
		hd.integralT0 = ts
		hd.integral = 0
	} else {
		dt := ts.Sub(hd.integralT0).Seconds()
		if dt > 0 {
			hd.integral += overpotential * dt
		}
		hd.integralT0 = ts

		windowDuration := hd.cfg.IntegralWindowSec
		cutoff := ts.Add(-time.Duration(windowDuration * float64(time.Second)))
		for len(hd.history) > 0 && hd.history[0].Timestamp.Before(cutoff) {
			if len(hd.history) > 1 {
				dt0 := hd.history[1].Timestamp.Sub(hd.history[0].Timestamp).Seconds()
				hd.integral -= hd.history[0].Overpotential * dt0
			}
			hd.history = hd.history[1:]
		}
	}

	assess := hd.assessHazard(overpotential, gradient, secondDeriv, state, ts)
	hd.lastAssess = assess

	if hd.cfg.RecordAllSamples && hd.audit != nil {
		hd.audit.Record(SafetyEvent{
			EventType:        "sample_processed",
			Severity:         SeverityInfo,
			Overpotential:    overpotential,
			OverpotGradient:  gradient,
			OverpotIntegral:  hd.integral,
			OverpotSecondDev: secondDeriv,
			SOC:              state.SOC,
			Current:          current,
			Voltage:          voltage,
			EtaAnode:         state.EtaAnode,
			UPhiSolid:        state.UPhiSolid,
			Rct:              state.Rct,
			TauDiff:          state.TauDiff,
			Message:          fmt.Sprintf("OP=%.6fV grad=%.6fV/s integ=%.6fVs", overpotential, gradient, hd.integral),
		})
	}

	return assess
}

func (hd *HazardDetector) assessHazard(overpotential, gradient, secondDeriv float64, state EKFState, ts time.Time) *HazardAssessment {
	assess := &HazardAssessment{
		Overpotential:    overpotential,
		Gradient:         gradient,
		SecondDerivative: secondDeriv,
		IntegralValue:    hd.integral,
		Timestamp:        ts,
	}

	newLevel := HazardNormal

	if overpotential < hd.cfg.OverpotentialThreshold {
		if overpotential < hd.cfg.OverpotentialCritThresh ||
			gradient < hd.cfg.GradientCritThreshold ||
			hd.integral < hd.cfg.IntegralThreshold {
			newLevel = HazardEmergency
		} else {
			newLevel = HazardCritical
		}
	} else if overpotential < hd.cfg.OverpotentialWarnThresh || gradient < hd.cfg.GradientThreshold {
		newLevel = HazardWarning
	} else if overpotential < hd.cfg.OverpotentialWarnThresh*2 {
		newLevel = HazardWatch
	}

	if newLevel >= HazardWarning && newLevel >= hd.hazardLevel {
		hd.debounceN++
	} else if newLevel < hd.hazardLevel && overpotential > hd.cfg.RecoveryThreshold {
		if hd.hazardLevel >= HazardCritical {
			newLevel = HazardWatch
		}
		hd.debounceN = 0
	}

	if hd.debounceN >= hd.cfg.DebounceCount && newLevel >= HazardCritical {
		assess.ContactorTriggered = true
		hd.triggerN++

		if hd.cfg.AutoContactorOpen && hd.contactor != nil && ts.After(hd.cooldownEnd) {
			resp, err := hd.contactor.EmergencyOpen()
			acknowledged := err == nil && resp != nil && resp.Acknowledged

			if hd.audit != nil {
				evtType := EventOverpotentialDrop
				if overpotential < hd.cfg.OverpotentialThreshold {
					evtType = EventLithiumPlating
				}

				hd.audit.Record(SafetyEvent{
					EventType:        evtType,
					Severity:         SeverityEmergency,
					Overpotential:    overpotential,
					OverpotGradient:  gradient,
					OverpotIntegral:  hd.integral,
					OverpotSecondDev: secondDeriv,
					SOC:              state.SOC,
					Current:          0,
					Voltage:          0,
					EtaAnode:         state.EtaAnode,
					UPhiSolid:        state.UPhiSolid,
					Rct:              state.Rct,
					TauDiff:          state.TauDiff,
					Message: fmt.Sprintf(
						"LI-PLATING EMERGENCY: eta=%.6fV < 0V, grad=%.6fV/s, integral=%.6fVs, d2=%.6fV/s2, contactor=%v",
						overpotential, gradient, hd.integral, secondDeriv, acknowledged,
					),
					ContactorCmdSent: acknowledged,
				})
			}

			hd.cooldownEnd = ts.Add(time.Duration(hd.cfg.CooldownSec * float64(time.Second)))
		}
	}

	if overpotential > hd.cfg.RecoveryThreshold && hd.hazardLevel >= HazardCritical && hd.audit != nil {
		hd.audit.Record(SafetyEvent{
			EventType:       EventGradualRecovery,
			Severity:        SeverityInfo,
			Overpotential:   overpotential,
			OverpotGradient: gradient,
			SOC:             state.SOC,
			Message:         fmt.Sprintf("Overpotential recovered to %.6fV above threshold", overpotential),
		})
	}

	hd.hazardLevel = newLevel
	assess.Level = newLevel
	assess.DebounceCounter = hd.debounceN

	return assess
}

func (hd *HazardDetector) Start() {
	hd.running.Store(true)
}

func (hd *HazardDetector) Stop() {
	hd.running.Store(false)
}

func (hd *HazardDetector) HazardLevel() HazardLevel {
	hd.mu.RLock()
	defer hd.mu.RUnlock()
	return hd.hazardLevel
}

func (hd *HazardDetector) LastAssessment() *HazardAssessment {
	hd.mu.RLock()
	defer hd.mu.RUnlock()
	if hd.lastAssess == nil {
		return nil
	}
	cp := *hd.lastAssess
	return &cp
}

func (hd *HazardDetector) Stats() (checks, triggers uint64, currentLevel HazardLevel) {
	hd.mu.RLock()
	defer hd.mu.RUnlock()
	return hd.totalChecks, hd.triggerN, hd.hazardLevel
}

func (hd *HazardDetector) Reset() {
	hd.mu.Lock()
	defer hd.mu.Unlock()
	hd.hazardLevel = HazardNormal
	hd.debounceN = 0
	hd.integral = 0
	hd.integralT0 = time.Time{}
	hd.lastAssess = nil
	hd.history = hd.history[:0]
	hd.cooldownEnd = time.Time{}
}

func (hd *HazardDetector) EKF() *ExtendedKalmanFilter {
	return hd.ekf
}

func (hd *HazardDetector) Contactor() *ContactorManager {
	return hd.contactor
}

func (hd *HazardDetector) Audit() *AuditLog {
	return hd.audit
}

func HazardLevelString(l HazardLevel) string {
	switch l {
	case HazardNormal:
		return "NORMAL"
	case HazardWatch:
		return "WATCH"
	case HazardWarning:
		return "WARNING"
	case HazardCritical:
		return "CRITICAL"
	case HazardEmergency:
		return "EMERGENCY"
	default:
		return "UNKNOWN"
	}
}

func IsLithiumPlating(overpotential float64, threshold float64) bool {
	return overpotential < threshold
}

func PlatingSeverity(overpotential float64) float64 {
	if overpotential >= 0 {
		return 0
	}
	return math.Min(math.Abs(overpotential)/0.1, 1.0)
}
