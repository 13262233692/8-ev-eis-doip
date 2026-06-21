package thermal

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"
)

type SeverityLevel int

const (
	SeverityInfo SeverityLevel = iota
	SeverityWarning
	SeverityCritical
	SeverityEmergency
)

type SafetyEventType string

const (
	EventLithiumPlating     SafetyEventType = "lithium_plating_detected"
	EventOverpotentialDrop  SafetyEventType = "overpotential_below_zero"
	EventContactorOpen      SafetyEventType = "contactor_emergency_open"
	EventContactorConfirm   SafetyEventType = "contactor_open_confirmed"
	EventContactorFailed    SafetyEventType = "contactor_open_failed"
	EventThermalRunawayRisk SafetyEventType = "thermal_runaway_risk"
	EventGradualRecovery    SafetyEventType = "overpotential_recovery"
)

type SafetyEvent struct {
	ID               string          `json:"id"`
	Timestamp        time.Time       `json:"timestamp"`
	EventType        SafetyEventType `json:"event_type"`
	Severity         SeverityLevel   `json:"severity"`
	Overpotential    float64         `json:"overpotential_v"`
	OverpotGradient  float64         `json:"overpotential_gradient_v_per_s"`
	OverpotIntegral  float64         `json:"overpotential_integral_v_s"`
	OverpotSecondDev float64         `json:"overpotential_second_derivative_v_per_s2"`
	SOC              float64         `json:"soc"`
	Current          float64         `json:"current_a"`
	Voltage          float64         `json:"voltage_v"`
	EtaAnode         float64         `json:"eta_anode"`
	UPhiSolid        float64         `json:"u_phi_solid_v"`
	Rct              float64         `json:"rct_ohm"`
	TauDiff          float64         `json:"tau_diff_s"`
	Message          string          `json:"message"`
	ContactorCmdSent bool            `json:"contactor_cmd_sent"`
	SessionID        string          `json:"session_id"`
}

type AuditLog struct {
	mu          sync.RWMutex
	file        *os.File
	events      []SafetyEvent
	maxEvents   int
	sessionID   string
	filePath    string
	totalEvents uint64
	emergencyN  uint64
	criticalN   uint64
	warningN    uint64
}

func NewAuditLog(filePath string, maxEvents int) *AuditLog {
	if maxEvents <= 0 {
		maxEvents = 10000
	}

	a := &AuditLog{
		events:    make([]SafetyEvent, 0, maxEvents),
		maxEvents: maxEvents,
		sessionID: fmt.Sprintf("session-%d", time.Now().UnixNano()),
		filePath:  filePath,
	}

	if filePath != "" {
		dir := filepath.Dir(filePath)
		_ = os.MkdirAll(dir, 0755)

		f, err := os.OpenFile(filePath, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0644)
		if err == nil {
			a.file = f
		}
	}

	return a
}

func (a *AuditLog) Record(evt SafetyEvent) {
	a.mu.Lock()
	defer a.mu.Unlock()

	if evt.ID == "" {
		evt.ID = fmt.Sprintf("evt-%d-%d", time.Now().UnixNano(), a.totalEvents)
	}
	if evt.Timestamp.IsZero() {
		evt.Timestamp = time.Now()
	}
	if evt.SessionID == "" {
		evt.SessionID = a.sessionID
	}

	a.events = append(a.events, evt)
	if len(a.events) > a.maxEvents {
		a.events = a.events[len(a.events)-a.maxEvents:]
	}

	a.totalEvents++
	switch evt.Severity {
	case SeverityEmergency:
		a.emergencyN++
	case SeverityCritical:
		a.criticalN++
	case SeverityWarning:
		a.warningN++
	}

	if a.file != nil {
		data, err := json.Marshal(evt)
		if err == nil {
			_, _ = a.file.Write(data)
			_, _ = a.file.Write([]byte("\n"))
		}
	}
}

func (a *AuditLog) Events(limit int) []SafetyEvent {
	a.mu.RLock()
	defer a.mu.RUnlock()

	n := len(a.events)
	if limit > 0 && limit < n {
		n = limit
	}

	result := make([]SafetyEvent, n)
	copy(result, a.events[len(a.events)-n:])
	return result
}

func (a *AuditLog) Stats() (total, emergency, critical, warning uint64) {
	a.mu.RLock()
	defer a.mu.RUnlock()
	return a.totalEvents, a.emergencyN, a.criticalN, a.warningN
}

func (a *AuditLog) LastEvent() *SafetyEvent {
	a.mu.RLock()
	defer a.mu.RUnlock()
	if len(a.events) == 0 {
		return nil
	}
	evt := a.events[len(a.events)-1]
	return &evt
}

func (a *AuditLog) Close() {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.file != nil {
		_ = a.file.Sync()
		_ = a.file.Close()
		a.file = nil
	}
}

func SeverityString(s SeverityLevel) string {
	switch s {
	case SeverityInfo:
		return "INFO"
	case SeverityWarning:
		return "WARNING"
	case SeverityCritical:
		return "CRITICAL"
	case SeverityEmergency:
		return "EMERGENCY"
	default:
		return "UNKNOWN"
	}
}
