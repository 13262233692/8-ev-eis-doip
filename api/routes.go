package api

import (
	"encoding/hex"
	"fmt"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"

	"github.com/evplatform/eis-doip-vcu/battery"
	"github.com/evplatform/eis-doip-vcu/doip"
	"github.com/evplatform/eis-doip-vcu/iso15765"
	"github.com/evplatform/eis-doip-vcu/thermal"
)

type Server struct {
	router       *gin.Engine
	engine       *battery.Engine
	doipScanner  *doip.ZeroCopyScanner
	isoParser    *iso15765.Parser
	hazard       *thermal.HazardDetector
	stats        *ServiceStats
}

type ServiceStats struct {
	APICalls      uint64
	DoIPFrames    uint64
	ISO15765Msgs  uint64
	EISProcessed  uint64
	Errors        uint64
	Uptime        time.Time
}

type diagnosisResponse struct {
	Timestamp        string  `json:"timestamp"`
	Status           string  `json:"status"`
	SOH              float64 `json:"soh_percent"`
	OhmicResistance  float64 `json:"ohmic_resistance_ohm"`
	ChargeTransferR  float64 `json:"charge_transfer_resistance_ohm"`
	DoubleLayerCap   float64 `json:"double_layer_capacitance_farad"`
	WarburgImpedance float64 `json:"warburg_impedance"`
	TimeConstant     float64 `json:"time_constant_seconds"`
	Residual         float64 `json:"fitting_residual"`
	Iterations       int     `json:"fitting_iterations"`
	Converged        bool    `json:"converged"`
	SampleCount      int     `json:"eis_sample_count"`
	ProcessingTimeMs int64   `json:"processing_time_ms"`
	LastError        string  `json:"last_error,omitempty"`
}

type statsResponse struct {
	APICalls      uint64  `json:"api_calls"`
	DoIPFrames    uint64  `json:"doip_frames_received"`
	ISO15765Msgs  uint64  `json:"iso15765_messages"`
	EISProcessed  uint64  `json:"eis_payloads_processed"`
	Errors        uint64  `json:"errors"`
	UptimeSeconds float64 `json:"uptime_seconds"`
	EngineStatus  string  `json:"engine_status"`
	ProcessCount  uint64  `json:"sample_process_count"`
}

type eisInjectRequest struct {
	DataHex string `json:"data_hex" binding:"required"`
}

type eisPointsRequest struct {
	Points []battery.EISPoint `json:"points" binding:"required"`
}

type referenceParamsRequest struct {
	Rs  float64 `json:"rs" binding:"required"`
	Rct float64 `json:"rct" binding:"required"`
	Cdl float64 `json:"cdl" binding:"required"`
	W   float64 `json:"w" binding:"required"`
	Tau float64 `json:"tau" binding:"required"`
}

type syntheticRequest struct {
	Rs        float64 `json:"rs" binding:"required"`
	Rct       float64 `json:"rct" binding:"required"`
	Cdl       float64 `json:"cdl" binding:"required"`
	W         float64 `json:"w" binding:"required"`
	Tau       float64 `json:"tau" binding:"required"`
	FreqStart float64 `json:"freq_start" binding:"required"`
	FreqEnd   float64 `json:"freq_end" binding:"required"`
	NumPoints int     `json:"num_points" binding:"required"`
}

type chargeSampleRequest struct {
	Voltage float64 `json:"voltage" binding:"required"`
	Current float64 `json:"current" binding:"required"`
}

type chargeBatchRequest struct {
	Samples []chargeSampleRequest `json:"samples" binding:"required"`
}

type hazardConfigRequest struct {
	OverpotentialThreshold   *float64 `json:"overpotential_threshold"`
	OverpotentialWarnThresh  *float64 `json:"overpotential_warn_thresh"`
	OverpotentialCritThresh  *float64 `json:"overpotential_crit_thresh"`
	GradientThreshold        *float64 `json:"gradient_threshold"`
	GradientCritThreshold    *float64 `json:"gradient_crit_threshold"`
	IntegralWindowSec        *float64 `json:"integral_window_sec"`
	IntegralThreshold        *float64 `json:"integral_threshold"`
	RecoveryThreshold        *float64 `json:"recovery_threshold"`
	DebounceCount            *int     `json:"debounce_count"`
	AutoContactorOpen        *bool    `json:"auto_contactor_open"`
}

func NewServer(engine *battery.Engine) *Server {
	gin.SetMode(gin.ReleaseMode)
	s := &Server{
		router: gin.New(),
		engine: engine,
		stats: &ServiceStats{
			Uptime: time.Now(),
		},
	}
	s.setupRoutes()
	return s
}

func (s *Server) Router() *gin.Engine {
	return s.router
}

func (s *Server) SetDoIPScanner(sc *doip.ZeroCopyScanner) {
	s.doipScanner = sc
}

func (s *Server) SetISOParser(p *iso15765.Parser) {
	s.isoParser = p
}

func (s *Server) SetHazardDetector(hd *thermal.HazardDetector) {
	s.hazard = hd
}

func (s *Server) setupRoutes() {
	s.router.Use(s.middleware())

	api := s.router.Group("/api/v1")
	{
		api.GET("/diagnosis/status", s.getDiagnosisStatus)
		api.GET("/diagnosis/last", s.getLastResult)
		api.POST("/diagnosis/run", s.runFitting)
		api.POST("/diagnosis/reset", s.resetDiagnosis)

		api.POST("/eis/inject", s.injectEISPayload)
		api.POST("/eis/inject/points", s.injectEISPoints)
		api.POST("/eis/synthetic", s.generateAndProcessSynthetic)
		api.GET("/eis/data", s.getEISData)
		api.GET("/eis/data/raw", s.getRawEISData)

		api.GET("/reference", s.getReferenceParams)
		api.POST("/reference", s.setReferenceParams)

		api.GET("/thermal/hazard/status", s.getHazardStatus)
		api.GET("/thermal/hazard/assessment", s.getHazardAssessment)
		api.POST("/thermal/charge/sample", s.injectChargeSample)
		api.POST("/thermal/charge/batch", s.injectChargeBatch)
		api.POST("/thermal/contactor/emergency-open", s.emergencyOpenContactor)
		api.POST("/thermal/contactor/close", s.closeContactor)
		api.GET("/thermal/contactor/state", s.getContactorState)
		api.GET("/thermal/ekf/state", s.getEKFState)
		api.POST("/thermal/ekf/reset", s.resetEKF)
		api.GET("/thermal/audit/events", s.getAuditEvents)
		api.GET("/thermal/audit/stats", s.getAuditStats)
		api.POST("/thermal/config", s.updateHazardConfig)
		api.POST("/thermal/reset", s.resetHazard)

		api.GET("/stats", s.getStats)
		api.GET("/health", s.healthCheck)
	}
}

func (s *Server) middleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Header("X-Service", "EIS-DOIP-VCU")
		c.Header("X-Version", "1.0.0")
		c.Next()
	}
}

func diagnosisToResponse(d *battery.HealthDiagnosis) diagnosisResponse {
	return diagnosisResponse{
		Timestamp:        d.Timestamp.Format(time.RFC3339Nano),
		Status:           string(d.Status),
		SOH:              d.SOH,
		OhmicResistance:  d.Params.Rs,
		ChargeTransferR:  d.Params.Rct,
		DoubleLayerCap:   d.Params.Cdl,
		WarburgImpedance: d.Params.W,
		TimeConstant:     d.Params.Tau,
		Residual:         d.FittingResidual,
		Iterations:       d.Iterations,
		Converged:        d.Converged,
		SampleCount:      d.SampleCount,
		ProcessingTimeMs: d.ProcessingTimeMs,
		LastError:        d.LastError,
	}
}

func (s *Server) getDiagnosisStatus(c *gin.Context) {
	d := s.engine.CurrentDiagnosis()
	c.JSON(http.StatusOK, diagnosisToResponse(d))
}

func (s *Server) getLastResult(c *gin.Context) {
	d := s.engine.LastResult()
	c.JSON(http.StatusOK, diagnosisToResponse(d))
}

func (s *Server) runFitting(c *gin.Context) {
	result, err := s.engine.RunFitting()
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": err.Error(),
		})
		return
	}
	c.JSON(http.StatusOK, diagnosisToResponse(result))
}

func (s *Server) resetDiagnosis(c *gin.Context) {
	s.engine.Reset()
	c.JSON(http.StatusOK, gin.H{"status": "reset"})
}

func (s *Server) injectEISPayload(c *gin.Context) {
	var req eisInjectRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	data, err := hex.DecodeString(req.DataHex)
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid hex encoding"})
		return
	}

	result, err := s.engine.ProcessEISPayload(data)
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	c.JSON(http.StatusOK, diagnosisToResponse(result))
}

func (s *Server) injectEISPoints(c *gin.Context) {
	var req eisPointsRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	s.engine.SetEISData(req.Points)

	result, err := s.engine.RunFitting()
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	c.JSON(http.StatusOK, diagnosisToResponse(result))
}

func (s *Server) generateAndProcessSynthetic(c *gin.Context) {
	var req syntheticRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	params := battery.RandlesParams{
		Rs:  req.Rs,
		Rct: req.Rct,
		Cdl: req.Cdl,
		W:   req.W,
		Tau: req.Tau,
	}

	points := battery.GenerateSyntheticEIS(params, req.FreqStart, req.FreqEnd, req.NumPoints)
	s.engine.SetEISData(points)

	result, err := s.engine.RunFitting()
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"ground_truth": gin.H{
			"rs":  params.Rs,
			"rct": params.Rct,
			"cdl": params.Cdl,
			"w":   params.W,
			"tau": params.Tau,
		},
		"fitted": diagnosisToResponse(result),
		"points": points,
	})
}

func (s *Server) getEISData(c *gin.Context) {
	data := s.engine.GetEISData()
	c.JSON(http.StatusOK, gin.H{
		"count": len(data),
		"points": data,
	})
}

func (s *Server) getRawEISData(c *gin.Context) {
	data := s.engine.GetRawEISData()
	c.JSON(http.StatusOK, gin.H{
		"count": len(data),
		"points": data,
	})
}

func (s *Server) getReferenceParams(c *gin.Context) {
	p := s.engine.ReferenceParams()
	c.JSON(http.StatusOK, gin.H{
		"rs":  p.Rs,
		"rct": p.Rct,
		"cdl": p.Cdl,
		"w":   p.W,
		"tau": p.Tau,
	})
}

func (s *Server) setReferenceParams(c *gin.Context) {
	var req referenceParamsRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	s.engine.SetReferenceParams(battery.RandlesParams{
		Rs:  req.Rs,
		Rct: req.Rct,
		Cdl: req.Cdl,
		W:   req.W,
		Tau: req.Tau,
	})

	c.JSON(http.StatusOK, gin.H{"status": "ok"})
}

func (s *Server) getHazardStatus(c *gin.Context) {
	if s.hazard == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "hazard detector not initialized"})
		return
	}

	checks, triggers, level := s.hazard.Stats()
	assess := s.hazard.LastAssessment()

	resp := gin.H{
		"hazard_level":      thermal.HazardLevelString(level),
		"total_checks":      checks,
		"trigger_count":     triggers,
		"running":           true,
	}

	if assess != nil {
		resp["overpotential_v"] = assess.Overpotential
		resp["gradient_v_per_s"] = assess.Gradient
		resp["integral_v_s"] = assess.IntegralValue
		resp["debounce_counter"] = assess.DebounceCounter
		resp["contactor_triggered"] = assess.ContactorTriggered
		resp["last_assessment_ts"] = assess.Timestamp.Format(time.RFC3339Nano)
	}

	c.JSON(http.StatusOK, resp)
}

func (s *Server) getHazardAssessment(c *gin.Context) {
	if s.hazard == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "hazard detector not initialized"})
		return
	}

	assess := s.hazard.LastAssessment()
	if assess == nil {
		c.JSON(http.StatusOK, gin.H{"status": "no_assessment_yet"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"level":                thermal.HazardLevelString(assess.Level),
		"overpotential_v":      assess.Overpotential,
		"gradient_v_per_s":     assess.Gradient,
		"second_derivative":    assess.SecondDerivative,
		"integral_v_s":         assess.IntegralValue,
		"debounce_counter":     assess.DebounceCounter,
		"contactor_triggered":  assess.ContactorTriggered,
		"timestamp":            assess.Timestamp.Format(time.RFC3339Nano),
	})
}

func (s *Server) injectChargeSample(c *gin.Context) {
	if s.hazard == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "hazard detector not initialized"})
		return
	}

	var req chargeSampleRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	assess := s.hazard.IngestSample(req.Voltage, req.Current, time.Now())
	if assess == nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "hazard detector not running"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"hazard_level":        thermal.HazardLevelString(assess.Level),
		"overpotential_v":     assess.Overpotential,
		"gradient_v_per_s":    assess.Gradient,
		"integral_v_s":        assess.IntegralValue,
		"contactor_triggered": assess.ContactorTriggered,
		"timestamp":           assess.Timestamp.Format(time.RFC3339Nano),
	})
}

func (s *Server) injectChargeBatch(c *gin.Context) {
	if s.hazard == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "hazard detector not initialized"})
		return
	}

	var req chargeBatchRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	var lastAssess *thermal.HazardAssessment
	maxHazard := thermal.HazardNormal

	for i, sample := range req.Samples {
		assess := s.hazard.IngestSample(sample.Voltage, sample.Current, time.Now())
		if assess != nil {
			lastAssess = assess
			if assess.Level > maxHazard {
				maxHazard = assess.Level
			}
		}
		_ = i
	}

	if lastAssess == nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "no assessments generated"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"samples_processed":   len(req.Samples),
		"max_hazard_level":    thermal.HazardLevelString(maxHazard),
		"overpotential_v":     lastAssess.Overpotential,
		"gradient_v_per_s":    lastAssess.Gradient,
		"integral_v_s":        lastAssess.IntegralValue,
		"contactor_triggered": lastAssess.ContactorTriggered,
		"timestamp":           lastAssess.Timestamp.Format(time.RFC3339Nano),
	})
}

func (s *Server) emergencyOpenContactor(c *gin.Context) {
	if s.hazard == nil || s.hazard.Contactor() == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "contactor manager not initialized"})
		return
	}

	resp, err := s.hazard.Contactor().EmergencyOpen()
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{
			"error":     fmt.Sprintf("contactor command failed: %v", err),
			"acknowledged": false,
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"acknowledged":    resp.Acknowledged,
		"contactor_state": thermal.ContactorStateString(resp.State),
		"timestamp":       resp.Timestamp.Format(time.RFC3339Nano),
	})
}

func (s *Server) closeContactor(c *gin.Context) {
	if s.hazard == nil || s.hazard.Contactor() == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "contactor manager not initialized"})
		return
	}

	resp, err := s.hazard.Contactor().Close()
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{
			"error":     fmt.Sprintf("contactor close failed: %v", err),
			"acknowledged": false,
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"acknowledged":    resp.Acknowledged,
		"contactor_state": thermal.ContactorStateString(resp.State),
		"timestamp":       resp.Timestamp.Format(time.RFC3339Nano),
	})
}

func (s *Server) getContactorState(c *gin.Context) {
	if s.hazard == nil || s.hazard.Contactor() == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "contactor manager not initialized"})
		return
	}

	cm := s.hazard.Contactor()
	cmdCount, emergencyN := cm.Stats()

	c.JSON(http.StatusOK, gin.H{
		"state":             thermal.ContactorStateString(cm.State()),
		"total_commands":    cmdCount,
		"emergency_opens":   emergencyN,
		"last_command":      cm.LastCommand(),
		"last_response":     cm.LastResponse(),
	})
}

func (s *Server) getEKFState(c *gin.Context) {
	if s.hazard == nil || s.hazard.EKF() == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "EKF not initialized"})
		return
	}

	state := s.hazard.EKF().State()

	c.JSON(http.StatusOK, gin.H{
		"soc":         state.SOC,
		"eta_anode":   state.EtaAnode,
		"u_phi_solid": state.UPhiSolid,
		"rct_ohm":     state.Rct,
		"tau_diff_s":  state.TauDiff,
		"step_count":  s.hazard.EKF().StepCount(),
	})
}

func (s *Server) resetEKF(c *gin.Context) {
	if s.hazard == nil || s.hazard.EKF() == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "EKF not initialized"})
		return
	}

	s.hazard.EKF().Reset()
	c.JSON(http.StatusOK, gin.H{"status": "ekf_reset"})
}

func (s *Server) getAuditEvents(c *gin.Context) {
	if s.hazard == nil || s.hazard.Audit() == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "audit log not initialized"})
		return
	}

	limit := 100
	if l, ok := c.GetQuery("limit"); ok {
		var parsed int
		if _, err := fmt.Sscanf(l, "%d", &parsed); err == nil && parsed > 0 {
			limit = parsed
		}
	}

	events := s.hazard.Audit().Events(limit)
	c.JSON(http.StatusOK, gin.H{
		"count":  len(events),
		"events": events,
	})
}

func (s *Server) getAuditStats(c *gin.Context) {
	if s.hazard == nil || s.hazard.Audit() == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "audit log not initialized"})
		return
	}

	total, emergency, critical, warning := s.hazard.Audit().Stats()
	c.JSON(http.StatusOK, gin.H{
		"total_events":   total,
		"emergency":      emergency,
		"critical":       critical,
		"warning":        warning,
	})
}

func (s *Server) updateHazardConfig(c *gin.Context) {
	if s.hazard == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "hazard detector not initialized"})
		return
	}

	var req hazardConfigRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	c.JSON(http.StatusOK, gin.H{"status": "config_updated", "note": "restart required for full config change"})
}

func (s *Server) resetHazard(c *gin.Context) {
	if s.hazard == nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "hazard detector not initialized"})
		return
	}

	s.hazard.Reset()
	c.JSON(http.StatusOK, gin.H{"status": "hazard_reset"})
}

func (s *Server) getStats(c *gin.Context) {
	resp := statsResponse{
		APICalls:      s.stats.APICalls,
		DoIPFrames:    s.stats.DoIPFrames,
		ISO15765Msgs:  s.stats.ISO15765Msgs,
		EISProcessed:  s.stats.EISProcessed,
		Errors:        s.stats.Errors,
		UptimeSeconds: time.Since(s.stats.Uptime).Seconds(),
		EngineStatus:  string(s.engine.Status()),
		ProcessCount:  s.engine.ProcessCount(),
	}
	c.JSON(http.StatusOK, resp)
}

func (s *Server) healthCheck(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"status": "ok",
		"service": "eis-doip-vcu",
		"uptime": time.Since(s.stats.Uptime).String(),
		"engine": string(s.engine.Status()),
	})
}

func (s *Server) IncrAPI() {
	s.stats.APICalls++
}

func (s *Server) IncrDoIP() {
	s.stats.DoIPFrames++
}

func (s *Server) IncrISO() {
	s.stats.ISO15765Msgs++
}

func (s *Server) IncrEIS() {
	s.stats.EISProcessed++
}

func (s *Server) IncrError() {
	s.stats.Errors++
}

func (s *Server) Run(addr string) error {
	return s.router.Run(addr)
}
