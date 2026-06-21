package api

import (
	"encoding/hex"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"

	"github.com/evplatform/eis-doip-vcu/battery"
	"github.com/evplatform/eis-doip-vcu/doip"
	"github.com/evplatform/eis-doip-vcu/iso15765"
)

type Server struct {
	router       *gin.Engine
	engine       *battery.Engine
	doipScanner  *doip.ZeroCopyScanner
	isoParser    *iso15765.Parser
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
	APICalls      uint64   `json:"api_calls"`
	DoIPFrames    uint64   `json:"doip_frames_received"`
	ISO15765Msgs  uint64   `json:"iso15765_messages"`
	EISProcessed  uint64   `json:"eis_payloads_processed"`
	Errors        uint64   `json:"errors"`
	UptimeSeconds float64  `json:"uptime_seconds"`
	EngineStatus  string   `json:"engine_status"`
	ProcessCount  uint64   `json:"sample_process_count"`
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
