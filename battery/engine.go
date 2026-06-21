package battery

import (
	"encoding/binary"
	"errors"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/evplatform/eis-doip-vcu/buffer"
	"github.com/evplatform/eis-doip-vcu/signal"
)

const (
	DefaultSampleRate = 10000
	EISMagicHeader    = 0x45495331
)

type DiagnosisStatus string

const (
	StatusIdle        DiagnosisStatus = "idle"
	StatusAcquiring   DiagnosisStatus = "acquiring"
	StatusFitting     DiagnosisStatus = "fitting"
	StatusComplete    DiagnosisStatus = "complete"
	StatusError       DiagnosisStatus = "error"
)

type HealthDiagnosis struct {
	Timestamp        time.Time
	Status           DiagnosisStatus
	SOH              float64
	Params           RandlesParams
	FittingResidual  float64
	Iterations       int
	Converged        bool
	SampleCount      int
	ProcessingTimeMs int64
	LastError        string
}

type Engine struct {
	mu              sync.RWMutex
	status          DiagnosisStatus
	current         *HealthDiagnosis
	referenceParams RandlesParams
	samples         *buffer.SampleRing
	kalmanVoltage   *signal.AdaptiveBilinearKalman
	kalmanCurrent   *signal.AdaptiveBilinearKalman
	complexKalman   *signal.ComplexKalman
	rawEISData      []EISPoint
	eisData         []EISPoint
	lastResult      *HealthDiagnosis
	processCount    uint64
	fitConfig       TrustRegionConfig
	sampleRate      float64
	autoFit         bool
}

type EngineConfig struct {
	SampleBufferSize int
	SampleRate       float64
	ReferenceParams  RandlesParams
	KalmanCfg        signal.KalmanConfig
	FitCfg           TrustRegionConfig
	AutoFit          bool
}

func DefaultEngineConfig() EngineConfig {
	refParams := RandlesParams{
		Rs:  0.005,
		Rct: 0.05,
		Cdl: 1e-3,
		W:   0.01,
		Tau: 1.0,
	}
	return EngineConfig{
		SampleBufferSize: 32768,
		SampleRate:       DefaultSampleRate,
		ReferenceParams:  refParams,
		KalmanCfg:        signal.DefaultKalmanConfig(),
		FitCfg:           DefaultTrustRegionConfig(),
		AutoFit:          true,
	}
}

func NewEngine(cfg EngineConfig) *Engine {
	if cfg.SampleBufferSize <= 0 {
		cfg.SampleBufferSize = 32768
	}
	if cfg.SampleRate <= 0 {
		cfg.SampleRate = DefaultSampleRate
	}

	e := &Engine{
		status:          StatusIdle,
		referenceParams: cfg.ReferenceParams,
		samples:         buffer.NewSampleRing(cfg.SampleBufferSize),
		kalmanVoltage:   signal.NewAdaptiveBilinearKalman(cfg.KalmanCfg),
		kalmanCurrent:   signal.NewAdaptiveBilinearKalman(cfg.KalmanCfg),
		complexKalman:   signal.NewComplexKalman(0.9),
		fitConfig:       cfg.FitCfg,
		sampleRate:      cfg.SampleRate,
		autoFit:         cfg.AutoFit,
	}
	return e
}

func (e *Engine) Status() DiagnosisStatus {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.status
}

func (e *Engine) SetStatus(s DiagnosisStatus) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.status = s
}

func (e *Engine) CurrentDiagnosis() *HealthDiagnosis {
	e.mu.RLock()
	defer e.mu.RUnlock()
	if e.current == nil {
		return &HealthDiagnosis{
			Status: StatusIdle,
		}
	}
	cp := *e.current
	return &cp
}

func (e *Engine) LastResult() *HealthDiagnosis {
	e.mu.RLock()
	defer e.mu.RUnlock()
	if e.lastResult == nil {
		return &HealthDiagnosis{Status: StatusIdle}
	}
	cp := *e.lastResult
	return &cp
}

func (e *Engine) ProcessCount() uint64 {
	return atomic.LoadUint64(&e.processCount)
}

func (e *Engine) InjectRawSample(voltage, current float64, timestamp time.Time) {
	_ = timestamp
	e.samples.Push([]float64{voltage, current})
	atomic.AddUint64(&e.processCount, 1)
}

func (e *Engine) FilterSamples(voltage, current float64) (float64, float64) {
	v := e.kalmanVoltage.Filter(voltage)
	i := e.kalmanCurrent.Filter(current)
	return v, i
}

func (e *Engine) ParseEISPayload(data []byte) ([]EISPoint, error) {
	if len(data) < 8 {
		return nil, errors.New("payload too short")
	}

	magic := binary.BigEndian.Uint32(data[0:4])
	if magic != EISMagicHeader {
		return nil, errors.New("invalid EIS magic header")
	}

	numPoints := int(binary.BigEndian.Uint32(data[4:8]))
	offset := 8
	pointSize := 24

	expectedLen := 8 + numPoints*pointSize
	if len(data) < expectedLen {
		numPoints = (len(data) - 8) / pointSize
		if numPoints <= 0 {
			return nil, errors.New("no EIS points in payload")
		}
	}

	points := make([]EISPoint, numPoints)
	for i := 0; i < numPoints; i++ {
		base := offset + i*pointSize
		freq := math.Float64frombits(binary.BigEndian.Uint64(data[base : base+8]))
		zReal := math.Float64frombits(binary.BigEndian.Uint64(data[base+8 : base+16]))
		zImag := math.Float64frombits(binary.BigEndian.Uint64(data[base+16 : base+24]))

		filteredReal, filteredImag := e.complexKalman.Filter(zReal, zImag)

		points[i] = EISPoint{
			Frequency: freq,
			ZReal:     filteredReal,
			ZImag:     filteredImag,
		}
	}

	return points, nil
}

func (e *Engine) SetEISData(points []EISPoint) {
	e.mu.Lock()
	e.rawEISData = make([]EISPoint, len(points))
	copy(e.rawEISData, points)

	filteredPoints := make([]EISPoint, len(points))
	for i, pt := range points {
		fr, fi := e.complexKalman.Filter(pt.ZReal, pt.ZImag)
		filteredPoints[i] = EISPoint{
			Frequency: pt.Frequency,
			ZReal:     fr,
			ZImag:     fi,
		}
	}
	e.eisData = filteredPoints
	e.mu.Unlock()
}

func (e *Engine) GetEISData() []EISPoint {
	e.mu.RLock()
	defer e.mu.RUnlock()
	result := make([]EISPoint, len(e.eisData))
	copy(result, e.eisData)
	return result
}

func (e *Engine) GetRawEISData() []EISPoint {
	e.mu.RLock()
	defer e.mu.RUnlock()
	result := make([]EISPoint, len(e.rawEISData))
	copy(result, e.rawEISData)
	return result
}

func (e *Engine) RunFitting() (*HealthDiagnosis, error) {
	start := time.Now()

	e.mu.Lock()
	e.status = StatusFitting
	diagnosis := &HealthDiagnosis{
		Timestamp: start,
		Status:    StatusFitting,
	}
	e.current = diagnosis

	data := make([]EISPoint, len(e.eisData))
	copy(data, e.eisData)
	e.mu.Unlock()

	if len(data) < 3 {
		diagnosis.Status = StatusError
		diagnosis.LastError = "insufficient EIS data points"
		e.mu.Lock()
		e.lastResult = diagnosis
		e.status = StatusError
		e.mu.Unlock()
		return diagnosis, errors.New(diagnosis.LastError)
	}

	diagnosis.SampleCount = len(data)

	initial := InitialGuessRandles(data)

	var result FittingResult
	done := make(chan struct{})
	go func() {
		result = TrustRegionReflectiveFit(data, initial, e.fitConfig)
		close(done)
	}()

	timeout := time.After(30 * time.Second)
	select {
	case <-done:
	case <-timeout:
		diagnosis.Status = StatusError
		diagnosis.LastError = "fitting timeout"
		e.mu.Lock()
		e.lastResult = diagnosis
		e.status = StatusError
		e.mu.Unlock()
		return diagnosis, errors.New(diagnosis.LastError)
	}

	diagnosis.Params = result.Params
	diagnosis.FittingResidual = result.Residual
	diagnosis.Iterations = result.Iterations
	diagnosis.Converged = result.Converged

	e.mu.RLock()
	ref := e.referenceParams
	e.mu.RUnlock()

	diagnosis.SOH = ComputeSOH(result.Params, ref)
	diagnosis.ProcessingTimeMs = time.Since(start).Milliseconds()

	if result.Converged {
		diagnosis.Status = StatusComplete
	} else {
		diagnosis.Status = StatusError
		diagnosis.LastError = "fitting did not converge"
	}

	e.mu.Lock()
	e.current = diagnosis
	e.lastResult = diagnosis
	e.status = diagnosis.Status
	e.mu.Unlock()

	return diagnosis, nil
}

func (e *Engine) ProcessEISPayload(data []byte) (*HealthDiagnosis, error) {
	e.SetStatus(StatusAcquiring)

	points, err := e.ParseEISPayload(data)
	if err != nil {
		e.SetStatus(StatusError)
		return nil, err
	}

	e.SetEISData(points)

	if e.autoFit {
		return e.RunFitting()
	}

	diag := &HealthDiagnosis{
		Timestamp:   time.Now(),
		Status:      StatusAcquiring,
		SampleCount: len(points),
	}
	e.mu.Lock()
	e.current = diag
	e.mu.Unlock()

	return diag, nil
}

func (e *Engine) Reset() {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.status = StatusIdle
	e.current = nil
	e.eisData = nil
	e.rawEISData = nil
	e.kalmanVoltage.Reset()
	e.kalmanCurrent.Reset()
	e.complexKalman.Reset()
}

func (e *Engine) SetReferenceParams(p RandlesParams) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.referenceParams = p
}

func (e *Engine) ReferenceParams() RandlesParams {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.referenceParams
}

func (e *Engine) SampleRate() float64 {
	return e.sampleRate
}

func (e *Engine) DrainBufferedSamples() [][]float64 {
	return e.samples.DrainAll()
}

func BuildEISPayload(points []EISPoint) []byte {
	buf := make([]byte, 8+len(points)*24)
	binary.BigEndian.PutUint32(buf[0:4], EISMagicHeader)
	binary.BigEndian.PutUint32(buf[4:8], uint32(len(points)))

	for i, pt := range points {
		base := 8 + i*24
		binary.BigEndian.PutUint64(buf[base:base+8], math.Float64bits(pt.Frequency))
		binary.BigEndian.PutUint64(buf[base+8:base+16], math.Float64bits(pt.ZReal))
		binary.BigEndian.PutUint64(buf[base+16:base+24], math.Float64bits(pt.ZImag))
	}

	return buf
}

func GenerateSyntheticEIS(params RandlesParams, freqStart, freqEnd float64, numPoints int) []EISPoint {
	points := make([]EISPoint, numPoints)
	logStart := math.Log10(freqStart)
	logEnd := math.Log10(freqEnd)
	for i := 0; i < numPoints; i++ {
		logFreq := logStart + (logEnd-logStart)*float64(i)/float64(numPoints-1)
		freq := math.Pow(10, logFreq)
		omega := 2.0 * math.Pi * freq
		zr, zi := RandlesImpedance(params, omega)
		points[i] = EISPoint{
			Frequency: freq,
			ZReal:     zr,
			ZImag:     zi,
		}
	}
	return points
}
