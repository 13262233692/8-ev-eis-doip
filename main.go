package main

import (
	"context"
	"encoding/binary"
	"flag"
	"fmt"
	"log"
	"net"
	"os"
	"os/signal"
	"sync"
	"sync/atomic"
	"syscall"
	"time"

	"github.com/evplatform/eis-doip-vcu/api"
	"github.com/evplatform/eis-doip-vcu/battery"
	"github.com/evplatform/eis-doip-vcu/doip"
	"github.com/evplatform/eis-doip-vcu/iso15765"
	"github.com/evplatform/eis-doip-vcu/thermal"
)

type ServiceConfig struct {
	UDPBindAddr  string
	TCPBindAddr  string
	HTTPAddr     string
	VCUSrcAddr   uint16
	VCUDstAddr   uint16
	HVDCAddr     uint16
	EnableUDP    bool
	EnableTCP    bool
	AuditLogPath string
}

type tcpConnEntry struct {
	conn    net.Conn
	scanner *doip.TCPStreamScanner
	active  atomic.Bool
	stopMu  sync.Mutex
}

type DiagnosticService struct {
	config       ServiceConfig
	engine       *battery.Engine
	server       *api.Server
	hazard       *thermal.HazardDetector
	udpConn      net.PacketConn
	udpConnLock  sync.Mutex
	tcpLn        net.Listener
	tcpLnLock    sync.Mutex
	isoParser    *iso15765.Parser
	scanner      *doip.ZeroCopyScanner
	tcpConns     map[net.Conn]*tcpConnEntry
	tcpConnsMu   sync.Mutex
	shutdownOnce sync.Once
	running      atomic.Bool
}

func main() {
	cfg := parseFlags()
	log.Printf("800V SiC VCU EIS-DOIP Diagnostic Microservice starting...")
	log.Printf("UDP bind: %s (enabled: %v)", cfg.UDPBindAddr, cfg.EnableUDP)
	log.Printf("TCP bind: %s (enabled: %v)", cfg.TCPBindAddr, cfg.EnableTCP)
	log.Printf("HTTP API: %s", cfg.HTTPAddr)
	log.Printf("VCU Source Addr: 0x%04X, Target Addr: 0x%04X, HVDC Addr: 0x%04X", cfg.VCUSrcAddr, cfg.VCUDstAddr, cfg.HVDCAddr)

	svc := NewDiagnosticService(cfg)
	if err := svc.Start(); err != nil {
		log.Fatalf("failed to start service: %v", err)
	}
	defer svc.Stop()

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	log.Printf("Service ready. Waiting for shutdown signal...")
	<-ctx.Done()
	log.Printf("Shutdown signal received. Stopping service...")
}

func parseFlags() ServiceConfig {
	cfg := ServiceConfig{}

	udpAddr := flag.String("udp-addr", "0.0.0.0:13400", "DoIP UDP bind address")
	tcpAddr := flag.String("tcp-addr", "0.0.0.0:13400", "DoIP TCP bind address")
	httpAddr := flag.String("http-addr", ":8080", "HTTP API bind address")
	srcAddr := flag.Int("src-addr", 0x0E80, "VCU source diagnostic address")
	dstAddr := flag.Int("dst-addr", 0x1001, "BMS target diagnostic address")
	hvdcAddr := flag.Int("hvdc-addr", 0x1010, "HVDC contactor target address")
	enableUDP := flag.Bool("enable-udp", false, "Enable DoIP UDP listener")
	enableTCP := flag.Bool("enable-tcp", false, "Enable DoIP TCP listener")
	auditPath := flag.String("audit-log", "logs/thermal_audit.jsonl", "Thermal audit log file path")

	flag.Parse()

	cfg.UDPBindAddr = *udpAddr
	cfg.TCPBindAddr = *tcpAddr
	cfg.HTTPAddr = *httpAddr
	cfg.VCUSrcAddr = uint16(*srcAddr)
	cfg.VCUDstAddr = uint16(*dstAddr)
	cfg.HVDCAddr = uint16(*hvdcAddr)
	cfg.EnableUDP = *enableUDP
	cfg.EnableTCP = *enableTCP
	cfg.AuditLogPath = *auditPath

	return cfg
}

func NewDiagnosticService(cfg ServiceConfig) *DiagnosticService {
	engineCfg := battery.DefaultEngineConfig()
	engineCfg.SampleRate = 10000.0
	engineCfg.SampleBufferSize = 65536
	engineCfg.AutoFit = true

	engine := battery.NewEngine(engineCfg)
	server := api.NewServer(engine)

	return &DiagnosticService{
		config:   cfg,
		engine:   engine,
		server:   server,
		tcpConns: make(map[net.Conn]*tcpConnEntry),
	}
}

func (svc *DiagnosticService) Start() error {
	if !svc.running.CompareAndSwap(false, true) {
		return fmt.Errorf("service already running")
	}

	svc.initThermalModule()

	svc.isoParser = iso15765.NewParser(func(msg *iso15765.ReassembledMessage) {
		svc.server.IncrISO()
		if len(msg.Data) >= 8 {
			_, err := svc.engine.ProcessEISPayload(msg.Data)
			if err == nil {
				svc.server.IncrEIS()
			} else {
				svc.server.IncrError()
			}
		}
	})
	svc.server.SetISOParser(svc.isoParser)

	svc.isoParser.SetAutoSendFC(true, 0xFF, 0)

	svc.isoParser.SetFlowControlCallback(func(srcAddr, dstAddr uint16, status, blockSize, stMin uint8) {
		svc.sendFlowControlResponse(srcAddr, dstAddr, status, blockSize, stMin)
	})

	if svc.config.EnableUDP {
		if err := svc.startUDPListener(); err != nil {
			log.Printf("Warning: failed to start UDP listener: %v", err)
		}
	}

	if svc.config.EnableTCP {
		if err := svc.startTCPListener(); err != nil {
			log.Printf("Warning: failed to start TCP listener: %v", err)
		}
	}

	go func() {
		log.Printf("HTTP API server starting on %s", svc.config.HTTPAddr)
		if err := svc.server.Run(svc.config.HTTPAddr); err != nil {
			if svc.running.Load() {
				log.Printf("HTTP server error: %v", err)
			}
		}
	}()

	time.Sleep(200 * time.Millisecond)
	log.Printf("Service started successfully with thermal hazard protection enabled")
	return nil
}

func (svc *DiagnosticService) initThermalModule() {
	ekfCfg := thermal.DefaultEKFConfig()
	ekfCfg.NominalCapacityAh = 80.0
	ekfCfg.DT = 1e-4
	ekfCfg.InitialSOC = 0.5
	ekfCfg.InitialEta = 0.95
	ekfCfg.InitialPhi = 0.8
	ekfCfg.InitialRct = 0.05
	ekfCfg.InitialTau = 1.0
	ekfCfg.MeasNoiseVoltage = 2e-3
	ekfCfg.MeasNoiseCurrent = 5e-3
	ekf := thermal.NewExtendedKalmanFilter(ekfCfg)

	contactorMgr := thermal.NewContactorManager(
		svc.config.VCUSrcAddr,
		svc.config.VCUDstAddr,
		svc.config.HVDCAddr,
	)

	contactorMgr.SetSendCallback(func(frame []byte, priority uint8) error {
		switch priority {
		case thermal.ContactorPriorityEmergency:
			log.Printf("[EMERGENCY] Sending contactor emergency open command to HVDC 0x%04X", svc.config.HVDCAddr)
		case thermal.ContactorPriorityHigh:
			log.Printf("[HIGH] Sending contactor command to HVDC 0x%04X", svc.config.HVDCAddr)
		default:
			log.Printf("[NORMAL] Sending contactor command to HVDC 0x%04X", svc.config.HVDCAddr)
		}

		svc.sendTCPResponse(frame)
		svc.sendUDPResponse(frame)
		return nil
	})

	auditLog := thermal.NewAuditLog(svc.config.AuditLogPath, 10000)

	hazardCfg := thermal.DefaultHazardConfig()
	hazardCfg.AutoContactorOpen = true
	hazardCfg.DebounceCount = 3
	hazardCfg.RecordAllSamples = false

	hd := thermal.NewHazardDetector(hazardCfg, ekf, contactorMgr, auditLog)
	hd.Start()

	svc.hazard = hd
	svc.server.SetHazardDetector(hd)

	log.Printf("Thermal hazard protection module initialized (HVDC=0x%04X, audit=%s)", svc.config.HVDCAddr, svc.config.AuditLogPath)
}

func (svc *DiagnosticService) startUDPListener() error {
	conn, err := net.ListenPacket("udp", svc.config.UDPBindAddr)
	if err != nil {
		return fmt.Errorf("udp bind failed: %w", err)
	}

	svc.udpConnLock.Lock()
	svc.udpConn = conn
	svc.udpConnLock.Unlock()

	scanner := doip.NewZeroCopyScanner(conn, func(payload []byte, srcAddr net.Addr, ts time.Time) {
		svc.server.IncrDoIP()
		svc.handleDoIPPayload(payload, srcAddr, ts)
	})
	svc.scanner = scanner
	svc.server.SetDoIPScanner(scanner)

	if err := scanner.Start(); err != nil {
		svc.udpConnLock.Lock()
		if svc.udpConn != nil {
			_ = svc.udpConn.Close()
			svc.udpConn = nil
		}
		svc.udpConnLock.Unlock()
		return fmt.Errorf("scanner start failed: %w", err)
	}

	log.Printf("DoIP UDP scanner started on %s", svc.config.UDPBindAddr)
	return nil
}

func (svc *DiagnosticService) startTCPListener() error {
	ln, err := net.Listen("tcp", svc.config.TCPBindAddr)
	if err != nil {
		return fmt.Errorf("tcp bind failed: %w", err)
	}

	svc.tcpLnLock.Lock()
	svc.tcpLn = ln
	svc.tcpLnLock.Unlock()

	go func() {
		for svc.running.Load() {
			conn, err := ln.Accept()
			if err != nil {
				if svc.running.Load() {
					log.Printf("TCP accept error: %v", err)
				}
				return
			}

			if !svc.running.Load() {
				_ = conn.Close()
				return
			}

			log.Printf("DoIP TCP connection from %s", conn.RemoteAddr())
			go svc.handleTCPConnection(conn)
		}
	}()

	log.Printf("DoIP TCP listener started on %s", svc.config.TCPBindAddr)
	return nil
}

func (svc *DiagnosticService) handleTCPConnection(conn net.Conn) {
	if conn == nil {
		return
	}

	entry := &tcpConnEntry{
		conn: conn,
	}
	entry.active.Store(true)

	svc.tcpConnsMu.Lock()
	svc.tcpConns[conn] = entry
	svc.tcpConnsMu.Unlock()

	defer func() {
		entry.stopMu.Lock()
		if entry.active.Load() {
			entry.active.Store(false)
			if entry.scanner != nil {
				entry.scanner.Stop()
			}
			_ = conn.Close()
		}
		entry.stopMu.Unlock()

		svc.tcpConnsMu.Lock()
		delete(svc.tcpConns, conn)
		svc.tcpConnsMu.Unlock()
	}()

	scanner := doip.NewTCPStreamScanner(conn, func(payload []byte, srcAddr net.Addr, ts time.Time) {
		if !entry.active.Load() {
			return
		}
		svc.server.IncrDoIP()
		svc.handleDoIPPayload(payload, srcAddr, ts)
	})
	entry.scanner = scanner

	if err := scanner.Start(); err != nil {
		log.Printf("TCP scanner start error: %v", err)
		return
	}

	for svc.running.Load() && entry.active.Load() {
		time.Sleep(100 * time.Millisecond)
	}
}

func (svc *DiagnosticService) handleDoIPPayload(payload []byte, srcAddr net.Addr, ts time.Time) {
	_ = srcAddr

	if len(payload) < 5 {
		return
	}

	src := binary.BigEndian.Uint16(payload[0:2])
	dst := binary.BigEndian.Uint16(payload[2:4])
	userDataLen := int(payload[4])

	var diagData []byte
	if userDataLen > 0 && len(payload) >= 5+userDataLen {
		diagData = payload[5 : 5+userDataLen]
	} else if len(payload) > 5 {
		diagData = payload[5:]
	}

	if len(diagData) > 0 && svc.isoParser != nil {
		_ = svc.isoParser.ProcessFrame(diagData, src, dst, ts)
	}
}

func (svc *DiagnosticService) sendFlowControlResponse(srcAddr, dstAddr uint16, status, blockSize, stMin uint8) {
	isoFC := make([]byte, 3)
	isoFC[0] = 0x30 | (status & 0x0F)
	isoFC[1] = blockSize
	isoFC[2] = stMin

	frame, err := doip.BuildDiagnosticMessage(dstAddr, srcAddr, isoFC)
	if err != nil {
		log.Printf("Failed to build DoIP FC frame: %v", err)
		return
	}

	svc.sendUDPResponse(frame)
	svc.sendTCPResponse(frame)
}

func (svc *DiagnosticService) sendUDPResponse(data []byte) {
	svc.udpConnLock.Lock()
	defer svc.udpConnLock.Unlock()

	if svc.udpConn == nil {
		return
	}

	broadcastAddr, err := net.ResolveUDPAddr("udp", "255.255.255.255:13400")
	if err != nil {
		return
	}

	_, _ = svc.udpConn.WriteTo(data, broadcastAddr)
}

func (svc *DiagnosticService) sendTCPResponse(data []byte) {
	svc.tcpConnsMu.Lock()
	defer svc.tcpConnsMu.Unlock()

	for _, entry := range svc.tcpConns {
		if entry == nil || !entry.active.Load() || entry.conn == nil {
			continue
		}

		func() {
			entry.stopMu.Lock()
			defer entry.stopMu.Unlock()

			if !entry.active.Load() || entry.conn == nil {
				return
			}

			_ = entry.conn.SetWriteDeadline(time.Now().Add(5 * time.Second))
			_, _ = entry.conn.Write(data)
		}()
	}
}

func (svc *DiagnosticService) Stop() {
	svc.shutdownOnce.Do(func() {
		svc.running.Store(false)

		if svc.hazard != nil {
			svc.hazard.Stop()
			if svc.hazard.Audit() != nil {
				svc.hazard.Audit().Close()
			}
		}

		svc.tcpConnsMu.Lock()
		for _, entry := range svc.tcpConns {
			if entry == nil {
				continue
			}
			entry.stopMu.Lock()
			if entry.active.Load() {
				entry.active.Store(false)
				if entry.scanner != nil {
					entry.scanner.Stop()
				}
				if entry.conn != nil {
					_ = entry.conn.Close()
				}
			}
			entry.stopMu.Unlock()
		}
		svc.tcpConns = make(map[net.Conn]*tcpConnEntry)
		svc.tcpConnsMu.Unlock()

		if svc.scanner != nil {
			svc.scanner.Stop()
		}

		svc.udpConnLock.Lock()
		if svc.udpConn != nil {
			_ = svc.udpConn.Close()
			svc.udpConn = nil
		}
		svc.udpConnLock.Unlock()

		svc.tcpLnLock.Lock()
		if svc.tcpLn != nil {
			_ = svc.tcpLn.Close()
			svc.tcpLn = nil
		}
		svc.tcpLnLock.Unlock()

		if svc.engine != nil {
			svc.engine.Reset()
		}

		log.Printf("Service stopped")
	})
}
