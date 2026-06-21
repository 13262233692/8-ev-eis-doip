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
	"syscall"
	"time"

	"github.com/evplatform/eis-doip-vcu/api"
	"github.com/evplatform/eis-doip-vcu/battery"
	"github.com/evplatform/eis-doip-vcu/doip"
	"github.com/evplatform/eis-doip-vcu/iso15765"
)

type ServiceConfig struct {
	UDPBindAddr  string
	TCPBindAddr  string
	HTTPAddr     string
	VCUSrcAddr   uint16
	VCUDstAddr   uint16
	EnableUDP    bool
	EnableTCP    bool
}

type DiagnosticService struct {
	config    ServiceConfig
	engine    *battery.Engine
	server    *api.Server
	udpConn   net.PacketConn
	tcpLn     net.Listener
	isoParser *iso15765.Parser
	scanner   *doip.ZeroCopyScanner
}

func main() {
	cfg := parseFlags()
	log.Printf("800V SiC VCU EIS-DOIP Diagnostic Microservice starting...")
	log.Printf("UDP bind: %s (enabled: %v)", cfg.UDPBindAddr, cfg.EnableUDP)
	log.Printf("TCP bind: %s (enabled: %v)", cfg.TCPBindAddr, cfg.EnableTCP)
	log.Printf("HTTP API: %s", cfg.HTTPAddr)
	log.Printf("VCU Source Addr: 0x%04X, Target Addr: 0x%04X", cfg.VCUSrcAddr, cfg.VCUDstAddr)

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
	enableUDP := flag.Bool("enable-udp", false, "Enable DoIP UDP listener")
	enableTCP := flag.Bool("enable-tcp", false, "Enable DoIP TCP listener")

	flag.Parse()

	cfg.UDPBindAddr = *udpAddr
	cfg.TCPBindAddr = *tcpAddr
	cfg.HTTPAddr = *httpAddr
	cfg.VCUSrcAddr = uint16(*srcAddr)
	cfg.VCUDstAddr = uint16(*dstAddr)
	cfg.EnableUDP = *enableUDP
	cfg.EnableTCP = *enableTCP

	return cfg
}

func NewDiagnosticService(cfg ServiceConfig) *DiagnosticService {
	engineCfg := battery.DefaultEngineConfig()
	engineCfg.SampleRate = 10000.0
	engineCfg.SampleBufferSize = 65536
	engineCfg.AutoFit = true

	engine := battery.NewEngine(engineCfg)
	server := api.NewServer(engine)

	isoParser := iso15765.NewParser(nil)

	return &DiagnosticService{
		config:    cfg,
		engine:    engine,
		server:    server,
		isoParser: isoParser,
	}
}

func (svc *DiagnosticService) Start() error {
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
			log.Printf("HTTP server error: %v", err)
		}
	}()

	time.Sleep(200 * time.Millisecond)
	log.Printf("Service started successfully")
	return nil
}

func (svc *DiagnosticService) startUDPListener() error {
	conn, err := net.ListenPacket("udp", svc.config.UDPBindAddr)
	if err != nil {
		return fmt.Errorf("udp bind failed: %w", err)
	}
	svc.udpConn = conn

	scanner := doip.NewZeroCopyScanner(conn, func(payload []byte, srcAddr net.Addr, ts time.Time) {
		svc.server.IncrDoIP()
		svc.handleDoIPPayload(payload, srcAddr, ts)
	})
	svc.scanner = scanner
	svc.server.SetDoIPScanner(scanner)

	if err := scanner.Start(); err != nil {
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
	svc.tcpLn = ln

	go func() {
		for {
			conn, err := ln.Accept()
			if err != nil {
				log.Printf("TCP accept error: %v", err)
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
	defer conn.Close()

	scanner := doip.NewTCPStreamScanner(conn, func(payload []byte, srcAddr net.Addr, ts time.Time) {
		svc.server.IncrDoIP()
		svc.handleDoIPPayload(payload, srcAddr, ts)
	})

	if err := scanner.Start(); err != nil {
		log.Printf("TCP scanner start error: %v", err)
		return
	}

	buf := make([]byte, 4096)
	for {
		_ = conn.SetReadDeadline(time.Now().Add(30 * time.Second))
		_, err := conn.Read(buf)
		if err != nil {
			scanner.Stop()
			return
		}
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

	if len(diagData) > 0 {
		_ = svc.isoParser.ProcessFrame(diagData, src, dst, ts)
	}
}

func (svc *DiagnosticService) Stop() {
	log.Printf("Stopping diagnostic service...")

	if svc.scanner != nil {
		svc.scanner.Stop()
	}
	if svc.udpConn != nil {
		_ = svc.udpConn.Close()
	}
	if svc.tcpLn != nil {
		_ = svc.tcpLn.Close()
	}

	svc.engine.Reset()
	log.Printf("Service stopped")
}
