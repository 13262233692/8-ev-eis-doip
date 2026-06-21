package thermal

import (
	"encoding/binary"
	"sync"
	"time"
)

const (
	ContactorCmdEmergencyOpen  uint8 = 0x01
	ContactorCmdClose          uint8 = 0x02
	ContactorCmdQueryState     uint8 = 0x03

	UDSServiceID         uint8 = 0x2E
	UDSContactorDIDHigh  uint8 = 0xF1
	UDSContactorDIDLow   uint8 = 0x90

	ContactorPriorityEmergency uint8 = 0x00
	ContactorPriorityHigh      uint8 = 0x01
	ContactorPriorityNormal    uint8 = 0x02

	ContactorRetries    = 3
	ContactorRetryDelay = 50 * time.Millisecond
)

type ContactorState int

const (
	ContactorUnknown ContactorState = iota
	ContactorOpen
	ContactorClosed
	ContactorStuck
	ContactorTransitioning
)

type ContactorCommand struct {
	CommandID   uint8
	Priority    uint8
	Timestamp   time.Time
	SourceAddr  uint16
	TargetAddr  uint16
	RetryCount  uint8
}

type ContactorResponse struct {
	Acknowledged bool
	State        ContactorState
	Timestamp    time.Time
	Latency      time.Duration
}

type ContactorManager struct {
	mu            sync.RWMutex
	state         ContactorState
	lastCmd       *ContactorCommand
	lastResp      *ContactorResponse
	cmdCount      uint64
	emergencyN    uint64
	sourceAddr    uint16
	targetAddr    uint16
	onSend        func(frame []byte, priority uint8) error
	hvdcAddr      uint16
}

func NewContactorManager(srcAddr, tgtAddr, hvdcAddr uint16) *ContactorManager {
	return &ContactorManager{
		state:      ContactorUnknown,
		sourceAddr: srcAddr,
		targetAddr: tgtAddr,
		hvdcAddr:   hvdcAddr,
	}
}

func (cm *ContactorManager) SetSendCallback(cb func(frame []byte, priority uint8) error) {
	cm.mu.Lock()
	defer cm.mu.Unlock()
	cm.onSend = cb
}

func (cm *ContactorManager) EmergencyOpen() (*ContactorResponse, error) {
	cm.mu.Lock()
	defer cm.mu.Unlock()

	cmd := &ContactorCommand{
		CommandID:  ContactorCmdEmergencyOpen,
		Priority:   ContactorPriorityEmergency,
		Timestamp:  time.Now(),
		SourceAddr: cm.sourceAddr,
		TargetAddr: cm.hvdcAddr,
		RetryCount: 0,
	}

	cm.lastCmd = cmd
	cm.emergencyN++
	cm.cmdCount++

	frame := cm.buildContactorUDSFrame(cmd)

	var lastErr error
	for attempt := uint8(0); attempt < ContactorRetries; attempt++ {
		cmd.RetryCount = attempt
		if cm.onSend != nil {
			if err := cm.onSend(frame, cmd.Priority); err != nil {
				lastErr = err
				if attempt < ContactorRetries-1 {
					time.Sleep(ContactorRetryDelay)
				}
				continue
			}
		}
		lastErr = nil
		break
	}

	resp := &ContactorResponse{
		Acknowledged: lastErr == nil,
		State:        ContactorOpen,
		Timestamp:    time.Now(),
	}
	if lastErr == nil {
		cm.state = ContactorOpen
	}
	cm.lastResp = resp

	return resp, lastErr
}

func (cm *ContactorManager) Close() (*ContactorResponse, error) {
	cm.mu.Lock()
	defer cm.mu.Unlock()

	cmd := &ContactorCommand{
		CommandID:  ContactorCmdClose,
		Priority:   ContactorPriorityNormal,
		Timestamp:  time.Now(),
		SourceAddr: cm.sourceAddr,
		TargetAddr: cm.hvdcAddr,
	}
	cm.lastCmd = cmd
	cm.cmdCount++

	frame := cm.buildContactorUDSFrame(cmd)

	var lastErr error
	if cm.onSend != nil {
		lastErr = cm.onSend(frame, cmd.Priority)
	}

	cm.state = ContactorClosed
	resp := &ContactorResponse{
		Acknowledged: lastErr == nil,
		State:        ContactorClosed,
		Timestamp:    time.Now(),
	}
	cm.lastResp = resp

	return resp, lastErr
}

func (cm *ContactorManager) buildContactorUDSFrame(cmd *ContactorCommand) []byte {
	udsData := []byte{
		UDSServiceID,
		UDSContactorDIDHigh,
		UDSContactorDIDLow,
		cmd.CommandID,
		cmd.Priority,
		0x00,
		0x00,
	}

	isoFrame := make([]byte, 1+len(udsData))
	isoFrame[0] = 0x00 | uint8(len(udsData))
	copy(isoFrame[1:], udsData)

	doipPayload := make([]byte, 5+len(isoFrame))
	binary.BigEndian.PutUint16(doipPayload[0:2], cmd.SourceAddr)
	binary.BigEndian.PutUint16(doipPayload[2:4], cmd.TargetAddr)
	doipPayload[4] = uint8(len(isoFrame))
	copy(doipPayload[5:], isoFrame)

	return doipPayload
}

func (cm *ContactorManager) State() ContactorState {
	cm.mu.RLock()
	defer cm.mu.RUnlock()
	return cm.state
}

func (cm *ContactorManager) LastCommand() *ContactorCommand {
	cm.mu.RLock()
	defer cm.mu.RUnlock()
	if cm.lastCmd == nil {
		return nil
	}
	cp := *cm.lastCmd
	return &cp
}

func (cm *ContactorManager) LastResponse() *ContactorResponse {
	cm.mu.RLock()
	defer cm.mu.RUnlock()
	if cm.lastResp == nil {
		return nil
	}
	cp := *cm.lastResp
	return &cp
}

func (cm *ContactorManager) Stats() (cmdCount, emergencyCount uint64) {
	cm.mu.RLock()
	defer cm.mu.RUnlock()
	return cm.cmdCount, cm.emergencyN
}

func ContactorStateString(s ContactorState) string {
	switch s {
	case ContactorUnknown:
		return "UNKNOWN"
	case ContactorOpen:
		return "OPEN"
	case ContactorClosed:
		return "CLOSED"
	case ContactorStuck:
		return "STUCK"
	case ContactorTransitioning:
		return "TRANSITIONING"
	default:
		return "INVALID"
	}
}
