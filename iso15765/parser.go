package iso15765

import (
	"encoding/binary"
	"errors"
	"sync"
	"time"
)

const (
	FrameTypeMask      = 0xF0
	FrameTypeSingle    = 0x00
	FrameTypeFirst     = 0x10
	FrameTypeConsec    = 0x20
	FrameTypeFlow      = 0x30

	FlowStatusClear    = 0x00
	FlowStatusWait     = 0x01
	FlowStatusOverflow = 0x02

	MaxPayloadSize     = 4 << 20
	DefaultBlockSize   = 0
	DefaultSTmin       = 0x14
	MaxCFCount         = 4096

	DefaultTimeout     = 1500 * time.Millisecond
)

type FrameType uint8

type ReassembledMessage struct {
	Data      []byte
	Timestamp time.Time
	SrcAddr   uint16
	DstAddr   uint16
	Complete  bool
}

type MessageCallback func(msg *ReassembledMessage)

type flowControlState int

const (
	flowWait flowControlState = iota
	flowClear
	flowOverflow
)

type sessionState int

const (
	sessionIdle sessionState = iota
	sessionReceiving
	sessionTimeout
)

type activeSession struct {
	mu              sync.Mutex
	state           sessionState
	flowState       flowControlState
	totalLength     uint32
	received        uint32
	sequenceNumber  uint8
	blockSize       uint8
	blockCount      uint8
	stMin           time.Duration
	lastFrameTime   time.Time
	buffer          []byte
	timeoutTimer    *time.Timer
	srcAddr         uint16
	dstAddr         uint16
	startTime       time.Time
}

type Parser struct {
	sessions   map[uint32]*activeSession
	mu         sync.RWMutex
	callback   MessageCallback
	timeout    time.Duration
	maxPayload uint32
}

func NewParser(cb MessageCallback) *Parser {
	return &Parser{
		sessions:   make(map[uint32]*activeSession),
		callback:   cb,
		timeout:    DefaultTimeout,
		maxPayload: MaxPayloadSize,
	}
}

func (p *Parser) SetTimeout(d time.Duration) {
	p.timeout = d
}

func (p *Parser) SetMaxPayload(sz uint32) {
	p.maxPayload = sz
}

func (p *Parser) ProcessFrame(data []byte, srcAddr, dstAddr uint16, ts time.Time) error {
	if len(data) < 1 {
		return errors.New("iso15765: frame too short")
	}

	pciType := data[0] & FrameTypeMask

	switch pciType {
	case FrameTypeSingle:
		return p.handleSingleFrame(data, srcAddr, dstAddr, ts)
	case FrameTypeFirst:
		return p.handleFirstFrame(data, srcAddr, dstAddr, ts)
	case FrameTypeConsec:
		return p.handleConsecutiveFrame(data, srcAddr, dstAddr, ts)
	case FrameTypeFlow:
		return p.handleFlowControl(data, srcAddr, dstAddr, ts)
	default:
		return errors.New("iso15765: unknown frame type")
	}
}

func sessionKey(src, dst uint16) uint32 {
	return uint32(src)<<16 | uint32(dst)
}

func (p *Parser) handleSingleFrame(data []byte, src, dst uint16, ts time.Time) error {
	dl := int(data[0] & 0x0F)

	var payload []byte
	if dl == 0 && len(data) > 1 {
		if len(data) < 2 {
			return errors.New("iso15765: malformed SF with extended length")
		}
		dl = int(data[1])
		if len(data) < 2+dl {
			return errors.New("iso15765: SF payload truncated")
		}
		payload = make([]byte, dl)
		copy(payload, data[2:2+dl])
	} else {
		if dl > len(data)-1 {
			dl = len(data) - 1
		}
		payload = make([]byte, dl)
		copy(payload, data[1:1+dl])
	}

	if p.callback != nil {
		p.callback(&ReassembledMessage{
			Data:      payload,
			Timestamp: ts,
			SrcAddr:   src,
			DstAddr:   dst,
			Complete:  true,
		})
	}
	return nil
}

func (p *Parser) handleFirstFrame(data []byte, src, dst uint16, ts time.Time) error {
	if len(data) < 2 {
		return errors.New("iso15765: FF too short")
	}

	var totalLen uint32
	highNibble := uint32(data[0] & 0x0F)
	totalLen = (highNibble << 8) | uint32(data[1])

	offset := 2
	if totalLen == 0 {
		if len(data) < 6 {
			return errors.New("iso15765: FF with extended length too short")
		}
		totalLen = binary.BigEndian.Uint32(data[2:6])
		offset = 6
	}

	if totalLen > p.maxPayload {
		return errors.New("iso15765: message exceeds maximum payload size")
	}

	key := sessionKey(src, dst)

	p.mu.Lock()
	if existing, ok := p.sessions[key]; ok {
		if existing.timeoutTimer != nil {
			existing.timeoutTimer.Stop()
		}
	}

	sess := &activeSession{
		state:          sessionReceiving,
		flowState:      flowClear,
		totalLength:    totalLen,
		received:       0,
		sequenceNumber: 0,
		blockSize:      DefaultBlockSize,
		stMin:          time.Duration(DefaultSTmin) * time.Millisecond,
		lastFrameTime:  ts,
		buffer:         make([]byte, totalLen),
		srcAddr:        src,
		dstAddr:        dst,
		startTime:      ts,
	}

	initialData := data[offset:]
	if len(initialData) > 0 {
		n := copy(sess.buffer, initialData)
		sess.received = uint32(n)
	}

	sess.sequenceNumber = 1
	sess.timeoutTimer = time.AfterFunc(p.timeout, func() {
		p.timeoutSession(key)
	})

	p.sessions[key] = sess
	p.mu.Unlock()

	if sess.received >= sess.totalLength {
		p.completeSession(key)
	}

	return nil
}

func (p *Parser) handleConsecutiveFrame(data []byte, src, dst uint16, ts time.Time) error {
	if len(data) < 1 {
		return errors.New("iso15765: CF too short")
	}

	seqNum := data[0] & 0x0F
	key := sessionKey(src, dst)

	p.mu.RLock()
	sess, ok := p.sessions[key]
	p.mu.RUnlock()

	if !ok {
		return errors.New("iso15765: no active session for CF")
	}

	sess.mu.Lock()
	defer sess.mu.Unlock()

	if sess.state != sessionReceiving {
		return errors.New("iso15765: session not in receiving state")
	}

	if seqNum != sess.sequenceNumber {
		sess.state = sessionTimeout
		p.cleanupSession(key)
		return errors.New("iso15765: sequence number mismatch")
	}

	sess.sequenceNumber = (sess.sequenceNumber + 1) % 16
	sess.lastFrameTime = ts

	payload := data[1:]
	if len(payload) > 0 {
		remaining := sess.totalLength - sess.received
		n := uint32(len(payload))
		if n > remaining {
			n = remaining
		}
		copy(sess.buffer[sess.received:], payload[:n])
		sess.received += n
	}

	sess.blockCount++
	if sess.blockSize > 0 && sess.blockCount >= sess.blockSize {
		sess.flowState = flowWait
		sess.blockCount = 0
	}

	if sess.timeoutTimer != nil {
		sess.timeoutTimer.Reset(p.timeout)
	}

	if sess.received >= sess.totalLength {
		sess.mu.Unlock()
		p.completeSession(key)
		sess.mu.Lock()
	}

	return nil
}

func (p *Parser) handleFlowControl(data []byte, src, dst uint16, ts time.Time) error {
	if len(data) < 3 {
		return errors.New("iso15765: FC too short")
	}

	_ = ts
	key := sessionKey(dst, src)
	p.mu.RLock()
	sess, ok := p.sessions[key]
	p.mu.RUnlock()

	if !ok {
		return nil
	}

	sess.mu.Lock()
	defer sess.mu.Unlock()

	status := data[0] & 0x0F
	sess.blockSize = data[1]
	stMinRaw := data[2]

	switch {
	case stMinRaw <= 0x7F:
		sess.stMin = time.Duration(stMinRaw) * time.Millisecond
	case stMinRaw >= 0xF1 && stMinRaw <= 0xF9:
		sess.stMin = time.Duration(stMinRaw-0xF0) * 100 * time.Microsecond
	default:
		sess.stMin = time.Duration(DefaultSTmin) * time.Millisecond
	}

	switch status {
	case FlowStatusClear:
		sess.flowState = flowClear
		sess.blockCount = 0
	case FlowStatusWait:
		sess.flowState = flowWait
	case FlowStatusOverflow:
		sess.flowState = flowOverflow
		sess.state = sessionTimeout
		p.cleanupSession(key)
	}

	return nil
}

func (p *Parser) timeoutSession(key uint32) {
	p.mu.Lock()
	sess, ok := p.sessions[key]
	if ok {
		sess.mu.Lock()
		sess.state = sessionTimeout
		sess.mu.Unlock()
		delete(p.sessions, key)
	}
	p.mu.Unlock()
}

func (p *Parser) completeSession(key uint32) {
	p.mu.Lock()
	sess, ok := p.sessions[key]
	if !ok {
		p.mu.Unlock()
		return
	}

	delete(p.sessions, key)
	p.mu.Unlock()

	if sess.timeoutTimer != nil {
		sess.timeoutTimer.Stop()
	}

	if p.callback != nil {
		finalData := make([]byte, sess.received)
		copy(finalData, sess.buffer[:sess.received])
		p.callback(&ReassembledMessage{
			Data:      finalData,
			Timestamp: sess.lastFrameTime,
			SrcAddr:   sess.srcAddr,
			DstAddr:   sess.dstAddr,
			Complete:  true,
		})
	}
}

func (p *Parser) cleanupSession(key uint32) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if sess, ok := p.sessions[key]; ok {
		if sess.timeoutTimer != nil {
			sess.timeoutTimer.Stop()
		}
		delete(p.sessions, key)
	}
}

func (p *Parser) ActiveSessions() int {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return len(p.sessions)
}

func EncodeSingleFrame(payload []byte) ([]byte, error) {
	if len(payload) <= 7 {
		frame := make([]byte, 1+len(payload))
		frame[0] = FrameTypeSingle | uint8(len(payload))
		copy(frame[1:], payload)
		return frame, nil
	}

	if len(payload) <= 255 {
		frame := make([]byte, 2+len(payload))
		frame[0] = FrameTypeSingle
		frame[1] = uint8(len(payload))
		copy(frame[2:], payload)
		return frame, nil
	}

	return nil, errors.New("iso15765: payload too large for single frame")
}

func EncodeFlowControl(status uint8, blockSize uint8, stMin uint8) []byte {
	frame := make([]byte, 3)
	frame[0] = FrameTypeFlow | (status & 0x0F)
	frame[1] = blockSize
	frame[2] = stMin
	return frame
}

func SplitIntoFrames(payload []byte, maxFrameLen int) ([][]byte, error) {
	if len(payload) <= 7 {
		sf, err := EncodeSingleFrame(payload)
		if err != nil {
			return nil, err
		}
		return [][]byte{sf}, nil
	}

	if maxFrameLen < 8 {
		maxFrameLen = 8
	}
	cfDataLen := maxFrameLen - 1

	var frames [][]byte

	var ff []byte
	if len(payload) <= 4095 {
		ff = make([]byte, 2)
		ff[0] = FrameTypeFirst | uint8((len(payload)>>8)&0x0F)
		ff[1] = uint8(len(payload) & 0xFF)
	} else {
		ff = make([]byte, 6)
		ff[0] = FrameTypeFirst
		ff[1] = 0
		binary.BigEndian.PutUint32(ff[2:6], uint32(len(payload)))
	}

	initialLen := maxFrameLen - len(ff)
	if initialLen > len(payload) {
		initialLen = len(payload)
	}
	ff = append(ff, payload[:initialLen]...)
	frames = append(frames, ff)

	remaining := payload[initialLen:]
	seqNum := uint8(1)

	for len(remaining) > 0 {
		chunk := cfDataLen
		if chunk > len(remaining) {
			chunk = len(remaining)
		}

		cf := make([]byte, 1+chunk)
		cf[0] = FrameTypeConsec | (seqNum & 0x0F)
		copy(cf[1:], remaining[:chunk])
		frames = append(frames, cf)

		remaining = remaining[chunk:]
		seqNum = (seqNum + 1) % 16
	}

	return frames, nil
}
