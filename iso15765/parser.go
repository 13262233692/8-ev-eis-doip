package iso15765

import (
	"encoding/binary"
	"errors"
	"sync"
	"sync/atomic"
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

	MaxPayloadSize     = 10 << 20
	DefaultBlockSize   = 0
	DefaultSTmin       = 0x14
	MaxCFCount         = 4096

	DefaultTimeout     = 1500 * time.Millisecond

	LookaheadBufferSize = 64
	MaxLookaheadRetries = 32
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

type FlowControlCallback func(srcAddr, dstAddr uint16, status, blockSize, stMin uint8)

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
	sessionCompleted
)

type cfEntry struct {
	seqNum uint8
	data   []byte
	ts     time.Time
	used   bool
}

type lookaheadBuffer struct {
	buffer  [LookaheadBufferSize]cfEntry
	head    uint32
	tail    uint32
	count   uint32
	mu      sync.Mutex
}

type activeSession struct {
	mu              sync.Mutex
	state           int32
	flowState       flowControlState
	totalLength     uint32
	received        uint32
	expectedSeq     uint8
	blockSize       uint8
	blockCount      uint8
	stMin           time.Duration
	lastFrameTime   time.Time
	buffer          []byte
	timeoutTimer    *time.Timer
	srcAddr         uint16
	dstAddr         uint16
	startTime       time.Time
	lookahead       lookaheadBuffer
	lastFCTime      time.Time
	fcSentCount     uint32
}

type Parser struct {
	sessions        map[uint32]*activeSession
	mu              sync.RWMutex
	callback        MessageCallback
	fcCallback      FlowControlCallback
	timeout         time.Duration
	maxPayload      uint32
	autoSendFC      bool
	defaultBlockSize uint8
	defaultSTmin    uint8
	stats           ParserStats
}

type ParserStats struct {
	SessionsCreated   uint64
	SessionsCompleted uint64
	SessionsTimeout   uint64
	SessionsAborted   uint64
	FramesSF          uint64
	FramesFF          uint64
	FramesCF          uint64
	FramesFC          uint64
	FramesOOO         uint64
	FramesDropped     uint64
}

func NewParser(cb MessageCallback) *Parser {
	return &Parser{
		sessions:         make(map[uint32]*activeSession),
		callback:         cb,
		timeout:          DefaultTimeout,
		maxPayload:       MaxPayloadSize,
		autoSendFC:       true,
		defaultBlockSize: DefaultBlockSize,
		defaultSTmin:    DefaultSTmin,
	}
}

func (p *Parser) SetFlowControlCallback(fc FlowControlCallback) {
	p.fcCallback = fc
}

func (p *Parser) SetAutoSendFC(enabled bool, blockSize uint8, stMin uint8) {
	p.autoSendFC = enabled
	p.defaultBlockSize = blockSize
	p.defaultSTmin = stMin
}

func (p *Parser) SetTimeout(d time.Duration) {
	p.timeout = d
}

func (p *Parser) SetMaxPayload(sz uint32) {
	p.maxPayload = sz
}

func (p *Parser) Stats() ParserStats {
	return ParserStats{
		SessionsCreated:   atomic.LoadUint64(&p.stats.SessionsCreated),
		SessionsCompleted: atomic.LoadUint64(&p.stats.SessionsCompleted),
		SessionsTimeout:   atomic.LoadUint64(&p.stats.SessionsTimeout),
		SessionsAborted:   atomic.LoadUint64(&p.stats.SessionsAborted),
		FramesSF:          atomic.LoadUint64(&p.stats.FramesSF),
		FramesFF:          atomic.LoadUint64(&p.stats.FramesFF),
		FramesCF:          atomic.LoadUint64(&p.stats.FramesCF),
		FramesFC:          atomic.LoadUint64(&p.stats.FramesFC),
		FramesOOO:         atomic.LoadUint64(&p.stats.FramesOOO),
		FramesDropped:     atomic.LoadUint64(&p.stats.FramesDropped),
	}
}

func (p *Parser) ProcessFrame(data []byte, srcAddr, dstAddr uint16, ts time.Time) error {
	if len(data) < 1 {
		return errors.New("iso15765: frame too short")
	}

	pciType := data[0] & FrameTypeMask

	switch pciType {
	case FrameTypeSingle:
		atomic.AddUint64(&p.stats.FramesSF, 1)
		return p.handleSingleFrame(data, srcAddr, dstAddr, ts)
	case FrameTypeFirst:
		atomic.AddUint64(&p.stats.FramesFF, 1)
		return p.handleFirstFrame(data, srcAddr, dstAddr, ts)
	case FrameTypeConsec:
		atomic.AddUint64(&p.stats.FramesCF, 1)
		return p.handleConsecutiveFrame(data, srcAddr, dstAddr, ts)
	case FrameTypeFlow:
		atomic.AddUint64(&p.stats.FramesFC, 1)
		return p.handleFlowControl(data, srcAddr, dstAddr, ts)
	default:
		atomic.AddUint64(&p.stats.FramesDropped, 1)
		return errors.New("iso15765: unknown frame type")
	}
}

func sessionKey(src, dst uint16) uint32 {
	return uint32(src)<<16 | uint32(dst)
}

func seqDistance(expected, actual uint8) int {
	diff := int(actual) - int(expected)
	if diff < -8 {
		diff += 16
	} else if diff > 8 {
		diff -= 16
	}
	return diff
}

func (p *Parser) handleSingleFrame(data []byte, src, dst uint16, ts time.Time) error {
	dl := int(data[0] & 0x0F)

	var payload []byte
	if dl == 0 && len(data) > 1 {
		if len(data) < 2 {
			return errors.New("iso15765: malformed SF with extended length")
		}
		dl = int(data[1])
		if dl < 0 || 2+dl > len(data) {
			if len(data) < 2 {
				return errors.New("iso15765: SF payload truncated")
			}
			dl = len(data) - 2
		}
		payload = make([]byte, dl)
		copy(payload, data[2:2+dl])
	} else {
		if dl > len(data)-1 {
			dl = len(data) - 1
		}
		if dl < 0 {
			dl = 0
		}
		payload = make([]byte, dl)
		if dl > 0 {
			copy(payload, data[1:1+dl])
		}
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

	if totalLen == 0 {
		return errors.New("iso15765: FF with zero length")
	}

	key := sessionKey(src, dst)

	p.mu.Lock()
	if existing, ok := p.sessions[key]; ok {
		existing.mu.Lock()
		if existing.timeoutTimer != nil {
			existing.timeoutTimer.Stop()
			existing.timeoutTimer = nil
		}
		atomic.StoreInt32(&existing.state, int32(sessionTimeout))
		existing.mu.Unlock()
		atomic.AddUint64(&p.stats.SessionsAborted, 1)
		delete(p.sessions, key)
	}

	sess := &activeSession{
		state:          int32(sessionReceiving),
		flowState:      flowClear,
		totalLength:    totalLen,
		received:       0,
		expectedSeq:    1,
		blockSize:      p.defaultBlockSize,
		stMin:          time.Duration(p.defaultSTmin) * time.Millisecond,
		lastFrameTime:  ts,
		buffer:         make([]byte, totalLen),
		srcAddr:        src,
		dstAddr:        dst,
		startTime:      ts,
	}

	initialData := data[offset:]
	if len(initialData) > 0 && totalLen > 0 {
		n := len(initialData)
		if uint32(n) > totalLen {
			n = int(totalLen)
		}
		copy(sess.buffer, initialData[:n])
		sess.received = uint32(n)
	}

	sess.timeoutTimer = time.AfterFunc(p.timeout, func() {
		p.timeoutSession(key)
	})

	p.sessions[key] = sess
	atomic.AddUint64(&p.stats.SessionsCreated, 1)
	p.mu.Unlock()

	if p.autoSendFC && p.fcCallback != nil {
		p.fcCallback(dst, src, FlowStatusClear, p.defaultBlockSize, p.defaultSTmin)
		sess.mu.Lock()
		sess.lastFCTime = ts
		sess.fcSentCount++
		sess.mu.Unlock()
	}

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
		atomic.AddUint64(&p.stats.FramesDropped, 1)
		return errors.New("iso15765: no active session for CF")
	}

	sess.mu.Lock()
	defer sess.mu.Unlock()

	state := atomic.LoadInt32(&sess.state)
	if state != int32(sessionReceiving) {
		atomic.AddUint64(&p.stats.FramesDropped, 1)
		return errors.New("iso15765: session not in receiving state")
	}

	dist := seqDistance(sess.expectedSeq, seqNum)

	if dist == 0 {
		sess.expectedSeq = (sess.expectedSeq + 1) % 16
		sess.lastFrameTime = ts

		payload := data[1:]
		if len(payload) > 0 && sess.received < sess.totalLength {
			remaining := sess.totalLength - sess.received
			n := uint32(len(payload))
			if n > remaining {
				n = remaining
			}
			if sess.received+n > sess.totalLength {
				n = sess.totalLength - sess.received
			}
			if n > 0 {
				copy(sess.buffer[sess.received:sess.received+n], payload[:n])
				sess.received += n
			}
		}

		sess.blockCount++
		if sess.blockSize > 0 && sess.blockCount >= sess.blockSize {
			sess.flowState = flowWait
			sess.blockCount = 0
		}

		if sess.timeoutTimer != nil {
			sess.timeoutTimer.Reset(p.timeout)
		}

		sess.mu.Unlock()
		p.processLookahead(sess, key)
		sess.mu.Lock()

		if sess.received >= sess.totalLength {
			sess.mu.Unlock()
			p.completeSession(key)
			return nil
		}

		if p.autoSendFC && p.fcCallback != nil &&
			sess.flowState == flowWait &&
			ts.Sub(sess.lastFCTime) > 10*time.Millisecond {
			p.fcCallback(sess.dstAddr, sess.srcAddr, FlowStatusClear, sess.blockSize, p.defaultSTmin)
			sess.lastFCTime = ts
			sess.fcSentCount++
			sess.flowState = flowClear
		}
	} else if dist > 0 && dist < LookaheadBufferSize {
		atomic.AddUint64(&p.stats.FramesOOO, 1)
		p.storeLookahead(sess, seqNum, data[1:], ts)
	} else {
		atomic.AddUint64(&p.stats.FramesDropped, 1)
		if dist < 0 {
			return errors.New("iso15765: duplicate or stale CF frame")
		}
		return errors.New("iso15765: CF too far ahead, dropping")
	}

	return nil
}

func (p *Parser) storeLookahead(sess *activeSession, seqNum uint8, data []byte, ts time.Time) {
	sess.lookahead.mu.Lock()
	defer sess.lookahead.mu.Unlock()

	if sess.lookahead.count >= LookaheadBufferSize {
		sess.lookahead.tail = (sess.lookahead.tail + 1) % LookaheadBufferSize
		sess.lookahead.count--
	}

	idx := sess.lookahead.head
	sess.lookahead.buffer[idx] = cfEntry{
		seqNum: seqNum,
		data:   append([]byte(nil), data...),
		ts:     ts,
		used:   false,
	}
	sess.lookahead.head = (sess.lookahead.head + 1) % LookaheadBufferSize
	sess.lookahead.count++
}

func (p *Parser) processLookahead(sess *activeSession, key uint32) {
	sess.lookahead.mu.Lock()
	defer sess.lookahead.mu.Unlock()

	retries := 0
	for sess.lookahead.count > 0 && retries < MaxLookaheadRetries {
		found := false
		var foundIdx uint32

		for i := uint32(0); i < sess.lookahead.count; i++ {
			idx := (sess.lookahead.tail + i) % LookaheadBufferSize
			entry := &sess.lookahead.buffer[idx]
			if !entry.used && seqDistance(sess.expectedSeq, entry.seqNum) == 0 {
				foundIdx = idx
				found = true
				break
			}
		}

		if !found {
			break
		}

		entry := &sess.lookahead.buffer[foundIdx]
		entry.used = true

		sess.expectedSeq = (sess.expectedSeq + 1) % 16
		sess.lastFrameTime = entry.ts

		if len(entry.data) > 0 && sess.received < sess.totalLength {
			remaining := sess.totalLength - sess.received
			n := uint32(len(entry.data))
			if n > remaining {
				n = remaining
			}
			if sess.received+n > sess.totalLength {
				n = sess.totalLength - sess.received
			}
			if n > 0 {
				copy(sess.buffer[sess.received:sess.received+n], entry.data[:n])
				sess.received += n
			}
		}

		sess.blockCount++
		if sess.blockSize > 0 && sess.blockCount >= sess.blockSize {
			sess.flowState = flowWait
			sess.blockCount = 0
		}

		sess.lookahead.tail = (sess.lookahead.tail + 1) % LookaheadBufferSize
		sess.lookahead.count--

		retries++
	}
}

func (p *Parser) handleFlowControl(data []byte, src, dst uint16, ts time.Time) error {
	if len(data) < 3 {
		return errors.New("iso15765: FC too short")
	}

	key := sessionKey(dst, src)
	p.mu.RLock()
	sess, ok := p.sessions[key]
	p.mu.RUnlock()

	if !ok {
		return nil
	}

	sess.mu.Lock()
	defer sess.mu.Unlock()

	state := atomic.LoadInt32(&sess.state)
	if state != int32(sessionReceiving) {
		return nil
	}

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
		atomic.StoreInt32(&sess.state, int32(sessionTimeout))
		sess.mu.Unlock()
		p.cleanupSession(key)
		sess.mu.Lock()
		atomic.AddUint64(&p.stats.SessionsAborted, 1)
	}

	return nil
}

func (p *Parser) timeoutSession(key uint32) {
	p.mu.Lock()
	sess, ok := p.sessions[key]
	if ok {
		sess.mu.Lock()
		if atomic.CompareAndSwapInt32(&sess.state, int32(sessionReceiving), int32(sessionTimeout)) {
			if sess.timeoutTimer != nil {
				sess.timeoutTimer.Stop()
				sess.timeoutTimer = nil
			}
		}
		sess.mu.Unlock()
		delete(p.sessions, key)
		atomic.AddUint64(&p.stats.SessionsTimeout, 1)
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

	sess.mu.Lock()
	atomic.StoreInt32(&sess.state, int32(sessionCompleted))
	if sess.timeoutTimer != nil {
		sess.timeoutTimer.Stop()
		sess.timeoutTimer = nil
	}

	received := sess.received
	totalLen := sess.totalLength
	buffer := sess.buffer
	lastTime := sess.lastFrameTime
	src := sess.srcAddr
	dst := sess.dstAddr
	sess.mu.Unlock()

	atomic.AddUint64(&p.stats.SessionsCompleted, 1)

	if p.callback != nil {
		if received > totalLen {
			received = totalLen
		}
		if received < 0 {
			received = 0
		}
		finalData := make([]byte, received)
		if received > 0 && totalLen > 0 && received <= totalLen {
			copy(finalData, buffer[:received])
		}
		p.callback(&ReassembledMessage{
			Data:      finalData,
			Timestamp: lastTime,
			SrcAddr:   src,
			DstAddr:   dst,
			Complete:  true,
		})
	}
}

func (p *Parser) cleanupSession(key uint32) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if sess, ok := p.sessions[key]; ok {
		sess.mu.Lock()
		if sess.timeoutTimer != nil {
			sess.timeoutTimer.Stop()
			sess.timeoutTimer = nil
		}
		atomic.StoreInt32(&sess.state, int32(sessionTimeout))
		sess.mu.Unlock()
		delete(p.sessions, key)
	}
}

func (p *Parser) AbortSession(srcAddr, dstAddr uint16) {
	key := sessionKey(srcAddr, dstAddr)
	p.cleanupSession(key)
	atomic.AddUint64(&p.stats.SessionsAborted, 1)
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
