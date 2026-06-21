package doip

import (
	"encoding/binary"
	"errors"
	"io"
	"net"
	"sync"
	"sync/atomic"
	"time"
)

const (
	DoIPHeaderLen    = 8
	DoIPMaxFrame     = 1 << 22
	DoIPProtocolVer  = 0x02
	DoIPInverseVer   = 0xFD
	DefaultRingSize  = 1 << 20
)

type FrameCallback func(payload []byte, srcAddr net.Addr, ts time.Time)

type ZeroCopyScanner struct {
	conn         net.PacketConn
	bufPool      sync.Pool
	callback     FrameCallback
	running      atomic.Bool
	readDeadline time.Duration
	stats        ScannerStats
	mu           sync.RWMutex
}

type ScannerStats struct {
	FramesRx    uint64
	BytesRx     uint64
	FramesErr   uint64
	FramesDropped uint64
}

type FrameHeader struct {
	ProtocolVersion    uint8
	InverseVersion     uint8
	PayloadType        uint16
	PayloadLength      uint32
}

func NewZeroCopyScanner(conn net.PacketConn, cb FrameCallback) *ZeroCopyScanner {
	return &ZeroCopyScanner{
		conn:         conn,
		callback:     cb,
		readDeadline: 100 * time.Millisecond,
		bufPool: sync.Pool{
			New: func() interface{} {
				b := make([]byte, DoIPMaxFrame)
				return &b
			},
		},
	}
}

func (s *ZeroCopyScanner) SetReadDeadline(d time.Duration) {
	s.readDeadline = d
}

func (s *ZeroCopyScanner) Start() error {
	if !s.running.CompareAndSwap(false, true) {
		return errors.New("scanner already running")
	}
	go s.scanLoop()
	return nil
}

func (s *ZeroCopyScanner) Stop() {
	s.running.Store(false)
}

func (s *ZeroCopyScanner) Stats() ScannerStats {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.stats
}

func (s *ZeroCopyScanner) scanLoop() {
	for s.running.Load() {
		bufPtr := s.bufPool.Get().(*[]byte)
		buf := *bufPtr

		if s.readDeadline > 0 {
			_ = s.conn.SetReadDeadline(time.Now().Add(s.readDeadline))
		}

		n, src, err := s.conn.ReadFrom(buf)
		if err != nil {
			var netErr net.Error
			if errors.As(err, &netErr) && netErr.Timeout() {
				s.bufPool.Put(bufPtr)
				continue
			}
			s.incError()
			s.bufPool.Put(bufPtr)
			continue
		}

		if n < DoIPHeaderLen {
			s.incError()
			s.bufPool.Put(bufPtr)
			continue
		}

		hdr, payload, ok := s.parseFrameHeader(buf[:n])
		if !ok {
			s.incError()
			s.bufPool.Put(bufPtr)
			continue
		}

		if int(hdr.PayloadLength) != len(payload) {
			s.incDropped()
			s.bufPool.Put(bufPtr)
			continue
		}

		s.incRx(uint64(n))
		ts := time.Now()

		go func(p []byte, a net.Addr, t time.Time, bp *[]byte) {
			defer s.bufPool.Put(bp)
			if s.callback != nil {
				s.callback(p, a, t)
			}
		}(payload, src, ts, bufPtr)
	}
}

func (s *ZeroCopyScanner) parseFrameHeader(data []byte) (FrameHeader, []byte, bool) {
	var hdr FrameHeader
	if len(data) < DoIPHeaderLen {
		return hdr, nil, false
	}

	hdr.ProtocolVersion = data[0]
	hdr.InverseVersion = data[1]

	if hdr.ProtocolVersion != DoIPProtocolVer || hdr.InverseVersion != DoIPInverseVer {
		return hdr, nil, false
	}

	hdr.PayloadType = binary.BigEndian.Uint16(data[2:4])
	hdr.PayloadLength = binary.BigEndian.Uint32(data[4:8])

	payload := data[DoIPHeaderLen:]
	return hdr, payload, true
}

func (s *ZeroCopyScanner) incRx(n uint64) {
	s.mu.Lock()
	s.stats.FramesRx++
	s.stats.BytesRx += n
	s.mu.Unlock()
}

func (s *ZeroCopyScanner) incError() {
	s.mu.Lock()
	s.stats.FramesErr++
	s.mu.Unlock()
}

func (s *ZeroCopyScanner) incDropped() {
	s.mu.Lock()
	s.stats.FramesDropped++
	s.mu.Unlock()
}

type TCPStreamScanner struct {
	conn     net.Conn
	running  atomic.Bool
	buffer   []byte
	offset   int
	callback FrameCallback
	mu       sync.Mutex
}

func NewTCPStreamScanner(conn net.Conn, cb FrameCallback) *TCPStreamScanner {
	return &TCPStreamScanner{
		conn:     conn,
		callback: cb,
		buffer:   make([]byte, DoIPMaxFrame*2),
	}
}

func (t *TCPStreamScanner) Start() error {
	if !t.running.CompareAndSwap(false, true) {
		return errors.New("scanner already running")
	}
	go t.readLoop()
	return nil
}

func (t *TCPStreamScanner) Stop() {
	t.running.Store(false)
	_ = t.conn.Close()
}

func (t *TCPStreamScanner) readLoop() {
	for t.running.Load() {
		if t.offset >= len(t.buffer) {
			t.growBuffer()
		}

		n, err := t.conn.Read(t.buffer[t.offset:])
		if err != nil {
			if err == io.EOF {
				return
			}
			continue
		}
		t.offset += n

		t.processFrames()
	}
}

func (t *TCPStreamScanner) processFrames() {
	t.mu.Lock()
	defer t.mu.Unlock()

	for t.offset >= DoIPHeaderLen {
		if t.buffer[0] != DoIPProtocolVer || t.buffer[1] != DoIPInverseVer {
			copy(t.buffer, t.buffer[1:t.offset])
			t.offset--
			continue
		}

		payloadLen := binary.BigEndian.Uint32(t.buffer[4:8])
		totalLen := DoIPHeaderLen + int(payloadLen)

		if t.offset < totalLen {
			break
		}

		hdr := FrameHeader{
			ProtocolVersion: t.buffer[0],
			InverseVersion:  t.buffer[1],
			PayloadType:     binary.BigEndian.Uint16(t.buffer[2:4]),
			PayloadLength:   payloadLen,
		}

		payload := make([]byte, payloadLen)
		copy(payload, t.buffer[DoIPHeaderLen:totalLen])

		if t.callback != nil {
			t.callback(payload, t.conn.RemoteAddr(), time.Now())
		}
		_ = hdr

		copy(t.buffer, t.buffer[totalLen:t.offset])
		t.offset -= totalLen
	}
}

func (t *TCPStreamScanner) growBuffer() {
	newBuf := make([]byte, len(t.buffer)*2)
	copy(newBuf, t.buffer[:t.offset])
	t.buffer = newBuf
}
