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
	MaxBufferGrowth  = 4
	ConnReadTimeout  = 30 * time.Second
)

type FrameCallback func(payload []byte, srcAddr net.Addr, ts time.Time)

type ZeroCopyScanner struct {
	conn         net.PacketConn
	connClosed   atomic.Bool
	bufPool      sync.Pool
	callback     FrameCallback
	running      atomic.Bool
	readDeadline time.Duration
	stats        ScannerStats
	mu           sync.RWMutex
	stopOnce     sync.Once
}

type ScannerStats struct {
	FramesRx      uint64
	BytesRx       uint64
	FramesErr     uint64
	FramesDropped uint64
}

type FrameHeader struct {
	ProtocolVersion uint8
	InverseVersion  uint8
	PayloadType     uint16
	PayloadLength   uint32
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
	s.stopOnce.Do(func() {
		s.running.Store(false)
		if s.conn != nil && !s.connClosed.Load() {
			s.connClosed.Store(true)
			_ = s.conn.Close()
		}
	})
}

func (s *ZeroCopyScanner) Stats() ScannerStats {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.stats
}

func (s *ZeroCopyScanner) scanLoop() {
	defer s.Stop()

	for s.running.Load() && !s.connClosed.Load() {
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
				if !s.running.Load() || s.connClosed.Load() {
					return
				}
				continue
			}
			if s.running.Load() && !s.connClosed.Load() {
				s.incError()
			}
			s.bufPool.Put(bufPtr)
			if !s.running.Load() || s.connClosed.Load() {
				return
			}
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
			if s.callback != nil && s.running.Load() {
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

	if hdr.PayloadLength > DoIPMaxFrame {
		return hdr, nil, false
	}

	payload := data[DoIPHeaderLen:]
	if int(hdr.PayloadLength) > len(payload) {
		return hdr, nil, false
	}

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
	conn       net.Conn
	connClosed atomic.Bool
	running    atomic.Bool
	buffer     []byte
	offset     int
	capLevel   int
	callback   FrameCallback
	mu         sync.Mutex
	stopOnce   sync.Once
	readDone   chan struct{}
}

func NewTCPStreamScanner(conn net.Conn, cb FrameCallback) *TCPStreamScanner {
	return &TCPStreamScanner{
		conn:     conn,
		callback: cb,
		buffer:   make([]byte, DoIPMaxFrame*2),
		capLevel: 1,
		readDone: make(chan struct{}, 1),
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
	t.stopOnce.Do(func() {
		t.running.Store(false)
		if t.conn != nil && !t.connClosed.Load() {
			t.connClosed.Store(true)
			_ = t.conn.Close()
		}
		select {
		case t.readDone <- struct{}{}:
		default:
		}
	})
}

func (t *TCPStreamScanner) readLoop() {
	defer func() {
		t.Stop()
		select {
		case <-t.readDone:
		default:
		}
	}()

	for t.running.Load() && !t.connClosed.Load() {
		if t.offset >= len(t.buffer) {
			if !t.growBuffer() {
				return
			}
		}

		if !t.running.Load() || t.connClosed.Load() {
			return
		}

		_ = t.conn.SetReadDeadline(time.Now().Add(ConnReadTimeout))

		n, err := t.conn.Read(t.buffer[t.offset:])
		if err != nil {
			if !t.running.Load() || t.connClosed.Load() {
				return
			}

			var netErr net.Error
			if errors.As(err, &netErr) && netErr.Timeout() {
				continue
			}

			if err == io.EOF {
				return
			}

			if opErr, ok := err.(*net.OpError); ok {
				if opErr.Err != nil && (opErr.Err.Error() == "use of closed network connection" ||
					opErr.Err.Error() == "operation canceled") {
					return
				}
			}

			if !t.running.Load() || t.connClosed.Load() {
				return
			}

			time.Sleep(10 * time.Millisecond)
			continue
		}

		if n <= 0 {
			if !t.running.Load() || t.connClosed.Load() {
				return
			}
			continue
		}

		t.offset += n

		t.mu.Lock()
		t.processFramesLocked()
		t.mu.Unlock()
	}
}

func (t *TCPStreamScanner) processFramesLocked() {
	for t.offset >= DoIPHeaderLen {
		if t.buffer[0] != DoIPProtocolVer || t.buffer[1] != DoIPInverseVer {
			copy(t.buffer, t.buffer[1:t.offset])
			t.offset--
			continue
		}

		payloadLen := binary.BigEndian.Uint32(t.buffer[4:8])

		if payloadLen > DoIPMaxFrame {
			copy(t.buffer, t.buffer[1:t.offset])
			t.offset--
			continue
		}

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
		_ = hdr

		if payloadLen > 0 {
			payload := make([]byte, payloadLen)
			copy(payload, t.buffer[DoIPHeaderLen:totalLen])

			if t.callback != nil && t.running.Load() {
				t.callback(payload, t.conn.RemoteAddr(), time.Now())
			}
		}

		remaining := t.offset - totalLen
		if remaining > 0 {
			copy(t.buffer, t.buffer[totalLen:t.offset])
		}
		t.offset = remaining

		if t.offset < 0 {
			t.offset = 0
		}
	}

	if t.offset > 0 && t.offset < DoIPHeaderLen && float64(t.offset) > 0.9*float64(len(t.buffer)) {
		t.offset = 0
	}
}

func (t *TCPStreamScanner) growBuffer() bool {
	if t.capLevel >= MaxBufferGrowth {
		t.offset = 0
		t.capLevel = 1
		if len(t.buffer) > DoIPMaxFrame*2 {
			t.buffer = make([]byte, DoIPMaxFrame*2)
		}
		return true
	}

	newSize := len(t.buffer) * 2
	if newSize > DoIPMaxFrame*MaxBufferGrowth {
		newSize = DoIPMaxFrame * MaxBufferGrowth
	}

	newBuf := make([]byte, newSize)
	if t.offset > 0 {
		copy(newBuf, t.buffer[:t.offset])
	}
	t.buffer = newBuf
	t.capLevel++
	return true
}

func (t *TCPStreamScanner) RemoteAddr() net.Addr {
	if t.conn != nil {
		return t.conn.RemoteAddr()
	}
	return nil
}

func (t *TCPStreamScanner) LocalAddr() net.Addr {
	if t.conn != nil {
		return t.conn.LocalAddr()
	}
	return nil
}
