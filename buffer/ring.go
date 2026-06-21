package buffer

import (
	"errors"
	"sync/atomic"
)

var (
	ErrBufferFull  = errors.New("buffer: ring buffer is full")
	ErrBufferEmpty = errors.New("buffer: ring buffer is empty")
)

type LockFreeRing struct {
	buffer    []byte
	capacity  uint64
	writePos  uint64
	readPos   uint64
	wrapped   uint32
}

func NewLockFreeRing(capacity int) *LockFreeRing {
	if capacity <= 0 {
		capacity = 1 << 20
	}
	return &LockFreeRing{
		buffer:   make([]byte, capacity),
		capacity: uint64(capacity),
	}
}

func (r *LockFreeRing) Capacity() int {
	return int(r.capacity)
}

func (r *LockFreeRing) Write(data []byte) (int, error) {
	writePos := atomic.LoadUint64(&r.writePos)
	readPos := atomic.LoadUint64(&r.readPos)

	used := (writePos - readPos) % r.capacity
	if writePos < readPos {
		used = writePos + (r.capacity - readPos)
	}
	available := r.capacity - used - 1

	if available == 0 {
		return 0, ErrBufferFull
	}

	n := uint64(len(data))
	if n > available {
		n = available
	}

	firstChunk := r.capacity - (writePos % r.capacity)
	if n <= firstChunk {
		copy(r.buffer[writePos%r.capacity:], data[:n])
	} else {
		copy(r.buffer[writePos%r.capacity:], data[:firstChunk])
		copy(r.buffer[0:], data[firstChunk:n])
	}

	atomic.StoreUint64(&r.writePos, (writePos+n)%r.capacity)
	return int(n), nil
}

func (r *LockFreeRing) Read(buf []byte) (int, error) {
	writePos := atomic.LoadUint64(&r.writePos)
	readPos := atomic.LoadUint64(&r.readPos)

	if writePos == readPos {
		return 0, ErrBufferEmpty
	}

	var available uint64
	if writePos > readPos {
		available = writePos - readPos
	} else {
		available = (r.capacity - readPos) + writePos
	}

	n := uint64(len(buf))
	if n > available {
		n = available
	}

	firstChunk := r.capacity - readPos
	if n <= firstChunk {
		copy(buf, r.buffer[readPos:readPos+n])
	} else {
		copy(buf, r.buffer[readPos:])
		copy(buf[firstChunk:], r.buffer[:n-firstChunk])
	}

	atomic.StoreUint64(&r.readPos, (readPos+n)%r.capacity)
	return int(n), nil
}

func (r *LockFreeRing) Peek(buf []byte) (int, error) {
	writePos := atomic.LoadUint64(&r.writePos)
	readPos := atomic.LoadUint64(&r.readPos)

	if writePos == readPos {
		return 0, ErrBufferEmpty
	}

	var available uint64
	if writePos > readPos {
		available = writePos - readPos
	} else {
		available = (r.capacity - readPos) + writePos
	}

	n := uint64(len(buf))
	if n > available {
		n = available
	}

	firstChunk := r.capacity - readPos
	if n <= firstChunk {
		copy(buf, r.buffer[readPos:readPos+n])
	} else {
		copy(buf, r.buffer[readPos:])
		copy(buf[firstChunk:], r.buffer[:n-firstChunk])
	}

	return int(n), nil
}

func (r *LockFreeRing) Available() int {
	writePos := atomic.LoadUint64(&r.writePos)
	readPos := atomic.LoadUint64(&r.readPos)

	if writePos == readPos {
		return 0
	}
	if writePos > readPos {
		return int(writePos - readPos)
	}
	return int((r.capacity - readPos) + writePos)
}

func (r *LockFreeRing) Free() int {
	return int(r.capacity) - r.Available() - 1
}

func (r *LockFreeRing) Reset() {
	atomic.StoreUint64(&r.writePos, 0)
	atomic.StoreUint64(&r.readPos, 0)
}

type SampleRing struct {
	buffer     [][]float64
	capacity   int
	writeIdx   int32
	readIdx    int32
	size       int32
}

func NewSampleRing(capacity int) *SampleRing {
	if capacity <= 0 {
		capacity = 8192
	}
	return &SampleRing{
		buffer:   make([][]float64, capacity),
		capacity: capacity,
	}
}

func (s *SampleRing) Push(sample []float64) {
	idx := atomic.LoadInt32(&s.writeIdx)
	s.buffer[idx] = sample

	newIdx := (idx + 1) % int32(s.capacity)
	atomic.StoreInt32(&s.writeIdx, newIdx)

	curSize := atomic.LoadInt32(&s.size)
	if curSize < int32(s.capacity) {
		atomic.AddInt32(&s.size, 1)
	} else {
		atomic.StoreInt32(&s.readIdx, newIdx)
	}
}

func (s *SampleRing) Pop() ([]float64, bool) {
	size := atomic.LoadInt32(&s.size)
	if size == 0 {
		return nil, false
	}

	idx := atomic.LoadInt32(&s.readIdx)
	sample := s.buffer[idx]
	s.buffer[idx] = nil

	newIdx := (idx + 1) % int32(s.capacity)
	atomic.StoreInt32(&s.readIdx, newIdx)
	atomic.AddInt32(&s.size, -1)

	return sample, true
}

func (s *SampleRing) Size() int {
	return int(atomic.LoadInt32(&s.size))
}

func (s *SampleRing) Capacity() int {
	return s.capacity
}

func (s *SampleRing) DrainAll() [][]float64 {
	size := atomic.LoadInt32(&s.size)
	if size == 0 {
		return nil
	}

	result := make([][]float64, 0, size)
	readIdx := atomic.LoadInt32(&s.readIdx)
	for i := int32(0); i < size; i++ {
		idx := (readIdx + i) % int32(s.capacity)
		if s.buffer[idx] != nil {
			sample := make([]float64, len(s.buffer[idx]))
			copy(sample, s.buffer[idx])
			result = append(result, sample)
		}
	}

	atomic.StoreInt32(&s.readIdx, (readIdx+size)%int32(s.capacity))
	atomic.StoreInt32(&s.size, 0)

	return result
}

type Float64Ring struct {
	buffer   []float64
	capacity int32
	head     int32
	tail     int32
	count    int32
}

func NewFloat64Ring(capacity int) *Float64Ring {
	if capacity <= 0 {
		capacity = 65536
	}
	return &Float64Ring{
		buffer:   make([]float64, capacity),
		capacity: int32(capacity),
	}
}

func (f *Float64Ring) Push(v float64) {
	f.buffer[f.head] = v
	f.head = (f.head + 1) % f.capacity
	if f.count < f.capacity {
		f.count++
	} else {
		f.tail = (f.tail + 1) % f.capacity
	}
}

func (f *Float64Ring) Pop() (float64, bool) {
	if f.count == 0 {
		return 0, false
	}
	v := f.buffer[f.tail]
	f.tail = (f.tail + 1) % f.capacity
	f.count--
	return v, true
}

func (f *Float64Ring) Count() int {
	return int(f.count)
}

func (f *Float64Ring) Snapshot() []float64 {
	result := make([]float64, f.count)
	for i := int32(0); i < f.count; i++ {
		idx := (f.tail + i) % f.capacity
		result[i] = f.buffer[idx]
	}
	return result
}

func (f *Float64Ring) Clear() {
	f.head = 0
	f.tail = 0
	f.count = 0
}
