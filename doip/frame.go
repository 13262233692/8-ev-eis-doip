package doip

import (
	"encoding/binary"
	"errors"
	"time"
)

const (
	PayloadTypeDiagnosticMessage           uint16 = 0x8001
	PayloadTypeDiagnosticMessagePositiveAck uint16 = 0x8002
	PayloadTypeDiagnosticMessageNegativeAck uint16 = 0x8003
	PayloadTypeAliveCheckRequest           uint16 = 0x0007
	PayloadTypeAliveCheckResponse          uint16 = 0x0008
	PayloadTypeRoutingActivationRequest    uint16 = 0x0005
	PayloadTypeRoutingActivationResponse   uint16 = 0x0006

	MaxDiagnosticPayload = 4 << 20
)

type DoIPFrame struct {
	Header      FrameHeader
	SourceAddr  uint16
	TargetAddr  uint16
	UserData    []byte
	Timestamp   time.Time
}

type RoutingActivationRequest struct {
	SourceAddress         uint16
	ActivationType        uint8
	Reserved              [3]byte
	ReservedOEM           [4]byte
}

type RoutingActivationResponse struct {
	TargetAddress         uint16
	SourceAddress         uint16
	ResponseCode          uint8
	Reserved              [3]byte
	ReservedOEM           [4]byte
}

type DiagnosticMessage struct {
	SourceAddress   uint16
	TargetAddress   uint16
	UserDataLength  uint8
	UserData        []byte
}

type AliveCheckResponse struct {
	SourceAddress uint16
}

var (
	ErrInvalidPayloadType = errors.New("doip: invalid payload type")
	ErrPayloadTooShort    = errors.New("doip: payload too short")
	ErrPayloadTooLarge    = errors.New("doip: payload exceeds maximum size")
)

func ParseDoIPFrame(payloadType uint16, rawPayload []byte, ts time.Time) (*DoIPFrame, error) {
	frame := &DoIPFrame{
		Header: FrameHeader{
			PayloadType:   payloadType,
			PayloadLength: uint32(len(rawPayload)),
		},
		Timestamp: ts,
	}

	switch payloadType {
	case PayloadTypeDiagnosticMessage:
		return parseDiagnosticMessage(frame, rawPayload)
	case PayloadTypeRoutingActivationRequest:
		return parseRoutingActivationRequest(frame, rawPayload)
	case PayloadTypeRoutingActivationResponse:
		return parseRoutingActivationResponse(frame, rawPayload)
	case PayloadTypeAliveCheckResponse:
		return parseAliveCheckResponse(frame, rawPayload)
	case PayloadTypeDiagnosticMessagePositiveAck, PayloadTypeDiagnosticMessageNegativeAck:
		return parseDiagnosticAck(frame, rawPayload)
	default:
		return nil, ErrInvalidPayloadType
	}
}

func parseDiagnosticMessage(frame *DoIPFrame, raw []byte) (*DoIPFrame, error) {
	if len(raw) < 5 {
		return nil, ErrPayloadTooShort
	}

	frame.SourceAddr = binary.BigEndian.Uint16(raw[0:2])
	frame.TargetAddr = binary.BigEndian.Uint16(raw[2:4])
	userDataLen := int(raw[4])

	if userDataLen > 0 && len(raw) >= 5+userDataLen {
		frame.UserData = make([]byte, userDataLen)
		copy(frame.UserData, raw[5:5+userDataLen])
	} else if len(raw) > 5 {
		frame.UserData = make([]byte, len(raw)-5)
		copy(frame.UserData, raw[5:])
	}

	return frame, nil
}

func parseRoutingActivationRequest(frame *DoIPFrame, raw []byte) (*DoIPFrame, error) {
	if len(raw) < 2 {
		return nil, ErrPayloadTooShort
	}
	frame.SourceAddr = binary.BigEndian.Uint16(raw[0:2])
	return frame, nil
}

func parseRoutingActivationResponse(frame *DoIPFrame, raw []byte) (*DoIPFrame, error) {
	if len(raw) < 4 {
		return nil, ErrPayloadTooShort
	}
	frame.TargetAddr = binary.BigEndian.Uint16(raw[0:2])
	frame.SourceAddr = binary.BigEndian.Uint16(raw[2:4])
	return frame, nil
}

func parseAliveCheckResponse(frame *DoIPFrame, raw []byte) (*DoIPFrame, error) {
	if len(raw) < 2 {
		return nil, ErrPayloadTooShort
	}
	frame.SourceAddr = binary.BigEndian.Uint16(raw[0:2])
	return frame, nil
}

func parseDiagnosticAck(frame *DoIPFrame, raw []byte) (*DoIPFrame, error) {
	if len(raw) < 4 {
		return nil, ErrPayloadTooShort
	}
	frame.SourceAddr = binary.BigEndian.Uint16(raw[0:2])
	frame.TargetAddr = binary.BigEndian.Uint16(raw[2:4])
	if len(raw) > 4 {
		frame.UserData = make([]byte, len(raw)-4)
		copy(frame.UserData, raw[4:])
	}
	return frame, nil
}

func (f *DoIPFrame) IsDiagnostic() bool {
	return f.Header.PayloadType == PayloadTypeDiagnosticMessage
}

func (f *DoIPFrame) DiagnosticPayload() []byte {
	if !f.IsDiagnostic() {
		return nil
	}
	return f.UserData
}

func BuildDiagnosticMessage(srcAddr, tgtAddr uint16, userData []byte) ([]byte, error) {
	if len(userData) > MaxDiagnosticPayload {
		return nil, ErrPayloadTooLarge
	}

	totalLen := DoIPHeaderLen + 5 + len(userData)
	buf := make([]byte, totalLen)

	buf[0] = DoIPProtocolVer
	buf[1] = DoIPInverseVer
	binary.BigEndian.PutUint16(buf[2:4], PayloadTypeDiagnosticMessage)
	binary.BigEndian.PutUint32(buf[4:8], uint32(5+len(userData)))

	binary.BigEndian.PutUint16(buf[8:10], srcAddr)
	binary.BigEndian.PutUint16(buf[10:12], tgtAddr)
	buf[12] = uint8(len(userData))

	if len(userData) > 0 {
		copy(buf[13:], userData)
	}

	return buf, nil
}

func BuildRoutingActivationRequest(srcAddr uint16, activationType uint8) []byte {
	buf := make([]byte, DoIPHeaderLen+11)
	buf[0] = DoIPProtocolVer
	buf[1] = DoIPInverseVer
	binary.BigEndian.PutUint16(buf[2:4], PayloadTypeRoutingActivationRequest)
	binary.BigEndian.PutUint32(buf[4:8], 11)

	binary.BigEndian.PutUint16(buf[8:10], srcAddr)
	buf[10] = activationType
	return buf
}

func BuildAliveCheckRequest() []byte {
	buf := make([]byte, DoIPHeaderLen)
	buf[0] = DoIPProtocolVer
	buf[1] = DoIPInverseVer
	binary.BigEndian.PutUint16(buf[2:4], PayloadTypeAliveCheckRequest)
	binary.BigEndian.PutUint32(buf[4:8], 0)
	return buf
}
