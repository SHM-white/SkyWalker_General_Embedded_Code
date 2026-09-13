#pragma once
#include <communication/interboard/interboard_protocol.hpp>
namespace skywalker::communication {
class InterBoardCodec {
public:
    // Encoders return COMPLETE frame bytes; decoders consume payloads and return 0/-errno.
    static int encodeHeartbeat(const robotics::BoardHeartbeat &,std::uint32_t sequence,std::uint8_t *,std::size_t);
    static int encodeChassisControl(const robotics::RemoteChassisControl &,std::uint32_t sequence,std::uint8_t *,std::size_t);
    static int encodeChassisConstraint(const robotics::ChassisConstraint &,std::uint32_t sequence,std::uint8_t *,std::size_t);
    static int encodeChassisFeedback(const robotics::ChassisFeedbackSummary &,std::uint32_t sequence,std::uint8_t *,std::size_t);
    static int decodeHeartbeat(const FrameMeta &,const std::uint8_t *,std::size_t,robotics::BoardHeartbeat &);
    static int decodeChassisControl(const FrameMeta &,const std::uint8_t *,std::size_t,robotics::RemoteChassisControl &);
    static int decodeChassisConstraint(const FrameMeta &,const std::uint8_t *,std::size_t,robotics::ChassisConstraint &);
    static int decodeChassisFeedback(const FrameMeta &,const std::uint8_t *,std::size_t,robotics::ChassisFeedbackSummary &);
};
}
