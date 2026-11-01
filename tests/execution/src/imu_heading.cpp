#include <zephyr/ztest.h>
#include <communication/interboard/interboard_endpoint.hpp>
#include <communication/wire.hpp>
#include <array>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <cmath>
using namespace skywalker;
using namespace communication;
using namespace robotics;
namespace {
GimbalImuYaw sample() {
    return {.valid=true,.yaw_rad=1,.yaw_rate_rad_s=-.5f,.source_age_ms=7,.sample_sequence=42,
            .zero_generation=2,.receiver_boot_id=22,.sender_boot_id=11,.stamp={100,1,true}};
}
class Cable final : public InterBoardTransport {
public:
    Cable *peer=nullptr;
    std::array<RxChunk,32> queue{};
    unsigned used=0;
    std::uint64_t now=0;
    std::array<std::uint8_t,240> last{};
    std::size_t last_size=0;
    InterBoardTransportKind kind() const override { return InterBoardTransportKind::Uart; }
    int service(std::uint64_t n) override { now=n;return 0; }
    int read(RxChunk &o) override {
        if(!used)return -EAGAIN;
        o=queue[0];for(unsigned i=1;i<used;++i)queue[i-1]=queue[i];--used;return 0;
    }
    int send(const std::uint8_t *p,std::size_t n,std::uint32_t) override {
        std::memcpy(last.data(),p,n); last_size=n;
        if(!peer)return 0;
        for(std::size_t i=0;i<n;i+=64) {
            if(peer->used==peer->queue.size())return -ENOSPC;
            auto &q=peer->queue[peer->used++];q.size=std::min<std::size_t>(64,n-i);
            std::memcpy(q.bytes,p+i,q.size);q.timestamp_ms=now;
        }
        return 0;
    }
    bool txBusy() const override { return false; }
};
}
ZTEST(imu_wire, test_codec_golden_bytes_length_reserved_and_finite) {
    std::uint8_t b[kMaxFrame]{}; auto s=sample();
    zassert_equal(InterBoardCodec::encodeGimbalImuYaw(s,9,b,sizeof(b)),54);
    zassert_equal(b[4],5);zassert_equal(b[5],4);zassert_equal(b[6],40);
    constexpr std::uint8_t prefix[]={1,0,0,0,0,0,0x80,0x3f,0,0,0,0xbf,7,0,0,0,42,0,0,0,2,0,0,0};
    zassert_mem_equal(b+12,prefix,sizeof(prefix));
    zassert_equal(wire::loadLe64(b+36),22);zassert_equal(wire::loadLe64(b+44),11);
    FrameMeta meta{BoardRole::GimbalController,MessageId::GimbalImuYaw,9,100};GimbalImuYaw out{};
    zassert_ok(InterBoardCodec::decodeGimbalImuYaw(meta,b+12,40,out));zassert_equal(out.sample_sequence,42);
    for(std::size_t n : {0u,31u,32u,39u,41u}) zassert_equal(InterBoardCodec::decodeGimbalImuYaw(meta,b+12,n,out),-EINVAL);
    b[13]=1;zassert_equal(InterBoardCodec::decodeGimbalImuYaw(meta,b+12,40,out),-EINVAL);b[13]=0;
    wire::storeFloatLe(b+16,NAN);zassert_equal(InterBoardCodec::decodeGimbalImuYaw(meta,b+12,40,out),-EINVAL);
    s.source_age_ms=51;zassert_equal(InterBoardCodec::encodeGimbalImuYaw(s,1,b,sizeof(b)),-EINVAL);
    s=sample();s.sender_boot_id=0;zassert_equal(InterBoardCodec::encodeGimbalImuYaw(s,1,b,sizeof(b)),-EINVAL);
}
ZTEST(imu_wire, test_link_replay_generations_session_and_corruption) {
    InterBoardLink link(BoardRole::ChassisController);std::uint8_t b[kMaxFrame]{};
    auto s=sample();BoardHeartbeat hb{};hb.role=BoardRole::GimbalController;hb.sender_boot_id=11;
    int n=InterBoardCodec::encodeGimbalImuYaw(s,1,b,sizeof(b));zassert_ok(link.processRxBytes(b,n,100));
    GimbalImuYaw got{};zassert_equal(link.latestGimbalImuYaw(got),-EAGAIN);
    n=InterBoardCodec::encodeHeartbeat(hb,1,b,sizeof(b));link.processRxBytes(b,n,100);
    n=InterBoardCodec::encodeGimbalImuYaw(s,2,b,sizeof(b));link.processRxBytes(b,n,100);zassert_ok(link.latestGimbalImuYaw(got));
    zassert_equal(got.stamp.timestamp_ms,100);
    n=InterBoardCodec::encodeGimbalImuYaw(s,3,b,sizeof(b));link.processRxBytes(b,n,130);zassert_ok(link.latestGimbalImuYaw(got));
    zassert_equal(got.stamp.timestamp_ms,100); // Retry does not renew sample age.
    s.zero_generation=1;s.sample_sequence=100;n=InterBoardCodec::encodeGimbalImuYaw(s,4,b,sizeof(b));link.processRxBytes(b,n,130);
    link.latestGimbalImuYaw(got);zassert_equal(got.zero_generation,2);
    s=sample();s.sender_boot_id=77;n=InterBoardCodec::encodeGimbalImuYaw(s,5,b,sizeof(b));link.processRxBytes(b,n,130);
    link.latestGimbalImuYaw(got);zassert_equal(got.sender_boot_id,11);
    s=sample();s.sample_sequence=43;n=InterBoardCodec::encodeGimbalImuYaw(s,6,b,sizeof(b));b[n-1]^=1;
    link.processRxBytes(b,n,140);zassert_true(link.stats().crc_errors>0);
    hb.sender_boot_id=77;n=InterBoardCodec::encodeHeartbeat(hb,1,b,sizeof(b));link.processRxBytes(b,n,140);
    zassert_equal(link.latestGimbalImuYaw(got),-EAGAIN);
    hb.sender_boot_id=11;n=InterBoardCodec::encodeHeartbeat(hb,999,b,sizeof(b));link.processRxBytes(b,n,140);
    BoardHeartbeat current{};zassert_ok(link.latestHeartbeat(current));zassert_equal(current.sender_boot_id,77);
    s=sample();n=InterBoardCodec::encodeGimbalImuYaw(s,100,b,sizeof(b));link.processRxBytes(b,n,140);
    zassert_equal(link.latestGimbalImuYaw(got),-EAGAIN);
}
ZTEST(imu_wire, test_two_endpoint_freshness_receiver_reboot_and_rebinding) {
    Cable ga,ca;ga.peer=&ca;ca.peer=&ga;
    InterBoardEndpoint g(ga,{BoardRole::GimbalController}),c(ca,{BoardRole::ChassisController});
    for(unsigned now=100;now<=160;now+=10) {g.poll(now);c.poll(now);}
    zassert_true(g.snapshot().online);zassert_true(c.snapshot().online);
    auto s=sample();s.receiver_boot_id=c.snapshot().local_boot_id;s.stamp={160,1,true};
    s.source_age_ms=0;g.submitGimbalImuYaw(s);
    g.poll(180);c.poll(180);zassert_true(c.snapshot().gimbal_imu.valid);
    zassert_equal(c.snapshot().gimbal_imu.sender_boot_id,g.snapshot().local_boot_id);
    g.poll(200);c.poll(200);zassert_true(c.snapshot().gimbal_imu.valid);
    g.poll(220);c.poll(220);zassert_false(c.snapshot().gimbal_imu.valid);
    s.stamp={220,2,true};++s.sample_sequence;g.submitGimbalImuYaw(s);
    g.poll(240);c.poll(240);zassert_true(c.snapshot().gimbal_imu.valid);
    // New receiver identity: producer must explicitly bind a fresh sample.
    Cable cb;cb.peer=&ga;ga.peer=&cb;
    InterBoardEndpoint reboot(cb,{BoardRole::ChassisController});
    for(unsigned now=250;now<=300;now+=10) {reboot.poll(now);g.poll(now);}
    reboot.poll(310);zassert_false(reboot.snapshot().gimbal_imu.valid);
    s.receiver_boot_id=reboot.snapshot().local_boot_id;s.stamp={310,3,true};++s.sample_sequence;
    g.submitGimbalImuYaw(s);g.poll(330);reboot.poll(330);zassert_true(reboot.snapshot().gimbal_imu.valid);
    // New sender boot rejects an old IMU frame even when target boot still matches.
    std::uint8_t b[kMaxFrame]{};BoardHeartbeat hb{};hb.role=BoardRole::GimbalController;hb.sender_boot_id=123;
    int n=InterBoardCodec::encodeHeartbeat(hb,1,b,sizeof(b));ga.now=340;ga.send(b,n,10);reboot.poll(340);
    s.sender_boot_id=g.snapshot().local_boot_id;n=InterBoardCodec::encodeGimbalImuYaw(s,999,b,sizeof(b));
    ga.send(b,n,10);reboot.poll(341);zassert_false(reboot.snapshot().gimbal_imu.valid);
}
ZTEST_SUITE(imu_wire,nullptr,nullptr,nullptr,nullptr,nullptr);
