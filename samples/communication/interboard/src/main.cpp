#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <communication/async_uart.hpp>
#include <communication/interboard/interboard_link.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(interboard_bench,LOG_LEVEL_INF);
int main() {
    using namespace skywalker::communication; using namespace skywalker::robotics;
    static AsyncUart uart(bench::uart); InterBoardLink link(bench::role);
    int ret=uart.init(); const auto boot=sys_rand64_get() | 1ULL;
    LOG_INF("UART init=%d role=%u boot=%llx; console p pause/resume COMMAND producer, heartbeats continue; g changes chassis generation",ret,unsigned(bench::role),boot);
    const device *console=DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    std::uint64_t next_tx=0,next_log=0; std::uint32_t frame_seq=0,command_seq=0,generation=1;
    RemoteChassisControl command{}; bool produce=true;
    for (;;) {
        const auto now=static_cast<std::uint64_t>(k_uptime_get()); uart.service(now); AsyncUart::RxChunk chunk{};
        for (unsigned budget=0;budget<8;++budget) {
            ret=uart.read(chunk); if (ret==-EOVERFLOW) { link.discardPartial(); continue; } if (ret<0) break;
            link.processRxBytes(chunk.bytes,chunk.size,chunk.timestamp_ms);
        }
        link.processRxBytes(nullptr,0,now);
        unsigned char key; if (uart_poll_in(console,&key)==0) { if (key=='p') produce=!produce; if (key=='g') ++generation; }
        BoardHeartbeat peer{}; link.latestHeartbeat(peer);
        if (now>=next_tx && !uart.txBusy()) {
            next_tx=now+20; BoardHeartbeat h{}; h.role=bench::role; h.sender_boot_id=boot; h.resume_generation=generation;
            h.sender_uptime_ms=now; h.ready=true; h.safety_state=SafetyState::Ready; h.sync_requested=!link.peerOnline(now);
            std::uint8_t bytes[128]{}; int n=InterBoardCodec::encodeHeartbeat(h,++frame_seq,bytes,sizeof(bytes));
            if (n>0 && bench::role==BoardRole::GimbalController) {
                if (produce) {
                    command.command.mode=ChassisMode::BodyVelocity; command.command.vx_m_s=bench::target_vx_m_s;
                    command.command.source=ControlSource::Remote; command.command.stamp={now,++command_seq,true};
                    command.global_action=SafetyAction::Active; command.receiver_boot_id=peer.sender_boot_id; command.resume_generation=peer.resume_generation;
                }
                const int m=InterBoardCodec::encodeChassisControl(command,++frame_seq,bytes+n,sizeof(bytes)-n); if (m>0) uart.send(bytes,n+m);
            } else if (n>0) {
                RemoteChassisControl c{}; link.latestChassisControl(c); ChassisFeedbackSummary f{};
                const bool fresh=isFresh(c.stamp,now,100),context=c.receiver_boot_id==boot && c.resume_generation==generation;
                f.execution_state=ExecutionState::Ready; f.safety_state=fresh && context ? SafetyState::Ready : SafetyState::Waiting;
                f.ready=true; f.last_command_sequence=c.command.stamp.sequence;
                if (!fresh) f.active_reasons|=CommandStale;
                if (!context) f.active_reasons|=RecoveryBoundary;
                const int m=InterBoardCodec::encodeChassisFeedback(f,++frame_seq,bytes+n,sizeof(bytes)-n); if (m>0) uart.send(bytes,n+m);
            }
        }
        if (now>=next_log) {
            next_log=now+200; RemoteChassisControl c{}; link.latestChassisControl(c); ChassisFeedbackSummary f{}; link.latestChassisFeedback(f);
            LOG_INF("peer_online=%d peer_boot=%llx peer_gen=%u local_gen=%u cmd_seq=%u cmd_fresh=%d cmd_age_ms=%llu peer_reason=%x CRC=%u dropped=%ld",
                link.peerOnline(now),peer.sender_boot_id,peer.resume_generation,generation,c.command.stamp.sequence,isFresh(c.stamp,now,100),
                c.stamp.valid ? now-c.stamp.timestamp_ms : 0,f.active_reasons,link.stats().crc_errors,long(uart.droppedChunks()));
        }
        k_sleep(K_MSEC(1));
    }
}
