#pragma once

#include <cstddef>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <drivers/motor/motor.hpp>

#ifndef CONFIG_SKYWALKER_MOTOR_MAX_MOTORS_PER_BUS
#define CONFIG_SKYWALKER_MOTOR_MAX_MOTORS_PER_BUS 16
#endif
#ifndef CONFIG_SKYWALKER_MOTOR_RX_QUEUE_DEPTH
#define CONFIG_SKYWALKER_MOTOR_RX_QUEUE_DEPTH 32
#endif
#ifndef CONFIG_SKYWALKER_MOTOR_IO_STACK_SIZE
#define CONFIG_SKYWALKER_MOTOR_IO_STACK_SIZE 3072
#endif

namespace skywalker::motor {

struct BusOptions {
    // 单帧 CAN 发送完成的等待上限，单位 ms。
    std::uint32_t tx_timeout_ms = 2;
    // CAN 控制器恢复失败后的重试间隔，单位 ms。
    std::uint32_t recovery_retry_ms = 100;
};

enum class BusState : std::uint8_t { Unstarted, Running, Recovering, ConfigBlocked };
enum class TxPurpose : std::uint8_t { Target, SafeOutput, Enable, Probe, ClearFault };

struct CommitResult {
    int error = 0;
    std::uint64_t sequence = 0;
};

struct TxResult {
    bool valid = false;
    std::uint64_t sequence = 0;
    std::uint32_t can_id = 0;
    TxPurpose purpose = TxPurpose::Target;
    int error = 0;
    std::uint64_t completed_ms = 0;
};

struct BusStatus {
    BusState state = BusState::Unstarted;
    int last_error = 0;
    std::uint64_t latest_submitted_sequence = 0;
    TxResult last_tx{};
    std::uint64_t superseded_batches = 0;
    std::uint64_t rx_overflows = 0;
    std::uint64_t rx_invalid_frames = 0;
};

class CanBus {
public:
    explicit CanBus(const device *can, BusOptions options = {});
    CanBus(const CanBus &) = delete;
    CanBus &operator=(const CanBus &) = delete;
    CanBus(CanBus &&) = delete;
    CanBus &operator=(CanBus &&) = delete;

    [[nodiscard]] int attach(Motor &motor);

    template <class... Motors> [[nodiscard]] int attach(Motor &first, Motor &second, Motors &...rest) {
        Motor *batch[] = {&first, &second, &rest...};
        return attachBatch(batch, sizeof...(rest) + 2);
    }

    [[nodiscard]] int start();
    // Publish this entire bus. Application writers must coordinate their
    // setter/update + commit batches; multiple control threads may own
    // different buses. Success acknowledges publication, not CAN completion.
    [[nodiscard]] CommitResult commit();
    BusStatus status() const;

private:
    friend class Group;
    friend class Motor;

    static constexpr std::size_t kMaxMotors = CONFIG_SKYWALKER_MOTOR_MAX_MOTORS_PER_BUS;
    static constexpr std::size_t kRxDepth = CONFIG_SKYWALKER_MOTOR_RX_QUEUE_DEPTH;

    enum class UnitKind : std::uint8_t { Dji, Dm };
    struct TxUnit {
        UnitKind kind = UnitKind::Dji;
        std::uint16_t command_id = 0;
        std::size_t motor_index = 0;
    };
    struct RxEvent {
        can_frame frame{};
        std::uint64_t received_ms = 0;
        std::uint64_t bus_generation = 0;
        std::uint64_t callback_order = 0;
    };
    struct InFlight {
        can_frame frame{};
        TxPurpose purpose = TxPurpose::Target;
        std::uint64_t sequence = 0;
        std::uint64_t bus_generation = 0;
        std::uint64_t operation_id = 0;
        std::uint64_t submitted_ms = 0;
        std::uint64_t enable_generation = 0;
        std::uint64_t clear_generation = 0;
        std::uint64_t stop_generations[kMaxMotors]{};
        std::size_t unit_index = 0;
        bool busy = false;
        bool callback_seen = false;
        int callback_error = 0;
        std::uint64_t completed_ms = 0;
        std::uint64_t completed_order = 0;
    };
    struct Route {
        std::uint16_t id = 0;
        int filter_id = -1;
    };

    // I/O-owned storage: frame bytes and their authority are captured together.
    struct CandidateMotor {
        StagedCommand command{};
        std::uint64_t enable_generation = 0;
        std::uint64_t stop_generation = 0;
        std::uint64_t feedback_ms = 0;
        MotorState state = MotorState::Offline;
        bool included = false;
        bool output_permitted = false;
        bool motion = false;
        bool safe_action = false;
    };
    struct TxCandidate {
        can_frame frame{};
        CandidateMotor motors[kMaxMotors]{};
        std::uint64_t sequence = 0;
        std::uint64_t bus_generation = 0;
        std::size_t unit_index = 0;
        TxPurpose purpose = TxPurpose::Target;
    };

    int attachBatch(Motor *const *batch, std::size_t count);
    static bool isDji(const Motor &motor);
    static int describeMotor(const Motor &motor, std::uint16_t &rx_id, std::uint16_t &tx_id, std::uint8_t &slot);
    int validateTopology();
    int installRoutes();
    void rollbackStart();
    void wake();
    std::uint64_t recordCallbackOrder();
    static void onRx(const device *, can_frame *frame, void *context);
    static void onTxDone(const device *, int error, void *context);
    static void threadEntry(void *context, void *, void *);
    void ioMain();
    void processRx(const RxEvent &event);
    void processTx();
    void checkDeadlines(std::uint64_t now_ms);
    void pumpTx(std::uint64_t now_ms);
    bool pumpTarget(std::uint64_t now_ms);
    void enterRecovery(int error, FaultReason reason);
    void recoverController(std::uint64_t now_ms);
    void captureCandidate(std::size_t unit_index, TxPurpose purpose);
    int submitCandidate();
    int buildTarget(std::uint64_t now_ms);
    int buildSafety();
    std::uint32_t nextWaitMs(std::uint64_t now_ms) const;
    bool unitHasPendingSafety(const TxUnit &unit) const;
    bool unitNeedsDmSafetyProbe(const TxUnit &unit, std::uint64_t now_ms) const;
    void updateStopAfterTx(const InFlight &completed);

    const device *can_ = nullptr;
    BusOptions options_{};
    Motor *motors_[kMaxMotors]{};
    std::size_t motor_count_ = 0;
    TxUnit units_[kMaxMotors]{};
    std::size_t unit_count_ = 0;
    Route routes_[kMaxMotors]{};
    std::size_t route_count_ = 0;
    StagedCommand published_[kMaxMotors]{};
    std::uint64_t neutral_done_generation_[kMaxMotors]{};
    std::uint64_t published_sequence_ = 0;
    std::size_t target_cursor_ = 0;
    std::size_t safety_probe_cursor_ = 0;
    bool target_before_safety_probe_ = true;
    std::size_t targets_remaining_ = 0;
    std::uint64_t bus_generation_ = 1;
    std::uint64_t next_operation_id_ = 1;
    std::uint64_t next_recovery_ms_ = 0;
    bool controller_started_ = false;
    bool thread_started_ = false;

    mutable k_spinlock state_lock_{};
    BusStatus status_{};
    mutable k_spinlock callback_order_lock_{};
    std::uint64_t next_callback_order_ = 1;
    mutable k_spinlock publication_lock_{};
    mutable k_spinlock rx_lock_{};
    RxEvent rx_queue_[kRxDepth]{};
    std::size_t rx_head_ = 0;
    std::size_t rx_tail_ = 0;
    std::size_t rx_count_ = 0;
    bool rx_overflowed_ = false;
    mutable k_spinlock tx_lock_{};
    InFlight in_flight_{};
    TxCandidate candidate_{};
    k_sem wake_sem_{};
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(thread_stack_, CONFIG_SKYWALKER_MOTOR_IO_STACK_SIZE);
};

} // namespace skywalker::motor
