#pragma once
#include <drivers/imu/imu_types.hpp>
#include <zephyr/spinlock.h>
namespace skywalker::imu {
struct Update {
    Sample sample{};
    std::uint32_t updated_mask = 0;
};
// Shared concrete publisher, not an additional virtual source layer.
// Writers are serialized by their owning source; snapshot supports many readers.
class ImuState {
public:
    explicit ImuState(const Freshness &f) : freshness_(f) {
    }
    ImuState(const ImuState &) = delete;
    ImuState &operator=(const ImuState &) = delete;
    int init(std::uint32_t capabilities, core::OrientationReference reference);
    int publish(const Update &);
    Snapshot snapshot() const;
    void error(int code, bool io = false);
    void transportGap();
    void setState(State);
    void resetOrientation(core::OrientationReference, AttitudeQuality = AttitudeQuality::Initializing);

private:
    const Freshness freshness_;
    mutable k_spinlock lock_{};
    Snapshot value_{};
};
}
