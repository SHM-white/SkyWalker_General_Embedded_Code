#pragma once
#include <drivers/imu/imu_types.hpp>
namespace skywalker::imu {
// One owner calls init/service; any thread may copy snapshot. No I/O in constructor.
// Sources with registered callbacks and their dependencies must remain alive.
class ImuSource {
public:
    virtual ~ImuSource() = default;
    ImuSource(const ImuSource &) = delete;
    ImuSource &operator=(const ImuSource &) = delete;
    virtual int init() = 0;
    virtual int service() = 0; // 0 progress, -EAGAIN no update, other negative errno.
    virtual Snapshot snapshot() const = 0;

protected:
    ImuSource() = default;
};
}
