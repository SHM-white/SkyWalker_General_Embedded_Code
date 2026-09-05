#define DT_DRV_COMPAT dji_gm6020_current

#include "dji_internal.hpp"

#define GM6020_DEFINE(inst)                                            \
    DJI_MOTOR_DEFINE(                                                  \
        inst,                                                          \
        skywalker::motor::dji::internal::kGM6020CurrentProfile)

DT_INST_FOREACH_STATUS_OKAY(GM6020_DEFINE)