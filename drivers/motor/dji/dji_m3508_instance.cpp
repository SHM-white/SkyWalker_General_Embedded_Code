#define DT_DRV_COMPAT dji_m3508_c620

#include "dji_internal.hpp"

#define M3508_DEFINE(inst)                                             \
    DJI_MOTOR_DEFINE(                                                  \
        inst,                                                          \
        skywalker::motor::dji::internal::kM3508C620Profile)

DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)