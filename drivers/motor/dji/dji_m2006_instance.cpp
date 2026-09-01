#define DT_DRV_COMPAT dji_m2006_c610

#include "dji_internal.hpp"

#define M2006_DEFINE(inst)                                             \
    DJI_MOTOR_DEFINE(                                                  \
        inst,                                                          \
        skywalker::motor::dji::internal::kM2006C610Profile)

DT_INST_FOREACH_STATUS_OKAY(M2006_DEFINE)