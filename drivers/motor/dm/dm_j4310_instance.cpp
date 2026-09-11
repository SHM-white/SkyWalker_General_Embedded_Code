#define DT_DRV_COMPAT dm_j4310_2ec_v1_1

#include "dm_internal.hpp"

#define DM_J4310_DEFINE(inst) DM_MOTOR_DEFINE(inst, skywalker::motor::dm::Model::J4310_2EC_V1_1)

DT_INST_FOREACH_STATUS_OKAY(DM_J4310_DEFINE)
