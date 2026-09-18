#pragma once

#include "daedalus/types/joint_types.hpp"

namespace daedalus {

struct ComputedTorqueConfig {
  JointVector kp;
  JointVector kd;
};

struct JointImpedanceConfig {
  JointVector stiffness;
  JointVector damping;
};

}  // namespace daedalus
