#pragma once

#include "daedalus/types/joint_types.hpp"

namespace daedalus {

// Caller-supplied software limits. Wrong values are still wrong commands.
// These are independent of URDF joint limits. If an operational-space
// controller should repel inside this box, set OperationalSpaceConfig
// joint_limit_lower/upper to the same values.
struct SafetyLimits {
  JointVector q_lower;
  JointVector q_upper;
  JointVector dq_max;
  JointVector ddq_max;
  JointVector tau_max;
  JointVector tau_rate_max;
};

}  // namespace daedalus
