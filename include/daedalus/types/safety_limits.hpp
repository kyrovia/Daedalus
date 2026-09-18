#pragma once

#include "daedalus/types/joint_types.hpp"

namespace daedalus {

struct SafetyLimits {
  JointVector q_lower;
  JointVector q_upper;
  JointVector dq_max;
  JointVector ddq_max;
  JointVector tau_max;
  JointVector tau_rate_max;
};

}  // namespace daedalus
