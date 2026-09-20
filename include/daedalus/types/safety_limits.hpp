#pragma once

#include "daedalus/types/joint_types.hpp"

namespace daedalus {

// Caller-supplied software limits. Wrong values are still wrong commands.
struct SafetyLimits {
  JointVector q_lower;
  JointVector q_upper;
  JointVector dq_max;
  JointVector ddq_max;
  JointVector tau_max;
  JointVector tau_rate_max;
};

}  // namespace daedalus
