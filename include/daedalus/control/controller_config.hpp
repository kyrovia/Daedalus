#pragma once

#include <string>

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

struct CartesianImpedanceConfig {
  JointVector stiffness;
  JointVector damping;
  double nullspace_stiffness{0.0};
  double nullspace_damping{0.0};
  std::string end_effector_frame;
};

}  // namespace daedalus
