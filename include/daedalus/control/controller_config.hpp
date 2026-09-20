#pragma once

#include <string>

#include "daedalus/types/cartesian_types.hpp"
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
  bool limit_error{true};
  JointVector error_clip =
      (JointVector(6) << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3).finished();
};

enum class NullspaceProjector {
  kDynamic,
  kKinematic,
  kNone,
};

struct OperationalSpaceConfig {
  JointVector stiffness;
  JointVector damping;
  double nullspace_stiffness{0.0};
  double nullspace_damping{-1.0};
  JointVector nullspace_weights;
  std::string end_effector_frame;
  JacobianReference jacobian_reference{JacobianReference::kLocalWorldAligned};
  double operational_space_regularization{0.01};
  double nullspace_regularization{1e-4};
  NullspaceProjector nullspace_projector{NullspaceProjector::kDynamic};
  bool limit_error{true};
  JointVector error_clip =
      (JointVector(6) << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3).finished();
  bool use_friction{false};
  JointVector friction_fp1;
  JointVector friction_fp2;
  JointVector friction_fp3;
  bool use_coriolis{true};
  bool use_gravity{true};
  bool use_joint_limit_repulsion{true};
  // Empty means URDF position limits. Match SafetyLimits when this
  // controller runs inside ControlPipeline so repulsion can act before
  // the pipeline rejects the measured state.
  JointVector joint_limit_lower;
  JointVector joint_limit_upper;
  double joint_limit_safe_range{0.3};
  double joint_limit_max_torque{5.0};
  double nullspace_max_torque{10.0};
  double target_filter_alpha{0.0};
};

}  // namespace daedalus
