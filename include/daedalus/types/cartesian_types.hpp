#pragma once

#include <Eigen/Geometry>

#include "daedalus/types/joint_types.hpp"

namespace daedalus {

using CartesianVector = Eigen::Matrix<double, 6, 1>;

enum class JacobianReference {
  kLocalWorldAligned,
  kLocal,
};

struct CartesianPose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

// Impedance regulation setpoint: pose and optional nullspace posture.
// There is no desired twist; damping acts on the measured task velocity.
struct CartesianReference {
  CartesianPose pose;
  JointVector q_nullspace;
  CartesianVector wrench = CartesianVector::Zero();
};

}  // namespace daedalus
