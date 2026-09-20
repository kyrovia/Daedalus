#pragma once

#include <Eigen/Core>

#include "daedalus/types/joint_types.hpp"

namespace daedalus {

[[nodiscard]] Eigen::MatrixXd dampedPseudoInverse(
    const Eigen::MatrixXd& matrix, double regularization);

[[nodiscard]] JointVector jointLimitTorque(const JointVector& q,
                                           const JointVector& lower,
                                           const JointVector& upper,
                                           double safe_range,
                                           double max_torque);

[[nodiscard]] JointVector frictionTorque(const JointVector& dq,
                                         const JointVector& fp1,
                                         const JointVector& fp2,
                                         const JointVector& fp3);

}  // namespace daedalus
