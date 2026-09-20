#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>

#include "daedalus/types/control_status.hpp"
#include "daedalus/types/joint_types.hpp"

namespace daedalus {

class DampedPseudoInverseWorkspace final {
 public:
  DampedPseudoInverseWorkspace(Eigen::Index rows, Eigen::Index cols);

  [[nodiscard]] ControlResult compute(
      const Eigen::Ref<const Eigen::MatrixXd>& matrix, double regularization,
      Eigen::Ref<Eigen::MatrixXd> output) noexcept;

 private:
#if EIGEN_MAJOR_VERSION >= 5
  Eigen::JacobiSVD<Eigen::MatrixXd,
                   Eigen::ComputeThinU | Eigen::ComputeThinV> svd_;
#else
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
#endif
  Eigen::VectorXd inverted_;
  Eigen::MatrixXd scaled_v_;
};

[[nodiscard]] ControlResult dampedPseudoInverse(
    const Eigen::Ref<const Eigen::MatrixXd>& matrix, double regularization,
    Eigen::Ref<Eigen::MatrixXd> output,
    DampedPseudoInverseWorkspace& workspace) noexcept;

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
