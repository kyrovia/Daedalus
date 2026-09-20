#include "daedalus/control/control_math.hpp"

#include <algorithm>
#include <cmath>

namespace daedalus {

DampedPseudoInverseWorkspace::DampedPseudoInverseWorkspace(
    const Eigen::Index rows, const Eigen::Index cols)
#if EIGEN_MAJOR_VERSION >= 5
    : svd_(rows, cols),
#else
    : svd_(rows, cols, Eigen::ComputeThinU | Eigen::ComputeThinV),
#endif
      inverted_(std::min(rows, cols)),
      scaled_v_(cols, std::min(rows, cols)) {}

ControlResult DampedPseudoInverseWorkspace::compute(
    const Eigen::Ref<const Eigen::MatrixXd>& matrix,
    const double regularization, Eigen::Ref<Eigen::MatrixXd> output) noexcept {
  const Eigen::Index rank = std::min(matrix.rows(), matrix.cols());
  if (output.rows() != matrix.cols() || output.cols() != matrix.rows() ||
      inverted_.size() != rank || scaled_v_.rows() != matrix.cols() ||
      scaled_v_.cols() != rank) {
    return {ControlStatus::kInvalidDimension};
  }
  if (!matrix.allFinite() || !std::isfinite(regularization)) {
    return {ControlStatus::kNonFiniteInput};
  }
  svd_.compute(matrix);
  const auto& singular_values = svd_.singularValues();
  const double lambda_squared = regularization * regularization;
  for (Eigen::Index index = 0; index < singular_values.size(); ++index) {
    const double sigma = singular_values[index];
    inverted_[index] = sigma / (sigma * sigma + lambda_squared);
  }
  scaled_v_.noalias() = svd_.matrixV();
  for (Eigen::Index index = 0; index < inverted_.size(); ++index) {
    scaled_v_.col(index) *= inverted_[index];
  }
  output.noalias() = scaled_v_ * svd_.matrixU().transpose();
  return {output.allFinite() ? ControlStatus::kOk
                             : ControlStatus::kNonFiniteOutput};
}

ControlResult dampedPseudoInverse(
    const Eigen::Ref<const Eigen::MatrixXd>& matrix,
    const double regularization, Eigen::Ref<Eigen::MatrixXd> output,
    DampedPseudoInverseWorkspace& workspace) noexcept {
  return workspace.compute(matrix, regularization, output);
}

Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& matrix,
                                    const double regularization) {
  Eigen::MatrixXd output(matrix.cols(), matrix.rows());
  DampedPseudoInverseWorkspace workspace(matrix.rows(), matrix.cols());
  const ControlResult result =
      dampedPseudoInverse(matrix, regularization, output, workspace);
  if (!result) {
    output.setZero();
  }
  return output;
}

JointVector jointLimitTorque(const JointVector& q, const JointVector& lower,
                             const JointVector& upper, const double safe_range,
                             const double max_torque) {
  const Eigen::ArrayXd lower_ratio =
      ((safe_range - (q - lower).array()) / safe_range)
          .cwiseMax(0.0)
          .cwiseMin(1.0);
  const Eigen::ArrayXd upper_ratio =
      ((safe_range - (upper - q).array()) / safe_range)
          .cwiseMax(0.0)
          .cwiseMin(1.0);
  return (max_torque * (lower_ratio - upper_ratio)).matrix();
}

JointVector frictionTorque(const JointVector& dq, const JointVector& fp1,
                           const JointVector& fp2, const JointVector& fp3) {
  const Eigen::ArrayXd ones = Eigen::ArrayXd::Ones(dq.size());
  return (fp1.array() /
              (ones + (-fp2.array() * (dq.array() + fp3.array())).exp()) -
          fp1.array() / (ones + (-fp2.array() * fp3.array()).exp()))
      .matrix();
}

}  // namespace daedalus
