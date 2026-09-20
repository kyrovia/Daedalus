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

void addJointLimitTorque(const JointVector& q, const JointVector& lower,
                         const JointVector& upper, const double safe_range,
                         const double max_torque,
                         Eigen::Ref<JointVector> output) noexcept {
  const Eigen::Index n = output.size();
  if (q.size() != n || lower.size() != n || upper.size() != n ||
      !(safe_range > 0.0) || !std::isfinite(safe_range) ||
      !std::isfinite(max_torque)) {
    return;
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    const double lower_ratio = std::clamp(
        (safe_range - (q[i] - lower[i])) / safe_range, 0.0, 1.0);
    const double upper_ratio = std::clamp(
        (safe_range - (upper[i] - q[i])) / safe_range, 0.0, 1.0);
    output[i] += max_torque * (lower_ratio - upper_ratio);
  }
}

void addFrictionTorque(const JointVector& dq, const JointVector& fp1,
                       const JointVector& fp2, const JointVector& fp3,
                       Eigen::Ref<JointVector> output) noexcept {
  const Eigen::Index n = output.size();
  if (dq.size() != n || fp1.size() != n || fp2.size() != n ||
      fp3.size() != n) {
    return;
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    output[i] +=
        fp1[i] / (1.0 + std::exp(-fp2[i] * (dq[i] + fp3[i]))) -
        fp1[i] / (1.0 + std::exp(-fp2[i] * fp3[i]));
  }
}

JointVector jointLimitTorque(const JointVector& q, const JointVector& lower,
                             const JointVector& upper, const double safe_range,
                             const double max_torque) {
  JointVector output = JointVector::Zero(q.size());
  addJointLimitTorque(q, lower, upper, safe_range, max_torque, output);
  return output;
}

JointVector frictionTorque(const JointVector& dq, const JointVector& fp1,
                           const JointVector& fp2, const JointVector& fp3) {
  JointVector output = JointVector::Zero(dq.size());
  addFrictionTorque(dq, fp1, fp2, fp3, output);
  return output;
}

}  // namespace daedalus
