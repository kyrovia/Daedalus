#include "daedalus/control/control_math.hpp"

#include <Eigen/SVD>

namespace daedalus {

Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& matrix,
                                    const double regularization) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& singular_values = svd.singularValues();
  Eigen::VectorXd inverted(singular_values.size());
  const double lambda_squared = regularization * regularization;
  for (Eigen::Index index = 0; index < singular_values.size(); ++index) {
    const double sigma = singular_values[index];
    inverted[index] = sigma / (sigma * sigma + lambda_squared);
  }
  return svd.matrixV() * inverted.asDiagonal() * svd.matrixU().transpose();
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
