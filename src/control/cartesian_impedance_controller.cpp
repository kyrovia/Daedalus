#include "daedalus/control/cartesian_impedance_controller.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/SVD>

#include "daedalus/common/vector_require.hpp"

namespace daedalus {
namespace {
///零空间投影矩阵用的阻尼伪逆
Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& matrix,
                                    const double lambda = 0.2) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& singular_values = svd.singularValues();
  Eigen::VectorXd inverted(singular_values.size());
  const double lambda_squared = lambda * lambda;
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    const double sigma = singular_values[i];
    inverted[i] = sigma / (sigma * sigma + lambda_squared);
  }
  return svd.matrixV() * inverted.asDiagonal() * svd.matrixU().transpose();
}

Eigen::Vector3d orientationError(const Eigen::Quaterniond& desired,
                                 Eigen::Quaterniond current) {
  if (desired.coeffs().dot(current.coeffs()) < 0.0) {
    current.coeffs() = -current.coeffs();
  }
  const Eigen::AngleAxisd error_angle_axis(current * desired.inverse());
  return error_angle_axis.axis() * error_angle_axis.angle();
}

}  // namespace

CartesianImpedanceController::CartesianImpedanceController(
    std::shared_ptr<const PinocchioModel> model, CartesianImpedanceConfig config)
    : model_(std::move(model)), config_(std::move(config)) {
  if (!model_) {
    throw std::invalid_argument("model must not be null");
  }
  requireNonnegative(config_.stiffness, 6, "stiffness");
  if (config_.damping.size() == 0) {
    config_.damping = 2.0 * config_.stiffness.array().sqrt();
  } else {
    requireNonnegative(config_.damping, 6, "damping");
  }
  requireNonnegative(config_.nullspace_stiffness, "nullspace_stiffness");
  requireNonnegative(config_.nullspace_damping, "nullspace_damping");
  if (config_.end_effector_frame.empty()) {
    throw std::invalid_argument("end_effector_frame must not be empty");
  }
  if (!model_->hasFrame(config_.end_effector_frame)) {
    throw std::invalid_argument("unknown frame '" +
                                config_.end_effector_frame + "'");
  }
}

JointVector CartesianImpedanceController::compute(
    const JointState& state, const CartesianReference& reference) const {
  requireSizeAndFinite(state.q, model_->nq(), "q");
  requireSizeAndFinite(state.dq, model_->nv(), "dq");
  requireFinitePose(reference.pose);
  requireFinite(reference.wrench, "wrench");

  if (reference.q_nullspace.size() != 0) {
    requireSizeAndFinite(reference.q_nullspace, model_->nv(), "q_nullspace");
  }
  const bool use_nullspace = reference.q_nullspace.size() != 0 &&
                             (config_.nullspace_stiffness > 0.0 ||
                              config_.nullspace_damping > 0.0);

  const CartesianPose pose =
      model_->framePose(state.q, config_.end_effector_frame);
  const Eigen::MatrixXd jacobian =
      model_->frameJacobian(state.q, config_.end_effector_frame);

  CartesianVector error;
  error.head<3>() = pose.position - reference.pose.position;
  error.tail<3>() = orientationError(reference.pose.orientation.normalized(),
                                     pose.orientation.normalized());

  const CartesianVector task_wrench =
      -config_.stiffness.cwiseProduct(error) -
      config_.damping.cwiseProduct(jacobian * state.dq) + reference.wrench;
  JointVector torque = jacobian.transpose() * task_wrench;

  if (use_nullspace) {
    const int nv = model_->nv();
    const Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
    const Eigen::MatrixXd nullspace_projector =
        Eigen::MatrixXd::Identity(nv, nv) -
        jacobian_transpose * dampedPseudoInverse(jacobian_transpose);
    torque += nullspace_projector *
              (config_.nullspace_stiffness *
                   (reference.q_nullspace - state.q) -
               config_.nullspace_damping * state.dq);
  }

  torque += model_->gravity(state.q);
  return torque;
}

}  // namespace daedalus