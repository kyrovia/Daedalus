#include "daedalus/control/operational_space_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/SVD>
#include <pinocchio/spatial/explog.hpp>

#include "daedalus/common/vector_require.hpp"

namespace daedalus {
namespace {

Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& matrix,
                                    const double regularization) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd inverse = Eigen::MatrixXd::Zero(
      svd.matrixV().cols(), svd.matrixU().rows());
  const double squared = regularization * regularization;
  for (Eigen::Index index = 0; index < svd.singularValues().size(); ++index) {
    const double sigma = svd.singularValues()[index];
    inverse(index, index) = sigma / (sigma * sigma + squared);
  }
  return svd.matrixV() * inverse * svd.matrixU().transpose();
}

JointVector jointLimitTorque(const JointVector& q,
                             const JointVector& lower,
                             const JointVector& upper,
                             const double safe_range,
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
///工程近似的库伦摩擦和粘性摩擦
JointVector frictionTorque(const JointVector& dq,
                           const JointVector& fp1,
                           const JointVector& fp2,
                           const JointVector& fp3) {
  const Eigen::ArrayXd ones = Eigen::ArrayXd::Ones(dq.size());
  return (fp1.array() /
              (ones + (-fp2.array() * (dq.array() + fp3.array())).exp()) -
          fp1.array() / (ones + (-fp2.array() * fp3.array()).exp()))
      .matrix();
}

}  // namespace

OperationalSpaceController::OperationalSpaceController(
    std::shared_ptr<const PinocchioModel> model, OperationalSpaceConfig config)
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
  requireNonnegative(
      config_.nullspace_stiffness, "nullspace_stiffness");
  if (config_.nullspace_damping < 0.0) {
    config_.nullspace_damping =
        2.0 * std::sqrt(config_.nullspace_stiffness);
  } else {
    requireNonnegative(
        config_.nullspace_damping, "nullspace_damping");
  }
  if (config_.nullspace_weights.size() == 0) {
    config_.nullspace_weights = JointVector::Ones(model_->nv());
  } else {
    requireNonnegative(
        config_.nullspace_weights, model_->nv(), "nullspace_weights");
  }
  if (config_.end_effector_frame.empty() ||
      !model_->hasFrame(config_.end_effector_frame)) {
    throw std::invalid_argument(
        "end_effector_frame must name an existing frame");
  }
  requirePositive(config_.operational_space_regularization,
                  "operational_space_regularization");
  requirePositive(
      config_.nullspace_regularization, "nullspace_regularization");
  requirePositive(config_.error_clip, 6, "error_clip");
  requirePositive(
      config_.joint_limit_safe_range, "joint_limit_safe_range");
  requireNonnegative(
      config_.joint_limit_max_torque, "joint_limit_max_torque");
  requireNonnegative(
      config_.nullspace_max_torque, "nullspace_max_torque");
  requireFinite(config_.target_filter_alpha, "target_filter_alpha");
  if (config_.target_filter_alpha < 0.0 ||
      config_.target_filter_alpha > 1.0) {
    throw std::invalid_argument(
        "target_filter_alpha must be in [0, 1]");
  }
  if (config_.use_friction) {
    requireSizeAndFinite(config_.friction_fp1, model_->nv(), "friction_fp1");
    requireSizeAndFinite(config_.friction_fp2, model_->nv(), "friction_fp2");
    requireSizeAndFinite(config_.friction_fp3, model_->nv(), "friction_fp3");
  }
}

JointVector OperationalSpaceController::compute(
    const JointState& state, const CartesianReference& reference) {
  requireSizeAndFinite(state.q, model_->nq(), "q");
  requireSizeAndFinite(state.dq, model_->nv(), "dq");
  requireFinitePose(reference.pose);
  requireFinite(reference.wrench, "wrench");
  if (reference.q_nullspace.size() != 0) {
    requireSizeAndFinite(
        reference.q_nullspace, model_->nv(), "q_nullspace");
  }

  const CartesianPose pose =
      model_->framePose(state.q, config_.end_effector_frame);
  if (!initialized_) {
    desired_position_ = pose.position;
    desired_orientation_ = pose.orientation.normalized();
    nullspace_reference_ = state.q;
    initialized_ = true;
  }
  const double alpha = config_.target_filter_alpha;
  desired_position_ =
      alpha * desired_position_ + (1.0 - alpha) * reference.pose.position;
  desired_orientation_ =
      reference.pose.orientation.normalized().slerp(
          alpha, desired_orientation_).normalized();
  if (reference.q_nullspace.size() != 0) {
    nullspace_reference_ = reference.q_nullspace;
  }

  CartesianVector error;
  const Eigen::Matrix3d current_rotation =
      pose.orientation.normalized().toRotationMatrix();
  const Eigen::Matrix3d desired_rotation =
      desired_orientation_.toRotationMatrix();
  if (config_.use_local_jacobian) {
    error.head<3>() =
        current_rotation.transpose() * (desired_position_ - pose.position);
    error.tail<3>() =
        pinocchio::log3(current_rotation.transpose() * desired_rotation);
  } else {
    error.head<3>() = desired_position_ - pose.position;
    error.tail<3>() =
        pinocchio::log3(desired_rotation * current_rotation.transpose());
  }
  ///误差限幅
  if (config_.limit_error) {
    error = error.cwiseMax(-config_.error_clip)
                .cwiseMin(config_.error_clip);
  }

  const Eigen::MatrixXd jacobian =
      config_.use_local_jacobian
          ? model_->localFrameJacobian(
                state.q, config_.end_effector_frame)
          : model_->frameJacobian(
                state.q, config_.end_effector_frame);
  const Eigen::MatrixXd inverse_mass =
      model_->inverseMassMatrix(state.q);
  ///任务惯性矩阵
  const Eigen::MatrixXd task_inertia = dampedPseudoInverse(
      jacobian * inverse_mass * jacobian.transpose(),
      config_.operational_space_regularization);
  JointVector torque =
      jacobian.transpose() * task_inertia *
      (config_.stiffness.cwiseProduct(error) -
       config_.damping.cwiseProduct(jacobian * state.dq));

  Eigen::MatrixXd projector =
      Eigen::MatrixXd::Identity(model_->nv(), model_->nv());

  ///运动学，保持末端速度不变，低精度场景
  if (config_.nullspace_projector == NullspaceProjector::kDynamic) {
    const Eigen::MatrixXd j_bar =
        inverse_mass * jacobian.transpose() * task_inertia;
    projector.noalias() -= jacobian.transpose() * j_bar.transpose();
  } else if (
      config_.nullspace_projector == NullspaceProjector::kKinematic) {
    projector.noalias() -=
        dampedPseudoInverse(
            jacobian, config_.nullspace_regularization) *
        jacobian;
  }///动力学一致伪逆，保持末端加速度不变，高精度场景
  JointVector secondary =
      config_.nullspace_stiffness *
          config_.nullspace_weights.cwiseProduct(
              nullspace_reference_ - state.q) -
      config_.nullspace_damping *
          config_.nullspace_weights.cwiseProduct(state.dq);
  JointVector nullspace_torque = projector * secondary;
  nullspace_torque =
      nullspace_torque
          .cwiseMax(-config_.nullspace_max_torque)
          .cwiseMin(config_.nullspace_max_torque);
  torque += nullspace_torque;

  if (config_.use_joint_limit_repulsion) {
    torque += jointLimitTorque(
        state.q, model_->lowerPositionLimits(),
        model_->upperPositionLimits(), config_.joint_limit_safe_range,
        config_.joint_limit_max_torque);
  }
  if (config_.use_friction) {
    torque += frictionTorque(
        state.dq, config_.friction_fp1, config_.friction_fp2,
        config_.friction_fp3);
  }
  if (config_.use_coriolis) {
    torque += model_->coriolis(state.q, state.dq);
  }
  if (config_.use_gravity) {
    torque += model_->gravity(state.q);
  }
  torque.noalias() += jacobian.transpose() * reference.wrench;
  requireFinite(torque, "operational space torque");
  return torque;
}

void OperationalSpaceController::reset() noexcept {
  initialized_ = false;
  nullspace_reference_.resize(0);
}

}  // namespace daedalus
