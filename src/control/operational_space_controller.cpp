#include "daedalus/control/operational_space_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <pinocchio/spatial/explog.hpp>

#include "daedalus/common/vector_require.hpp"
#include "daedalus/control/control_validation.hpp"

namespace daedalus {

OperationalSpaceController::OperationalSpaceController(
    std::shared_ptr<const PinocchioModel> model, OperationalSpaceConfig config)
    : model_(std::move(model)),
      context_(model_ ? model_->createContext()
                      : throw std::invalid_argument("model must not be null")),
      config_(std::move(config)),
      nullspace_reference_(JointVector::Zero(model_->nq())),
      jacobian_(6, model_->nv()),
      inverse_mass_(model_->nv(), model_->nv()),
      task_matrix_(6, 6),
      task_inertia_(6, 6),
      temp_6xn_(6, model_->nv()),
      temp_nx6_(model_->nv(), 6),
      j_bar_(model_->nv(), 6),
      projector_(model_->nv(), model_->nv()),
      jacobian_pseudoinverse_(model_->nv(), 6),
      secondary_(model_->nv()),
      nullspace_torque_(model_->nv()),
      model_term_(model_->nv()),
      lower_limits_(model_->lowerPositionLimits()),
      upper_limits_(model_->upperPositionLimits()),
      task_inverse_workspace_(6, 6),
      jacobian_inverse_workspace_(6, model_->nv()) {
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

ControlResult OperationalSpaceController::compute(
    const JointState& state, const CartesianReference& reference,
    JointVector& output) noexcept {
  if (!model_) {
    return {ControlStatus::kNotInitialized};
  }
  if (output.size() != model_->nv()) {
    return {ControlStatus::kInvalidDimension};
  }
  output.setZero();
  ControlStatus status = validateJointState(state, model_->nq(), model_->nv());
  if (status == ControlStatus::kOk) {
    status = validateCartesianReference(reference, model_->nv());
  }
  if (status != ControlStatus::kOk) {
    return {status};
  }

  ControlResult result = model_->framePoseRealtime(
      context_, state.q, config_.end_effector_frame, pose_);
  if (!result) {
    return result;
  }
  if (!initialized_) {
    desired_position_ = pose_.position;
    desired_orientation_ = pose_.orientation.normalized();
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
      pose_.orientation.normalized().toRotationMatrix();
  const Eigen::Matrix3d desired_rotation =
      desired_orientation_.toRotationMatrix();
  const bool use_local =
      config_.jacobian_reference == JacobianReference::kLocal;
  if (use_local) {
    error.head<3>() =
        current_rotation.transpose() * (desired_position_ - pose_.position);
    error.tail<3>() =
        pinocchio::log3(current_rotation.transpose() * desired_rotation);
  } else {
    error.head<3>() = desired_position_ - pose_.position;
    error.tail<3>() =
        pinocchio::log3(desired_rotation * current_rotation.transpose());
  }
  result = model_->frameJacobianRealtime(
      context_, state.q, config_.end_effector_frame,
      config_.jacobian_reference, jacobian_);
  if (!result) {
    output.setZero();
    return result;
  }
  result = model_->inverseMassMatrixRealtime(
      context_, state.q, inverse_mass_);
  if (!result) {
    output.setZero();
    return result;
  }
  temp_6xn_.noalias() = jacobian_ * inverse_mass_;
  task_matrix_.noalias() = temp_6xn_ * jacobian_.transpose();
  result = dampedPseudoInverse(task_matrix_,
                               config_.operational_space_regularization,
                               task_inertia_, task_inverse_workspace_);
  if (!result) {
    output.setZero();
    return result;
  }
  task_velocity_.noalias() = jacobian_ * state.dq;
  for (Eigen::Index index = 0; index < 6; ++index) {
    if (config_.limit_error) {
      error[index] = std::clamp(error[index], -config_.error_clip[index],
                                config_.error_clip[index]);
    }
    task_command_[index] = config_.stiffness[index] * error[index] -
                           config_.damping[index] * task_velocity_[index];
  }
  task_velocity_.noalias() = task_inertia_ * task_command_;
  output.noalias() = jacobian_.transpose() * task_velocity_;

  projector_.setIdentity();

  if (config_.nullspace_projector == NullspaceProjector::kDynamic) {
    temp_nx6_.noalias() = inverse_mass_ * jacobian_.transpose();
    j_bar_.noalias() = temp_nx6_ * task_inertia_;
    projector_.noalias() -= jacobian_.transpose() * j_bar_.transpose();
  } else if (
      config_.nullspace_projector == NullspaceProjector::kKinematic) {
    result = dampedPseudoInverse(jacobian_, config_.nullspace_regularization,
                                 jacobian_pseudoinverse_,
                                 jacobian_inverse_workspace_);
    if (!result) {
      output.setZero();
      return result;
    }
    projector_.noalias() -= jacobian_pseudoinverse_ * jacobian_;
  }
  for (Eigen::Index index = 0; index < secondary_.size(); ++index) {
    secondary_[index] =
        config_.nullspace_stiffness * config_.nullspace_weights[index] *
            (nullspace_reference_[index] - state.q[index]) -
        config_.nullspace_damping * config_.nullspace_weights[index] *
            state.dq[index];
  }
  nullspace_torque_.noalias() = projector_ * secondary_;
  for (Eigen::Index index = 0; index < nullspace_torque_.size(); ++index) {
    nullspace_torque_[index] = std::clamp(
        nullspace_torque_[index], -config_.nullspace_max_torque,
        config_.nullspace_max_torque);
  }
  output += nullspace_torque_;

  if (config_.use_joint_limit_repulsion) {
    for (Eigen::Index i = 0; i < output.size(); ++i) {
      const double lower_ratio = std::clamp(
          (config_.joint_limit_safe_range -
           (state.q[i] - lower_limits_[i])) /
              config_.joint_limit_safe_range,
          0.0, 1.0);
      const double upper_ratio = std::clamp(
          (config_.joint_limit_safe_range -
           (upper_limits_[i] - state.q[i])) /
              config_.joint_limit_safe_range,
          0.0, 1.0);
      output[i] += config_.joint_limit_max_torque *
                   (lower_ratio - upper_ratio);
    }
  }
  if (config_.use_friction) {
    for (Eigen::Index i = 0; i < output.size(); ++i) {
      output[i] +=
          config_.friction_fp1[i] /
              (1.0 + std::exp(-config_.friction_fp2[i] *
                              (state.dq[i] + config_.friction_fp3[i]))) -
          config_.friction_fp1[i] /
              (1.0 + std::exp(-config_.friction_fp2[i] *
                              config_.friction_fp3[i]));
    }
  }
  if (config_.use_coriolis) {
    result = model_->coriolisRealtime(
        context_, state.q, state.dq, model_term_);
    if (!result) {
      output.setZero();
      return result;
    }
    output += model_term_;
  }
  if (config_.use_gravity) {
    result = model_->gravityRealtime(context_, state.q, model_term_);
    if (!result) {
      output.setZero();
      return result;
    }
    output += model_term_;
  }
  output.noalias() += jacobian_.transpose() * reference.wrench;
  return {validateTorqueOutput(output, model_->nv())};
}

JointVector OperationalSpaceController::compute(
    const JointState& state, const CartesianReference& reference) {
  JointVector torque(model_->nv());
  const ControlResult result = compute(state, reference, torque);
  if (!result) {
    throw std::runtime_error(controlStatusMessage(result.status));
  }
  return torque;
}

void OperationalSpaceController::reset() noexcept {
  initialized_ = false;
  nullspace_reference_.setZero();
}

}  // namespace daedalus
