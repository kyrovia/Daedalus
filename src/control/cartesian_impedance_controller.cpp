#include "daedalus/control/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "daedalus/common/vector_require.hpp"
#include "daedalus/control/control_validation.hpp"

namespace daedalus {
namespace {

void orientationError(const Eigen::Quaterniond& desired,
                      Eigen::Quaterniond current,
                      Eigen::Ref<Eigen::Vector3d> error) {
  if (desired.coeffs().dot(current.coeffs()) < 0.0) {
    current.coeffs() = -current.coeffs();
  }
  const Eigen::AngleAxisd error_angle_axis(current * desired.inverse());
  error = error_angle_axis.axis() * error_angle_axis.angle();
}

}  // namespace

CartesianImpedanceController::CartesianImpedanceController(
    std::shared_ptr<const PinocchioModel> model, CartesianImpedanceConfig config)
    : model_(std::move(model)),
      context_(model_ ? model_->createContext()
                      : throw std::invalid_argument("model must not be null")),
      config_(std::move(config)),
      jacobian_(6, model_->nv()),
      jacobian_transpose_(model_->nv(), 6),
      jacobian_pinv_(6, model_->nv()),
      projector_(model_->nv(), model_->nv()),
      secondary_(model_->nv()),
      gravity_workspace_(model_->nv()),
      pinv_workspace_(model_->nv(), 6) {
  requireNonnegative(config_.stiffness, 6, "stiffness");
  if (config_.damping.size() == 0) {
    config_.damping = 2.0 * config_.stiffness.array().sqrt();
  } else {
    requireNonnegative(config_.damping, 6, "damping");
  }
  requireNonnegative(config_.nullspace_stiffness, "nullspace_stiffness");
  requireNonnegative(config_.nullspace_damping, "nullspace_damping");
  requirePositive(config_.error_clip, 6, "error_clip");
  if (config_.end_effector_frame.empty()) {
    throw std::invalid_argument("end_effector_frame must not be empty");
  }
  end_effector_frame_id_ = model_->frameId(config_.end_effector_frame);
}

ControlResult CartesianImpedanceController::compute(
    const JointState& state, const CartesianReference& reference,
    JointVector& output) const noexcept {
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
      context_, state.q, end_effector_frame_id_, pose_);
  if (!result) {
    return result;
  }
  result = model_->frameJacobianRealtime(
      context_, state.q, end_effector_frame_id_,
      JacobianReference::kLocalWorldAligned, jacobian_);
  if (!result) {
    return result;
  }

  error_.head<3>() = pose_.position - reference.pose.position;
  orientationError(reference.pose.orientation.normalized(),
                   pose_.orientation.normalized(), error_.tail<3>());
  if (config_.limit_error) {
    for (Eigen::Index index = 0; index < 6; ++index) {
      error_[index] = std::clamp(error_[index], -config_.error_clip[index],
                                 config_.error_clip[index]);
    }
  }

  task_wrench_.noalias() = jacobian_ * state.dq;
  for (Eigen::Index index = 0; index < 6; ++index) {
    task_wrench_[index] =
        -config_.stiffness[index] * error_[index] -
        config_.damping[index] * task_wrench_[index] +
        reference.wrench[index];
  }
  output.noalias() = jacobian_.transpose() * task_wrench_;

  const bool use_nullspace = reference.q_nullspace.size() != 0 &&
                             (config_.nullspace_stiffness > 0.0 ||
                              config_.nullspace_damping > 0.0);
  if (use_nullspace) {
    jacobian_transpose_.noalias() = jacobian_.transpose();
    result = dampedPseudoInverse(jacobian_transpose_, 0.2, jacobian_pinv_,
                                 pinv_workspace_);
    if (!result) {
      output.setZero();
      return result;
    }
    projector_.setIdentity();
    projector_.noalias() -= jacobian_transpose_ * jacobian_pinv_;
    for (Eigen::Index index = 0; index < secondary_.size(); ++index) {
      secondary_[index] =
          config_.nullspace_stiffness *
              (reference.q_nullspace[index] - state.q[index]) -
          config_.nullspace_damping * state.dq[index];
    }
    output.noalias() += projector_ * secondary_;
  }

  result = model_->gravityRealtime(context_, state.q, gravity_workspace_);
  if (!result) {
    output.setZero();
    return result;
  }
  output += gravity_workspace_;
  return {validateTorqueOutput(output, model_->nv())};
}

JointVector CartesianImpedanceController::compute(
    const JointState& state, const CartesianReference& reference) const {
  JointVector torque(model_->nv());
  const ControlResult result = compute(state, reference, torque);
  if (!result) {
    throw std::runtime_error(controlStatusMessage(result.status));
  }
  return torque;
}

}  // namespace daedalus
