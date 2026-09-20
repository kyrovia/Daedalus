#include "daedalus/model/pinocchio_model.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "daedalus/common/vector_require.hpp"

namespace daedalus {

struct PinocchioModel::Impl {
  pinocchio::Model model;
  std::vector<std::string> joint_names;

  explicit Impl(pinocchio::Model loaded_model)
      : model(std::move(loaded_model)) {
    joint_names.reserve(model.nv);
    const auto joint_count =
        static_cast<pinocchio::JointIndex>(model.njoints);
    for (pinocchio::JointIndex index = 1; index < joint_count; ++index) {
      if (model.joints[index].nv() > 0) {
        joint_names.push_back(model.names[index]);
      }
    }
  }
};

struct PinocchioModel::Context::Impl {
  const PinocchioModel::Impl* owner;
  pinocchio::Data data;

  explicit Impl(const PinocchioModel::Impl& model_impl)
      : owner(&model_impl), data(model_impl.model) {}
};

PinocchioModel::Context::Context(std::unique_ptr<Context::Impl> impl)
    : impl_(std::move(impl)) {}

PinocchioModel::Context::~Context() = default;
PinocchioModel::Context::Context(Context&&) noexcept = default;
PinocchioModel::Context& PinocchioModel::Context::operator=(
    Context&&) noexcept = default;

PinocchioModel::PinocchioModel(const std::string& urdf_path) {
  if (urdf_path.empty()) {
    throw std::invalid_argument("URDF path must not be empty");
  }

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  if (model.nv == 0) {
    throw std::runtime_error("robot model has no actuated degrees of freedom");
  }
  if (model.nq != model.nv) {
    throw std::runtime_error(
        "the fixed-base controller currently requires nq == nv");
  }
  impl_ = std::make_unique<Impl>(std::move(model));
}

PinocchioModel::~PinocchioModel() = default;
PinocchioModel::PinocchioModel(PinocchioModel&&) noexcept = default;
PinocchioModel& PinocchioModel::operator=(PinocchioModel&&) noexcept = default;

int PinocchioModel::nq() const noexcept {
  return impl_->model.nq;
}

int PinocchioModel::nv() const noexcept {
  return impl_->model.nv;
}

const std::vector<std::string>& PinocchioModel::jointNames() const noexcept {
  return impl_->joint_names;
}

std::vector<std::string> PinocchioModel::bodyFrameNames() const {
  std::vector<std::string> names;
  names.reserve(impl_->model.nframes);
  for (const auto& frame : impl_->model.frames) {
    if (frame.type == pinocchio::BODY) {
      names.push_back(frame.name);
    }
  }
  return names;
}

bool PinocchioModel::hasFrame(const std::string& frame_name) const {
  return impl_->model.existFrame(frame_name);
}

int PinocchioModel::frameId(const std::string& frame_name) const {
  if (!impl_->model.existFrame(frame_name)) {
    throw std::invalid_argument("unknown frame '" + frame_name + "'");
  }
  return static_cast<int>(impl_->model.getFrameId(frame_name));
}

JointVector PinocchioModel::effortLimits() const {
  return impl_->model.effortLimit;
}

JointVector PinocchioModel::lowerPositionLimits() const {
  return impl_->model.lowerPositionLimit;
}

JointVector PinocchioModel::upperPositionLimits() const {
  return impl_->model.upperPositionLimit;
}

PinocchioModel::Context PinocchioModel::createContext() const {
  return Context(std::make_unique<Context::Impl>(*impl_));
}

ControlResult PinocchioModel::validateContextRealtime(
    const Context& context) const noexcept {
  if (!context.impl_ || context.impl_->owner != impl_.get()) {
    return {ControlStatus::kModelError};
  }
  return {};
}

void PinocchioModel::throwIfFailed(
    const ControlResult result, const std::string& unknown_frame) {
  if (result) {
    return;
  }
  if (!unknown_frame.empty() && result.status == ControlStatus::kModelError) {
    throw std::invalid_argument("unknown frame '" + unknown_frame + "'");
  }
  throw std::invalid_argument(controlStatusMessage(result.status));
}

namespace {

pinocchio::ReferenceFrame pinocchioReference(
    const JacobianReference reference) {
  switch (reference) {
    case JacobianReference::kLocal:
      return pinocchio::LOCAL;
    case JacobianReference::kLocalWorldAligned:
      return pinocchio::LOCAL_WORLD_ALIGNED;
  }
  return pinocchio::LOCAL_WORLD_ALIGNED;
}

}  // namespace

ControlResult PinocchioModel::gravityRealtime(
    Context& context, const JointVector& q,
    Eigen::Ref<JointVector> output) const noexcept {
  ControlResult result = validateContextRealtime(context);
  if (!result) {
    return result;
  }
  result.status = validateSizeAndFinite(q, nq());
  if (!result) {
    return result;
  }
  result.status = validateMatrixShape(output.size(), 1, nv(), 1);
  if (!result) {
    return result;
  }
  auto& data = context.impl_->data;
  pinocchio::computeGeneralizedGravity(impl_->model, data, q);
  output = data.g;
  return {output.allFinite() ? ControlStatus::kOk
                             : ControlStatus::kNonFiniteOutput};
}

ControlResult PinocchioModel::coriolisRealtime(
    Context& context, const JointVector& q, const JointVector& dq,
    Eigen::Ref<JointVector> output) const noexcept {
  ControlResult result = validateContextRealtime(context);
  if (!result) {
    return result;
  }
  result.status = validateSizeAndFinite(q, nq());
  if (!result) {
    return result;
  }
  result.status = validateSizeAndFinite(dq, nv());
  if (!result) {
    return result;
  }
  result.status = validateMatrixShape(output.size(), 1, nv(), 1);
  if (!result) {
    return result;
  }
  auto& data = context.impl_->data;
  pinocchio::computeCoriolisMatrix(impl_->model, data, q, dq);
  output.noalias() = data.C * dq;
  return {output.allFinite() ? ControlStatus::kOk
                             : ControlStatus::kNonFiniteOutput};
}

ControlResult PinocchioModel::inverseMassMatrixRealtime(
    Context& context, const JointVector& q,
    Eigen::Ref<Eigen::MatrixXd> output) const noexcept {
  ControlResult result = validateContextRealtime(context);
  if (!result) {
    return result;
  }
  result.status = validateSizeAndFinite(q, nq());
  if (!result) {
    return result;
  }
  result.status = validateMatrixShape(output.rows(), output.cols(), nv(), nv());
  if (!result) {
    return result;
  }
  auto& data = context.impl_->data;
  pinocchio::computeMinverse(impl_->model, data, q);
  data.Minv.triangularView<Eigen::StrictlyLower>() =
      data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
  output = data.Minv;
  return {output.allFinite() ? ControlStatus::kOk
                             : ControlStatus::kNonFiniteOutput};
}

ControlResult PinocchioModel::inverseDynamicsRealtime(
    Context& context, const JointVector& q, const JointVector& dq,
    const JointVector& ddq, Eigen::Ref<JointVector> output) const noexcept {
  ControlResult result = validateContextRealtime(context);
  if (!result) {
    return result;
  }
  result.status = validateSizeAndFinite(q, nq());
  if (!result) {
    return result;
  }
  result.status = validateSizeAndFinite(dq, nv());
  if (!result) {
    return result;
  }
  result.status = validateSizeAndFinite(ddq, nv());
  if (!result) {
    return result;
  }
  result.status = validateMatrixShape(output.size(), 1, nv(), 1);
  if (!result) {
    return result;
  }
  auto& data = context.impl_->data;
  pinocchio::rnea(impl_->model, data, q, dq, ddq);
  output = data.tau;
  return {output.allFinite() ? ControlStatus::kOk
                             : ControlStatus::kNonFiniteOutput};
}

ControlResult PinocchioModel::framePoseRealtime(
    Context& context, const JointVector& q, const int frame_id,
    CartesianPose& output) const noexcept {
  ControlResult result = validateContextRealtime(context);
  if (!result) {
    return result;
  }
  result.status = validateSizeAndFinite(q, nq());
  if (!result) {
    return result;
  }
  if (frame_id < 0 ||
      frame_id >= static_cast<int>(impl_->model.nframes)) {
    return {ControlStatus::kModelError};
  }
  auto& data = context.impl_->data;
  pinocchio::forwardKinematics(impl_->model, data, q);
  pinocchio::updateFramePlacements(impl_->model, data);
  const pinocchio::SE3& placement =
      data.oMf[static_cast<pinocchio::FrameIndex>(frame_id)];
  output.position = placement.translation();
  output.orientation = Eigen::Quaterniond(placement.rotation());
  if (!output.position.allFinite() ||
      !output.orientation.coeffs().allFinite()) {
    return {ControlStatus::kNonFiniteOutput};
  }
  return {};
}

ControlResult PinocchioModel::framePoseRealtime(
    Context& context, const JointVector& q, const std::string& frame_name,
    CartesianPose& output) const noexcept {
  if (!impl_->model.existFrame(frame_name)) {
    return {ControlStatus::kModelError};
  }
  return framePoseRealtime(
      context, q, static_cast<int>(impl_->model.getFrameId(frame_name)),
      output);
}

ControlResult PinocchioModel::frameJacobianRealtime(
    Context& context, const JointVector& q, const int frame_id,
    const JacobianReference reference,
    Eigen::Ref<Eigen::MatrixXd> output) const noexcept {
  ControlResult result = validateContextRealtime(context);
  if (!result) {
    return result;
  }
  result.status = validateSizeAndFinite(q, nq());
  if (!result) {
    return result;
  }
  result.status = validateMatrixShape(output.rows(), output.cols(), 6, nv());
  if (!result) {
    return result;
  }
  if (frame_id < 0 ||
      frame_id >= static_cast<int>(impl_->model.nframes)) {
    return {ControlStatus::kModelError};
  }
  auto& data = context.impl_->data;
  pinocchio::computeFrameJacobian(
      impl_->model, data, q, static_cast<pinocchio::FrameIndex>(frame_id),
      pinocchioReference(reference), output);
  return {output.allFinite() ? ControlStatus::kOk
                             : ControlStatus::kNonFiniteOutput};
}

ControlResult PinocchioModel::frameJacobianRealtime(
    Context& context, const JointVector& q, const std::string& frame_name,
    const JacobianReference reference,
    Eigen::Ref<Eigen::MatrixXd> output) const noexcept {
  if (!impl_->model.existFrame(frame_name)) {
    return {ControlStatus::kModelError};
  }
  return frameJacobianRealtime(
      context, q, static_cast<int>(impl_->model.getFrameId(frame_name)),
      reference, output);
}

JointVector PinocchioModel::gravity(Context& context,
                                    const JointVector& q) const {
  JointVector output(nv());
  gravity(context, q, output);
  return output;
}

void PinocchioModel::gravity(Context& context, const JointVector& q,
                             Eigen::Ref<JointVector> output) const {
  throwIfFailed(gravityRealtime(context, q, output));
}

JointVector PinocchioModel::coriolis(
    Context& context, const JointVector& q, const JointVector& dq) const {
  JointVector output(nv());
  coriolis(context, q, dq, output);
  return output;
}

void PinocchioModel::coriolis(
    Context& context, const JointVector& q, const JointVector& dq,
    Eigen::Ref<JointVector> output) const {
  throwIfFailed(coriolisRealtime(context, q, dq, output));
}

Eigen::MatrixXd PinocchioModel::inverseMassMatrix(
    Context& context, const JointVector& q) const {
  Eigen::MatrixXd output(nv(), nv());
  inverseMassMatrix(context, q, output);
  return output;
}

void PinocchioModel::inverseMassMatrix(
    Context& context, const JointVector& q,
    Eigen::Ref<Eigen::MatrixXd> output) const {
  throwIfFailed(inverseMassMatrixRealtime(context, q, output));
}

JointVector PinocchioModel::inverseDynamics(
    Context& context, const JointVector& q, const JointVector& dq,
    const JointVector& ddq) const {
  JointVector output(nv());
  inverseDynamics(context, q, dq, ddq, output);
  return output;
}

void PinocchioModel::inverseDynamics(
    Context& context, const JointVector& q, const JointVector& dq,
    const JointVector& ddq, Eigen::Ref<JointVector> output) const {
  throwIfFailed(inverseDynamicsRealtime(context, q, dq, ddq, output));
}

CartesianPose PinocchioModel::framePose(
    Context& context, const JointVector& q,
    const std::string& frame_name) const {
  CartesianPose output;
  framePose(context, q, frame_name, output);
  return output;
}

void PinocchioModel::framePose(
    Context& context, const JointVector& q, const std::string& frame_name,
    CartesianPose& output) const {
  throwIfFailed(framePoseRealtime(context, q, frame_name, output),
                frame_name);
}

Eigen::MatrixXd PinocchioModel::frameJacobian(
    Context& context, const JointVector& q, const std::string& frame_name,
    const JacobianReference reference) const {
  Eigen::MatrixXd jacobian(6, nv());
  frameJacobian(context, q, frame_name, reference, jacobian);
  return jacobian;
}

void PinocchioModel::frameJacobian(
    Context& context, const JointVector& q, const std::string& frame_name,
    const JacobianReference reference,
    Eigen::Ref<Eigen::MatrixXd> output) const {
  throwIfFailed(frameJacobianRealtime(context, q, frame_name, reference,
                                      output),
                frame_name);
}

}  // namespace daedalus
