#include "daedalus/model/pinocchio_model.hpp"

#include <stdexcept>
#include <utility>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace daedalus {
namespace {

void requireVector(const JointVector& value, const int expected,
                   const char* name) {
  if (value.size() != expected) {
    throw std::invalid_argument(std::string(name) + " has incorrect size");
  }
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(name) + " contains NaN or Inf");
  }
}

}  // namespace

struct PinocchioModel::Impl {
  pinocchio::Model model;
  mutable pinocchio::Data data;
  std::vector<std::string> joint_names;

  explicit Impl(pinocchio::Model loaded_model)
      : model(std::move(loaded_model)), data(model) {
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

JointVector PinocchioModel::effortLimits() const {
  return impl_->model.effortLimit;
}

JointVector PinocchioModel::gravity(const JointVector& q) const {
  requireVector(q, nq(), "q");
  return pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, q);
}

JointVector PinocchioModel::inverseDynamics(
    const JointVector& q, const JointVector& dq,
    const JointVector& ddq) const {
  requireVector(q, nq(), "q");
  requireVector(dq, nv(), "dq");
  requireVector(ddq, nv(), "ddq");
  return pinocchio::rnea(impl_->model, impl_->data, q, dq, ddq);
}

}  // namespace daedalus
