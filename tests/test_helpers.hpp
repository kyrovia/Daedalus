#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "daedalus/control/controller_config.hpp"
#include "daedalus/model/pinocchio_model.hpp"
#include "daedalus/types/cartesian_types.hpp"
#include "daedalus/types/control_status.hpp"
#include "daedalus/types/joint_types.hpp"
#include "daedalus/types/safety_limits.hpp"

namespace daedalus {
namespace test {

inline std::shared_ptr<const PinocchioModel> makeModel() {
  return std::make_shared<const PinocchioModel>(DAEDALUS_TEST_URDF);
}

inline JointVector gravity(const std::shared_ptr<const PinocchioModel>& model,
                           const JointVector& q) {
  auto context = model->createContext();
  return model->gravity(context, q);
}

inline JointVector inverseDynamics(
    const std::shared_ptr<const PinocchioModel>& model, const JointVector& q,
    const JointVector& dq, const JointVector& ddq) {
  auto context = model->createContext();
  return model->inverseDynamics(context, q, dq, ddq);
}

inline CartesianPose framePose(
    const std::shared_ptr<const PinocchioModel>& model, const JointVector& q,
    const std::string& frame_name) {
  auto context = model->createContext();
  return model->framePose(context, q, frame_name);
}

inline Eigen::MatrixXd frameJacobian(
    const std::shared_ptr<const PinocchioModel>& model, const JointVector& q,
    const std::string& frame_name,
    const JacobianReference reference = JacobianReference::kLocalWorldAligned) {
  auto context = model->createContext();
  return model->frameJacobian(context, q, frame_name, reference);
}

inline SafetyLimits makeLimits(const double tau_max = 100.0,
                               const double tau_rate_max = 1000.0) {
  return {JointVector::Constant(2, -3.0), JointVector::Constant(2, 3.0),
          JointVector::Constant(2, 4.0), JointVector::Constant(2, 8.0),
          JointVector::Constant(2, tau_max),
          JointVector::Constant(2, tau_rate_max)};
}

inline JointState zeroState() {
  return {JointVector::Zero(2), JointVector::Zero(2)};
}

inline JointReference zeroReference() {
  return {JointVector::Zero(2), JointVector::Zero(2), JointVector::Zero(2)};
}

inline CartesianImpedanceConfig makeCartesianConfig() {
  CartesianImpedanceConfig config;
  config.stiffness.resize(6);
  config.stiffness << 200.0, 200.0, 200.0, 20.0, 20.0, 20.0;
  config.end_effector_frame = "link2";
  return config;
}

inline OperationalSpaceConfig makeOperationalSpaceConfig() {
  OperationalSpaceConfig config;
  config.stiffness.resize(6);
  config.stiffness << 200.0, 200.0, 200.0, 20.0, 20.0, 20.0;
  config.end_effector_frame = "link2";
  return config;
}

inline CartesianReference poseReference(const CartesianPose& pose) {
  CartesianReference reference;
  reference.pose = pose;
  return reference;
}

template <typename Controller, typename Reference>
inline JointVector computeTorque(Controller& controller, const JointState& state,
                                 const Reference& reference) {
  JointVector torque = JointVector::Zero(state.dq.size());
  const ControlResult result = controller.compute(state, reference, torque);
  if (!result) {
    throw std::runtime_error(controlStatusMessage(result.status));
  }
  return torque;
}

}  // namespace test
}  // namespace daedalus
