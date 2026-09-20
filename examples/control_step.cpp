#include <cmath>
#include <iostream>
#include <memory>

#include "daedalus/control/control_pipeline.hpp"
#include "daedalus/control/joint_impedance_controller.hpp"
#include "daedalus/model/pinocchio_model.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: daedalus_control_step <robot.urdf>\n";
    return 1;
  }

  const auto model = std::make_shared<const daedalus::PinocchioModel>(argv[1]);
  const int n = model->nv();
  daedalus::JointImpedanceController controller(
      model, {daedalus::JointVector::Constant(n, 80.0),
              daedalus::JointVector::Constant(n, 12.0)});

  daedalus::JointVector tau_max = model->effortLimits();
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(tau_max[i]) || tau_max[i] <= 0.0) {
      tau_max[i] = 10.0;
    }
  }
  const daedalus::SafetyLimits limits{
      model->lowerPositionLimits(),
      model->upperPositionLimits(),
      daedalus::JointVector::Constant(n, 2.0),
      daedalus::JointVector::Constant(n, 8.0),
      tau_max,
      daedalus::JointVector::Constant(n, 50.0)};
  daedalus::ControlPipeline<daedalus::JointImpedanceController> pipeline(
      controller, limits);

  const daedalus::JointState state{daedalus::JointVector::Zero(n),
                                   daedalus::JointVector::Zero(n)};
  const daedalus::JointReference reference{
      daedalus::JointVector::Zero(n), daedalus::JointVector::Zero(n),
      daedalus::JointVector::Zero(n)};
  const daedalus::ControlStepResult result =
      pipeline.step(state, reference, 0.001);
  if (!result) {
    std::cerr << daedalus::controlStatusMessage(result.status) << '\n';
    return 1;
  }

  std::cout << result.command->torque().transpose() << '\n';
  return 0;
}
