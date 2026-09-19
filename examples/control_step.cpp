#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "daedalus/control/daedalus_loop.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: daedalus_control_step <robot.urdf>\n";
    return 2;
  }

  try {
    auto model = std::make_shared<daedalus::PinocchioModel>(argv[1]);
    const int n = model->nv();

    daedalus::SafetyLimits limits{
        daedalus::JointVector::Constant(n, -3.14),
        daedalus::JointVector::Constant(n, 3.14),
        daedalus::JointVector::Constant(n, 4.0),
        daedalus::JointVector::Constant(n, 10.0),
        daedalus::JointVector::Constant(n, 100.0),
        daedalus::JointVector::Constant(n, 1000.0)};
    daedalus::ComputedTorqueConfig ctc{
        daedalus::JointVector::Constant(n, 100.0),
        daedalus::JointVector::Constant(n, 20.0)};
    daedalus::JointImpedanceConfig impedance{
        daedalus::JointVector::Constant(n, 80.0),
        daedalus::JointVector::Constant(n, 12.0)};
    daedalus::CartesianImpedanceConfig cartesian;
    cartesian.stiffness.resize(6);
    cartesian.stiffness << 200.0, 200.0, 200.0, 20.0, 20.0, 20.0;
    const auto body_frames = model->bodyFrameNames();
    if (body_frames.empty()) {
      throw std::runtime_error("URDF has no body frames");
    }
    cartesian.end_effector_frame = body_frames.back();

    daedalus::DaedalusLoop loop(
        model, limits, ctc, impedance, cartesian,
        daedalus::ControllerMode::kGravityCompensation,
        daedalus::JointVector::Zero(n));
    const daedalus::JointState state{
        daedalus::JointVector::Zero(n), daedalus::JointVector::Zero(n)};
    const daedalus::JointReference reference{
        daedalus::JointVector::Zero(n), daedalus::JointVector::Zero(n),
        daedalus::JointVector::Zero(n)};

    std::cout << "safe torque: "
              << loop.compute(state, reference, 0.001).transpose() << '\n';
  } catch (const std::exception& error) {
    std::cerr << "control step failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
