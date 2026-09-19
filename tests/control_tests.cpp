#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include <catch2/catch_test_macros.hpp>

#include "daedalus/control/cartesian_impedance_controller.hpp"
#include "daedalus/control/computed_torque_controller.hpp"
#include "daedalus/control/daedalus_loop.hpp"
#include "daedalus/control/gravity_compensator.hpp"
#include "daedalus/control/joint_impedance_controller.hpp"
#include "daedalus/safety/reference_limiter.hpp"
#include "daedalus/safety/torque_filter.hpp"

namespace {

using daedalus::JointReference;
using daedalus::JointState;
using daedalus::JointVector;

std::shared_ptr<const daedalus::PinocchioModel> makeModel() {
  return std::make_shared<const daedalus::PinocchioModel>(
      DAEDALUS_TEST_URDF);
}

daedalus::SafetyLimits makeLimits(
    const double tau_max = 100.0, const double tau_rate_max = 1000.0) {
  return {
      JointVector::Constant(2, -3.0), JointVector::Constant(2, 3.0),
      JointVector::Constant(2, 4.0), JointVector::Constant(2, 8.0),
      JointVector::Constant(2, tau_max),
      JointVector::Constant(2, tau_rate_max)};
}

JointState zeroState() {
  return {JointVector::Zero(2), JointVector::Zero(2)};
}

JointReference zeroReference() {
  return {
      JointVector::Zero(2), JointVector::Zero(2), JointVector::Zero(2)};
}

daedalus::CartesianImpedanceConfig makeCartesianConfig() {
  daedalus::CartesianImpedanceConfig config;
  config.stiffness.resize(6);
  config.stiffness << 200.0, 200.0, 200.0, 20.0, 20.0, 20.0;
  config.end_effector_frame = "link2";
  return config;
}

daedalus::CartesianReference poseReference(const daedalus::CartesianPose& pose) {
  daedalus::CartesianReference reference;
  reference.pose = pose;
  return reference;
}

}  // namespace

TEST_CASE("Pinocchio model exposes dimensions limits and consistent gravity") {
  const auto model = makeModel();
  REQUIRE(model->nq() == 2);
  REQUIRE(model->nv() == 2);
  REQUIRE(model->jointNames().size() == 2);
  REQUIRE((model->effortLimits() - (JointVector(2) << 50.0, 25.0).finished())
              .norm() < 1e-12);

  JointVector q(2);
  q << 0.3, -0.4;
  const JointVector gravity = model->gravity(q);
  const JointVector rnea =
      model->inverseDynamics(q, JointVector::Zero(2), JointVector::Zero(2));
  REQUIRE((gravity - rnea).norm() < 1e-12);
  REQUIRE(gravity.norm() > 1e-6);
}

TEST_CASE("Computed torque contains exactly one gravity term") {
  const auto model = makeModel();
  daedalus::ComputedTorqueController controller(
      model, {JointVector::Zero(2), JointVector::Zero(2)});
  const JointState state = zeroState();
  const JointVector torque = controller.compute(state, zeroReference());
  REQUIRE((torque - model->gravity(state.q)).norm() < 1e-12);
}

TEST_CASE("Joint impedance at zero error reduces to gravity") {
  const auto model = makeModel();
  daedalus::JointImpedanceController controller(
      model, {JointVector::Constant(2, 80.0),
              JointVector::Constant(2, 12.0)});
  const JointState state = zeroState();
  const JointVector torque = controller.compute(state, zeroReference());
  REQUIRE((torque - model->gravity(state.q)).norm() < 1e-12);
}

TEST_CASE("Reference limiter clips commands and rejects unsafe measurements") {
  daedalus::ReferenceLimiter limiter(makeLimits());
  JointReference reference{
      (JointVector(2) << 9.0, -9.0).finished(),
      (JointVector(2) << 7.0, -7.0).finished(),
      (JointVector(2) << 20.0, -20.0).finished()};
  const JointReference safe = limiter.limit(reference);
  REQUIRE((safe.q - (JointVector(2) << 3.0, -3.0).finished()).norm() <
          1e-12);
  REQUIRE((safe.dq - (JointVector(2) << 4.0, -4.0).finished()).norm() <
          1e-12);
  REQUIRE((safe.ddq - (JointVector(2) << 8.0, -8.0).finished()).norm() <
          1e-12);

  JointState unsafe = zeroState();
  unsafe.q[0] = 3.1;
  REQUIRE_THROWS_AS(limiter.validateState(unsafe), std::out_of_range);
}

TEST_CASE("Torque filter applies absolute and rate limits") {
  daedalus::TorqueFilter filter(
      JointVector::Constant(2, 5.0), JointVector::Constant(2, 10.0));
  filter.reset(JointVector::Zero(2));

  const JointVector first =
      filter.filter(JointVector::Constant(2, 100.0), 0.1);
  REQUIRE((first - JointVector::Ones(2)).norm() < 1e-12);

  filter.reset(JointVector::Zero(2));
  const JointVector saturated =
      filter.filter(JointVector::Constant(2, 100.0), 1.0);
  REQUIRE((saturated - JointVector::Constant(2, 5.0)).norm() < 1e-12);
}

TEST_CASE("Invalid dimensions and nonfinite values are rejected") {
  const auto model = makeModel();
  REQUIRE_THROWS_AS(model->gravity(JointVector::Zero(1)),
                    std::invalid_argument);

  JointVector invalid = JointVector::Zero(2);
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  REQUIRE_THROWS_AS(model->gravity(invalid), std::invalid_argument);
}

TEST_CASE("DaedalusLoop mode switching preserves torque continuity") {
  const auto model = makeModel();
  auto limits = makeLimits(100.0, 10.0);
  daedalus::DaedalusLoop loop(
      model, limits,
      {JointVector::Constant(2, 100.0), JointVector::Constant(2, 20.0)},
      {JointVector::Constant(2, 80.0), JointVector::Constant(2, 12.0)},
      makeCartesianConfig(),
      daedalus::ControllerMode::kGravityCompensation,
      JointVector::Zero(2));

  const JointState state = zeroState();
  JointReference reference = zeroReference();
  const JointVector before = loop.compute(state, reference, 0.1);
  loop.setMode(
      daedalus::ControllerMode::kComputedTorque, before);
  reference.q = JointVector::Ones(2);
  const JointVector after = loop.compute(state, reference, 0.001);

  REQUIRE(((after - before).array().abs() <= 0.0100000001).all());
  REQUIRE(loop.mode() ==
          daedalus::ControllerMode::kComputedTorque);
}

TEST_CASE("Pinocchio model exposes link2 kinematics") {
  const auto model = makeModel();
  REQUIRE(model->hasFrame("link2"));
  REQUIRE_FALSE(model->hasFrame("missing_frame"));

  const auto body_frames = model->bodyFrameNames();
  REQUIRE(body_frames.size() >= 2);
  REQUIRE(body_frames.back() == "link2");

  const JointVector q = JointVector::Zero(2);
  const daedalus::CartesianPose pose = model->framePose(q, "link2");
  REQUIRE((pose.position - Eigen::Vector3d(1.0, 0.0, 0.0)).norm() < 1e-12);

  const Eigen::MatrixXd jacobian = model->frameJacobian(q, "link2");
  REQUIRE(jacobian.rows() == 6);
  REQUIRE(jacobian.cols() == 2);
  REQUIRE_THROWS_AS(model->framePose(q, "missing_frame"),
                    std::invalid_argument);
}

TEST_CASE("Cartesian impedance at zero error reduces to gravity") {
  const auto model = makeModel();
  daedalus::CartesianImpedanceController controller(
      model, makeCartesianConfig());
  const JointState state = zeroState();
  const JointVector torque = controller.compute(
      state, poseReference(model->framePose(state.q, "link2")));
  REQUIRE((torque - model->gravity(state.q)).norm() < 1e-12);
}

TEST_CASE("Cartesian impedance translation error maps through J transpose") {
  const auto model = makeModel();
  daedalus::CartesianImpedanceController controller(
      model, makeCartesianConfig());
  const JointState state = zeroState();
  daedalus::CartesianReference reference =
      poseReference(model->framePose(state.q, "link2"));
  reference.pose.position.x() += 0.1;

  const JointVector torque = controller.compute(state, reference);
  daedalus::CartesianVector force = daedalus::CartesianVector::Zero();
  force[0] = 200.0 * 0.1;
  const JointVector expected =
      model->frameJacobian(state.q, "link2").transpose() * force +
      model->gravity(state.q);
  REQUIRE((torque - expected).norm() < 1e-10);
}

TEST_CASE("Cartesian impedance rejects invalid configuration and input") {
  const auto model = makeModel();
  daedalus::CartesianImpedanceConfig config = makeCartesianConfig();
  config.stiffness = JointVector::Zero(3);
  REQUIRE_THROWS_AS(daedalus::CartesianImpedanceController(model, config),
                    std::invalid_argument);

  config = makeCartesianConfig();
  config.end_effector_frame = "missing_frame";
  REQUIRE_THROWS_AS(daedalus::CartesianImpedanceController(model, config),
                    std::invalid_argument);

  daedalus::CartesianImpedanceController controller(
      model, makeCartesianConfig());
  JointState state = zeroState();
  state.q[0] = std::numeric_limits<double>::quiet_NaN();
  REQUIRE_THROWS_AS(
      controller.compute(
          state, poseReference(model->framePose(JointVector::Zero(2), "link2"))),
      std::invalid_argument);
}

TEST_CASE("DaedalusLoop cartesian mode uses CartesianReference") {
  const auto model = makeModel();
  daedalus::DaedalusLoop loop(
      model, makeLimits(),
      {JointVector::Constant(2, 100.0), JointVector::Constant(2, 20.0)},
      {JointVector::Constant(2, 80.0), JointVector::Constant(2, 12.0)},
      makeCartesianConfig(),
      daedalus::ControllerMode::kCartesianImpedance, JointVector::Zero(2));

  const JointState state = zeroState();
  const daedalus::CartesianReference reference =
      poseReference(model->framePose(state.q, "link2"));
  const JointVector torque = loop.compute(state, reference, 0.1);
  REQUIRE((torque - model->gravity(state.q)).norm() < 1e-12);
  REQUIRE_THROWS_AS(loop.compute(state, zeroReference(), 0.001),
                    std::logic_error);
}
