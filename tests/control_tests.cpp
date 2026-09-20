#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include <catch2/catch_test_macros.hpp>

#include "daedalus/control/cartesian_impedance_controller.hpp"
#include "daedalus/control/computed_torque_controller.hpp"
#include "daedalus/control/control_math.hpp"
#include "daedalus/control/gravity_compensator.hpp"
#include "daedalus/control/joint_impedance_controller.hpp"
#include "daedalus/control/operational_space_controller.hpp"
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

daedalus::OperationalSpaceConfig makeOperationalSpaceConfig() {
  daedalus::OperationalSpaceConfig config;
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

TEST_CASE("Local and world-aligned Jacobians differ at a bent pose") {
  const auto model = makeModel();
  JointVector q(2);
  q << 0.4, -0.7;
  const Eigen::MatrixXd world = model->frameJacobian(
      q, "link2", daedalus::JacobianReference::kLocalWorldAligned);
  const Eigen::MatrixXd local = model->frameJacobian(
      q, "link2", daedalus::JacobianReference::kLocal);
  REQUIRE(world.rows() == 6);
  REQUIRE(world.cols() == 2);
  REQUIRE(local.rows() == 6);
  REQUIRE(local.cols() == 2);
  REQUIRE((world - local).norm() > 1e-6);
}

TEST_CASE("Damped pseudo-inverse of identity is near identity") {
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(3, 3);
  const Eigen::MatrixXd inverse =
      daedalus::dampedPseudoInverse(identity, 1e-6);
  REQUIRE((inverse - identity).norm() < 1e-8);
}

TEST_CASE("Friction feedforward is zero at rest") {
  const JointVector zeros = JointVector::Zero(2);
  const JointVector torque = daedalus::frictionTorque(
      zeros, JointVector::Constant(2, 2.0), JointVector::Constant(2, 3.0),
      JointVector::Constant(2, 0.1));
  REQUIRE(torque.norm() < 1e-12);
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

TEST_CASE("Operational space controller reduces to gravity at zero error") {
  const auto model = makeModel();
  daedalus::OperationalSpaceController controller(
      model, makeOperationalSpaceConfig());
  const JointState state = zeroState();
  const auto reference =
      poseReference(model->framePose(state.q, "link2"));

  const JointVector torque = controller.compute(state, reference);
  REQUIRE((torque - model->gravity(state.q)).norm() < 1e-10);
}

TEST_CASE("Operational space controller produces finite task torque") {
  const auto model = makeModel();
  auto config = makeOperationalSpaceConfig();
  config.use_gravity = false;
  config.use_coriolis = false;
  config.use_joint_limit_repulsion = false;
  daedalus::OperationalSpaceController controller(model, config);
  const JointState state = zeroState();
  auto reference = poseReference(model->framePose(state.q, "link2"));
  reference.pose.position.z() += 0.05;

  const JointVector torque = controller.compute(state, reference);
  REQUIRE(torque.allFinite());
  REQUIRE(torque.norm() > 1e-6);
}

TEST_CASE("Operational space controller validates config and inputs") {
  const auto model = makeModel();
  auto config = makeOperationalSpaceConfig();
  config.operational_space_regularization = 0.0;
  REQUIRE_THROWS_AS(
      daedalus::OperationalSpaceController(model, config),
      std::invalid_argument);

  config = makeOperationalSpaceConfig();
  config.nullspace_projector =
      daedalus::NullspaceProjector::kKinematic;
  config.nullspace_stiffness = 10.0;
  daedalus::OperationalSpaceController controller(model, config);
  JointState state = zeroState();
  auto reference = poseReference(model->framePose(state.q, "link2"));
  reference.q_nullspace = JointVector::Ones(2);
  REQUIRE(controller.compute(state, reference).allFinite());

  reference.pose.orientation.coeffs()[0] =
      std::numeric_limits<double>::quiet_NaN();
  REQUIRE_THROWS_AS(
      controller.compute(state, reference), std::invalid_argument);
}

TEST_CASE("Operational space optional friction and joint-limit terms apply") {
  const auto model = makeModel();
  auto base_config = makeOperationalSpaceConfig();
  base_config.use_gravity = false;
  base_config.use_coriolis = false;
  base_config.use_joint_limit_repulsion = false;
  daedalus::OperationalSpaceController base(model, base_config);

  auto friction_config = base_config;
  friction_config.use_friction = true;
  friction_config.friction_fp1 = JointVector::Constant(2, 2.0);
  friction_config.friction_fp2 = JointVector::Constant(2, 3.0);
  friction_config.friction_fp3 = JointVector::Constant(2, 0.1);
  daedalus::OperationalSpaceController friction(model, friction_config);
  JointState moving{
      JointVector::Zero(2), JointVector::Constant(2, 0.2)};
  const auto moving_reference =
      poseReference(model->framePose(moving.q, "link2"));
  REQUIRE((friction.compute(moving, moving_reference) -
           base.compute(moving, moving_reference))
              .norm() > 1e-6);

  auto limit_config = base_config;
  limit_config.use_joint_limit_repulsion = true;
  daedalus::OperationalSpaceController no_limits(model, base_config);
  daedalus::OperationalSpaceController limits(model, limit_config);
  JointState near_limit = zeroState();
  near_limit.q[0] = 3.0;
  const auto limit_reference =
      poseReference(model->framePose(near_limit.q, "link2"));
  REQUIRE((limits.compute(near_limit, limit_reference) -
           no_limits.compute(near_limit, limit_reference))[0] < 0.0);
}

