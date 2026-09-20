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
#include "test_helpers.hpp"

using daedalus::JointReference;
using daedalus::JointState;
using daedalus::JointVector;
using daedalus::test::computeTorque;
using daedalus::test::frameJacobian;
using daedalus::test::framePose;
using daedalus::test::gravity;
using daedalus::test::makeCartesianConfig;
using daedalus::test::makeModel;
using daedalus::test::makeOperationalSpaceConfig;
using daedalus::test::poseReference;
using daedalus::test::zeroReference;
using daedalus::test::zeroState;

TEST_CASE("Computed torque contains exactly one gravity term") {
  const auto model = makeModel();
  daedalus::ComputedTorqueController controller(
      model, {JointVector::Zero(2), JointVector::Zero(2)});
  const JointState state = zeroState();
  const JointVector torque = computeTorque(controller, state, zeroReference());
  REQUIRE((torque - gravity(model, state.q)).norm() < 1e-12);
}

TEST_CASE("Joint impedance at zero error reduces to gravity") {
  const auto model = makeModel();
  daedalus::JointImpedanceController controller(
      model, {JointVector::Constant(2, 80.0), JointVector::Constant(2, 12.0)});
  const JointState state = zeroState();
  const JointVector torque = computeTorque(controller, state, zeroReference());
  REQUIRE((torque - gravity(model, state.q)).norm() < 1e-12);
}

TEST_CASE("Gravity compensator matches model gravity and ignores reference") {
  const auto model = makeModel();
  daedalus::GravityCompensator controller(model);
  JointState state{JointVector::Constant(2, 0.2), JointVector::Ones(2)};
  JointReference reference{
      JointVector::Constant(2, 1.0), JointVector::Constant(2, 2.0),
      JointVector::Constant(2, 3.0)};
  JointVector torque = JointVector::Zero(2);
  REQUIRE(controller.compute(state, reference, torque));
  REQUIRE((torque - gravity(model, state.q)).norm() < 1e-12);
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
  const JointVector torque = computeTorque(
      controller, state, poseReference(framePose(model, state.q, "link2")));
  REQUIRE((torque - gravity(model, state.q)).norm() < 1e-12);
}

TEST_CASE("Cartesian impedance translation error maps through J transpose") {
  const auto model = makeModel();
  daedalus::CartesianImpedanceController controller(
      model, makeCartesianConfig());
  const JointState state = zeroState();
  daedalus::CartesianReference reference =
      poseReference(framePose(model, state.q, "link2"));
  reference.pose.position.x() += 0.1;

  const JointVector torque = computeTorque(controller, state, reference);
  daedalus::CartesianVector force = daedalus::CartesianVector::Zero();
  force[0] = 200.0 * 0.1;
  const JointVector expected =
      frameJacobian(model, state.q, "link2").transpose() * force +
      gravity(model, state.q);
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
  JointVector torque = JointVector::Ones(2);
  const daedalus::ControlResult result = controller.compute(
      state, poseReference(framePose(model, JointVector::Zero(2), "link2")),
      torque);
  REQUIRE(result.status == daedalus::ControlStatus::kNonFiniteInput);
  REQUIRE(torque.isZero());
}

TEST_CASE("Operational space controller reduces to gravity at zero error") {
  const auto model = makeModel();
  daedalus::OperationalSpaceController controller(
      model, makeOperationalSpaceConfig());
  const JointState state = zeroState();
  const auto reference = poseReference(framePose(model, state.q, "link2"));

  const JointVector torque = computeTorque(controller, state, reference);
  REQUIRE((torque - gravity(model, state.q)).norm() < 1e-10);
}

TEST_CASE("Operational space controller produces finite task torque") {
  const auto model = makeModel();
  auto config = makeOperationalSpaceConfig();
  config.use_gravity = false;
  config.use_coriolis = false;
  config.use_joint_limit_repulsion = false;
  daedalus::OperationalSpaceController controller(model, config);
  const JointState state = zeroState();
  auto reference = poseReference(framePose(model, state.q, "link2"));
  reference.pose.position.z() += 0.05;

  const JointVector torque = computeTorque(controller, state, reference);
  REQUIRE(torque.allFinite());
  REQUIRE(torque.norm() > 1e-6);
}

TEST_CASE("Operational space controller validates config and inputs") {
  const auto model = makeModel();
  auto config = makeOperationalSpaceConfig();
  config.operational_space_regularization = 0.0;
  REQUIRE_THROWS_AS(daedalus::OperationalSpaceController(model, config),
                    std::invalid_argument);

  config = makeOperationalSpaceConfig();
  config.nullspace_projector = daedalus::NullspaceProjector::kKinematic;
  config.nullspace_stiffness = 10.0;
  daedalus::OperationalSpaceController controller(model, config);
  JointState state = zeroState();
  auto reference = poseReference(framePose(model, state.q, "link2"));
  reference.q_nullspace = JointVector::Ones(2);
  REQUIRE(computeTorque(controller, state, reference).allFinite());

  reference.pose.orientation.coeffs()[0] =
      std::numeric_limits<double>::quiet_NaN();
  JointVector torque = JointVector::Ones(2);
  const daedalus::ControlResult result =
      controller.compute(state, reference, torque);
  REQUIRE(result.status == daedalus::ControlStatus::kNonFiniteInput);
  REQUIRE(torque.isZero());
}

TEST_CASE("Realtime controller APIs are noexcept and validate inputs uniformly") {
  const auto model = makeModel();
  daedalus::GravityCompensator gravity_controller(model);
  daedalus::JointImpedanceController joint_controller(
      model, {JointVector::Constant(2, 80.0), JointVector::Constant(2, 12.0)});
  daedalus::ComputedTorqueController torque_controller(
      model, {JointVector::Zero(2), JointVector::Zero(2)});
  daedalus::CartesianImpedanceController cartesian_controller(
      model, makeCartesianConfig());
  daedalus::OperationalSpaceController operational_controller(
      model, makeOperationalSpaceConfig());
  const JointState state = zeroState();
  const JointReference joint_reference = zeroReference();
  const auto cartesian_reference =
      poseReference(framePose(model, state.q, "link2"));
  JointVector output = JointVector::Zero(2);

  static_assert(
      noexcept(gravity_controller.compute(state, joint_reference, output)));
  static_assert(
      noexcept(joint_controller.compute(state, joint_reference, output)));
  static_assert(
      noexcept(torque_controller.compute(state, joint_reference, output)));
  static_assert(noexcept(
      cartesian_controller.compute(state, cartesian_reference, output)));
  static_assert(noexcept(
      operational_controller.compute(state, cartesian_reference, output)));

  JointState wrong_dimension{JointVector::Zero(1), JointVector::Zero(2)};
  REQUIRE(gravity_controller.compute(wrong_dimension, joint_reference, output)
              .status == daedalus::ControlStatus::kInvalidDimension);
  REQUIRE(joint_controller.compute(wrong_dimension, joint_reference, output)
              .status == daedalus::ControlStatus::kInvalidDimension);
  REQUIRE(torque_controller.compute(wrong_dimension, joint_reference, output)
              .status == daedalus::ControlStatus::kInvalidDimension);
  REQUIRE(cartesian_controller
              .compute(wrong_dimension, cartesian_reference, output)
              .status == daedalus::ControlStatus::kInvalidDimension);
  REQUIRE(operational_controller
              .compute(wrong_dimension, cartesian_reference, output)
              .status == daedalus::ControlStatus::kInvalidDimension);

  JointVector wrong_output = JointVector::Ones(1);
  REQUIRE(gravity_controller.compute(state, joint_reference, wrong_output)
              .status == daedalus::ControlStatus::kInvalidDimension);

  JointReference nonfinite_reference = joint_reference;
  nonfinite_reference.ddq[0] = std::numeric_limits<double>::infinity();
  REQUIRE(torque_controller.compute(state, nonfinite_reference, output)
              .status == daedalus::ControlStatus::kNonFiniteInput);
  REQUIRE(output.isZero());
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
  JointState moving{JointVector::Zero(2), JointVector::Constant(2, 0.2)};
  const auto moving_reference =
      poseReference(framePose(model, moving.q, "link2"));
  REQUIRE((computeTorque(friction, moving, moving_reference) -
           computeTorque(base, moving, moving_reference))
              .norm() > 1e-6);

  auto limit_config = base_config;
  limit_config.use_joint_limit_repulsion = true;
  daedalus::OperationalSpaceController no_limits(model, base_config);
  daedalus::OperationalSpaceController limits(model, limit_config);
  JointState near_limit = zeroState();
  near_limit.q[0] = 3.0;
  const auto limit_reference =
      poseReference(framePose(model, near_limit.q, "link2"));
  REQUIRE((computeTorque(limits, near_limit, limit_reference) -
           computeTorque(no_limits, near_limit, limit_reference))[0] < 0.0);
}

TEST_CASE("Operational space realtime compute performs no Eigen allocation") {
  const auto model = makeModel();
  auto dynamic_config = makeOperationalSpaceConfig();
  dynamic_config.nullspace_stiffness = 10.0;
  dynamic_config.use_friction = true;
  dynamic_config.friction_fp1 = JointVector::Constant(2, 2.0);
  dynamic_config.friction_fp2 = JointVector::Constant(2, 3.0);
  dynamic_config.friction_fp3 = JointVector::Constant(2, 0.1);
  auto kinematic_config = dynamic_config;
  kinematic_config.nullspace_projector =
      daedalus::NullspaceProjector::kKinematic;
  daedalus::OperationalSpaceController dynamic_controller(
      model, dynamic_config);
  daedalus::OperationalSpaceController kinematic_controller(
      model, kinematic_config);
  const JointState state = zeroState();
  auto reference = poseReference(framePose(model, state.q, "link2"));
  reference.pose.position.z() += 0.01;
  reference.q_nullspace = JointVector::Constant(2, 0.1);
  JointVector dynamic_output = JointVector::Zero(2);
  JointVector kinematic_output = JointVector::Zero(2);

  REQUIRE(dynamic_controller.compute(state, reference, dynamic_output));
  REQUIRE(kinematic_controller.compute(state, reference, kinematic_output));
  Eigen::internal::set_is_malloc_allowed(false);
  const daedalus::ControlResult dynamic_result =
      dynamic_controller.compute(state, reference, dynamic_output);
  const daedalus::ControlResult kinematic_result =
      kinematic_controller.compute(state, reference, kinematic_output);
  Eigen::internal::set_is_malloc_allowed(true);

  REQUIRE(dynamic_result);
  REQUIRE(kinematic_result);
  REQUIRE(dynamic_output.allFinite());
  REQUIRE(kinematic_output.allFinite());
}

TEST_CASE("Joint and cartesian realtime compute perform no Eigen allocation") {
  const auto model = makeModel();
  daedalus::ComputedTorqueController torque_controller(
      model, {JointVector::Constant(2, 20.0), JointVector::Constant(2, 4.0)});
  daedalus::CartesianImpedanceController cartesian_controller(
      model, makeCartesianConfig());
  const JointState state = zeroState();
  const JointReference joint_reference = zeroReference();
  auto cartesian_reference =
      poseReference(framePose(model, state.q, "link2"));
  cartesian_reference.pose.position.x() += 0.02;
  cartesian_reference.q_nullspace = JointVector::Constant(2, 0.1);
  daedalus::CartesianImpedanceConfig nullspace_config = makeCartesianConfig();
  nullspace_config.nullspace_stiffness = 5.0;
  nullspace_config.nullspace_damping = 1.0;
  daedalus::CartesianImpedanceController nullspace_controller(
      model, nullspace_config);
  JointVector torque_output = JointVector::Zero(2);
  JointVector cartesian_output = JointVector::Zero(2);
  JointVector nullspace_output = JointVector::Zero(2);

  REQUIRE(torque_controller.compute(state, joint_reference, torque_output));
  REQUIRE(cartesian_controller.compute(
      state, cartesian_reference, cartesian_output));
  REQUIRE(nullspace_controller.compute(
      state, cartesian_reference, nullspace_output));
  Eigen::internal::set_is_malloc_allowed(false);
  const daedalus::ControlResult torque_result =
      torque_controller.compute(state, joint_reference, torque_output);
  const daedalus::ControlResult cartesian_result =
      cartesian_controller.compute(state, cartesian_reference,
                                   cartesian_output);
  const daedalus::ControlResult nullspace_result =
      nullspace_controller.compute(state, cartesian_reference,
                                   nullspace_output);
  Eigen::internal::set_is_malloc_allowed(true);

  REQUIRE(torque_result);
  REQUIRE(cartesian_result);
  REQUIRE(nullspace_result);
}
