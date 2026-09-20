#include <limits>
#include <type_traits>

#include <catch2/catch_test_macros.hpp>

#include "daedalus/control/cartesian_impedance_controller.hpp"
#include "daedalus/control/control_pipeline.hpp"
#include "daedalus/control/joint_impedance_controller.hpp"
#include "daedalus/safety/reference_limiter.hpp"
#include "daedalus/safety/torque_filter.hpp"
#include "test_helpers.hpp"

using daedalus::JointReference;
using daedalus::JointState;
using daedalus::JointVector;
using daedalus::test::framePose;
using daedalus::test::makeCartesianConfig;
using daedalus::test::makeLimits;
using daedalus::test::makeModel;
using daedalus::test::poseReference;
using daedalus::test::zeroReference;
using daedalus::test::zeroState;

class RecordingJointController final {
 public:
  daedalus::ControlResult compute(const JointState&,
                                  const JointReference& reference,
                                  JointVector& torque) noexcept {
    ++calls;
    last_reference = reference;
    torque.setConstant(100.0);
    return {};
  }

  int calls{0};
  JointReference last_reference{JointVector::Zero(2), JointVector::Zero(2),
                                JointVector::Zero(2)};
};

TEST_CASE("Reference limiter clips commands and rejects unsafe measurements") {
  daedalus::ReferenceLimiter limiter(makeLimits());
  JointReference reference{(JointVector(2) << 9.0, -9.0).finished(),
                           (JointVector(2) << 7.0, -7.0).finished(),
                           (JointVector(2) << 20.0, -20.0).finished()};
  const JointReference safe = limiter.limit(reference);
  REQUIRE((safe.q - (JointVector(2) << 3.0, -3.0).finished()).norm() < 1e-12);
  REQUIRE((safe.dq - (JointVector(2) << 4.0, -4.0).finished()).norm() < 1e-12);
  REQUIRE((safe.ddq - (JointVector(2) << 8.0, -8.0).finished()).norm() <
          1e-12);

  JointState unsafe = zeroState();
  unsafe.q[0] = 3.1;
  REQUIRE_THROWS_AS(limiter.validateState(unsafe), std::out_of_range);
}

TEST_CASE("Torque filter applies absolute and rate limits") {
  daedalus::TorqueFilter filter(JointVector::Constant(2, 5.0),
                                JointVector::Constant(2, 10.0));
  filter.reset(JointVector::Zero(2));

  const JointVector first =
      filter.filter(JointVector::Constant(2, 100.0), 0.1);
  REQUIRE((first - JointVector::Ones(2)).norm() < 1e-12);

  filter.reset(JointVector::Zero(2));
  const JointVector saturated =
      filter.filter(JointVector::Constant(2, 100.0), 1.0);
  REQUIRE((saturated - JointVector::Constant(2, 5.0)).norm() < 1e-12);
}

TEST_CASE("Torque filter rejects use before reset without throwing") {
  daedalus::TorqueFilter filter(JointVector::Constant(2, 5.0),
                                JointVector::Constant(2, 10.0));
  JointVector filtered = JointVector::Zero(2);
  const daedalus::ControlResult result = filter.filterRealtime(
      JointVector::Constant(2, 1.0), 0.001, filtered);
  REQUIRE(result.status == daedalus::ControlStatus::kNotInitialized);
  REQUIRE_THROWS_AS(filter.filter(JointVector::Constant(2, 1.0), 0.001),
                    std::logic_error);
}

TEST_CASE("Control pipeline is the only producer of safe torque commands") {
  static_assert(!std::is_default_constructible_v<daedalus::TorqueCommand>);
  static_assert(
      !std::is_constructible_v<daedalus::TorqueCommand, Eigen::Index>);

  RecordingJointController controller;
  daedalus::ControlPipeline<RecordingJointController> pipeline(
      controller, makeLimits(5.0, 10.0));
  JointReference reference{JointVector::Constant(2, 20.0),
                           JointVector::Constant(2, -20.0),
                           JointVector::Constant(2, 20.0)};
  const JointState state = zeroState();

  static_assert(noexcept(pipeline.step(state, reference, 0.1)));
  const daedalus::ControlStepResult first =
      pipeline.step(state, reference, 0.1);
  REQUIRE(first.ok());
  REQUIRE(first.command != nullptr);
  REQUIRE((first.command->torque() - JointVector::Ones(2)).norm() < 1e-12);
  REQUIRE((controller.last_reference.q - JointVector::Constant(2, 3.0)).norm() <
          1e-12);
  REQUIRE(
      (controller.last_reference.dq - JointVector::Constant(2, -4.0)).norm() <
      1e-12);
  REQUIRE(
      (controller.last_reference.ddq - JointVector::Constant(2, 8.0)).norm() <
      1e-12);

  const daedalus::ControlStepResult saturated =
      pipeline.step(state, reference, 1.0);
  REQUIRE(saturated.ok());
  REQUIRE((saturated.command->torque() - JointVector::Constant(2, 5.0)).norm() <
          1e-12);
}

TEST_CASE("Control pipeline rejects unsafe inputs before controller compute") {
  RecordingJointController controller;
  daedalus::ControlPipeline<RecordingJointController> pipeline(
      controller, makeLimits());

  JointState unsafe = zeroState();
  unsafe.q[0] = 3.1;
  const daedalus::ControlStepResult unsafe_result =
      pipeline.step(unsafe, zeroReference(), 0.001);
  REQUIRE(unsafe_result.status ==
          daedalus::ControlStatus::kStateLimitViolation);
  REQUIRE(unsafe_result.command == nullptr);
  REQUIRE(controller.calls == 0);

  JointReference invalid = zeroReference();
  invalid.q[0] = std::numeric_limits<double>::quiet_NaN();
  const daedalus::ControlStepResult invalid_result =
      pipeline.step(zeroState(), invalid, 0.001);
  REQUIRE(invalid_result.status == daedalus::ControlStatus::kNonFiniteInput);
  REQUIRE(invalid_result.command == nullptr);
  REQUIRE(controller.calls == 0);
}

TEST_CASE("Control pipeline accepts existing joint reference controllers") {
  const auto model = makeModel();
  daedalus::JointImpedanceController controller(
      model, {JointVector::Constant(2, 80.0), JointVector::Constant(2, 12.0)});
  daedalus::ControlPipeline<daedalus::JointImpedanceController> pipeline(
      controller, makeLimits());

  const daedalus::ControlStepResult result =
      pipeline.step(zeroState(), zeroReference(), 0.001);
  REQUIRE(result.ok());
  REQUIRE(result.command->torque().allFinite());
}

TEST_CASE("Control pipeline saturates cartesian controller output") {
  const auto model = makeModel();
  daedalus::CartesianImpedanceController controller(
      model, makeCartesianConfig());
  daedalus::ControlPipeline<daedalus::CartesianImpedanceController> pipeline(
      controller, makeLimits(0.05, 10.0));
  const JointState state = zeroState();
  auto reference = poseReference(framePose(model, state.q, "link2"));
  reference.pose.position.x() += 0.5;

  const daedalus::ControlStepResult result =
      pipeline.step(state, reference, 1.0);
  REQUIRE(result.ok());
  REQUIRE((result.command->torque().cwiseAbs().array() <= 0.05 + 1e-12).all());
}

TEST_CASE("Safe control pipeline step performs no Eigen allocation") {
  const auto model = makeModel();
  daedalus::JointImpedanceController controller(
      model, {JointVector::Constant(2, 80.0), JointVector::Constant(2, 12.0)});
  daedalus::ControlPipeline<daedalus::JointImpedanceController> pipeline(
      controller, makeLimits());
  const JointState state = zeroState();
  const JointReference reference = zeroReference();

  REQUIRE(pipeline.step(state, reference, 0.001));
  Eigen::internal::set_is_malloc_allowed(false);
  const daedalus::ControlStepResult result =
      pipeline.step(state, reference, 0.001);
  Eigen::internal::set_is_malloc_allowed(true);

  REQUIRE(result);
  REQUIRE(result.command->torque().allFinite());
}
