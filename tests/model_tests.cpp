#include <limits>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "daedalus/model/pinocchio_model.hpp"
#include "daedalus/types/control_status.hpp"
#include "test_helpers.hpp"

using daedalus::JointVector;
using daedalus::test::frameJacobian;
using daedalus::test::framePose;
using daedalus::test::gravity;
using daedalus::test::inverseDynamics;
using daedalus::test::makeModel;

TEST_CASE("Pinocchio model exposes dimensions limits and consistent gravity") {
  const auto model = makeModel();
  REQUIRE(model->nq() == 2);
  REQUIRE(model->nv() == 2);
  REQUIRE(model->jointNames().size() == 2);
  REQUIRE((model->effortLimits() - (JointVector(2) << 50.0, 25.0).finished())
              .norm() < 1e-12);

  JointVector q(2);
  q << 0.3, -0.4;
  const JointVector gravity_torque = gravity(model, q);
  const JointVector rnea =
      inverseDynamics(model, q, JointVector::Zero(2), JointVector::Zero(2));
  REQUIRE((gravity_torque - rnea).norm() < 1e-12);
  REQUIRE(gravity_torque.norm() > 1e-6);
}

TEST_CASE("Pinocchio contexts are exclusive and model-bound") {
  static_assert(
      !std::is_copy_constructible_v<daedalus::PinocchioModel::Context>);
  static_assert(
      std::is_move_constructible_v<daedalus::PinocchioModel::Context>);

  const auto model = makeModel();
  auto first = model->createContext();
  auto second = model->createContext();
  JointVector q(2);
  q << 0.25, -0.5;

  const JointVector first_result = model->gravity(first, q);
  REQUIRE(model->frameJacobian(second, q, "link2").allFinite());
  REQUIRE((model->gravity(first, q) - first_result).norm() < 1e-12);

  const auto other_model = makeModel();
  REQUIRE_THROWS_AS(other_model->gravity(first, q), std::invalid_argument);
}

TEST_CASE("Shared model with exclusive contexts is safe concurrently") {
  const auto model = makeModel();
  JointVector q(2);
  q << 0.3, -0.4;
  const JointVector expected = gravity(model, q);
  std::vector<JointVector> results(8, JointVector::Zero(2));
  std::vector<daedalus::ControlStatus> statuses(
      results.size(), daedalus::ControlStatus::kInternalError);
  std::vector<std::thread> workers;
  workers.reserve(results.size());
  for (std::size_t index = 0; index < results.size(); ++index) {
    workers.emplace_back([&, index] {
      auto context = model->createContext();
      statuses[index] =
          model->gravityRealtime(context, q, results[index]).status;
    });
  }
  for (auto& worker : workers) {
    worker.join();
  }
  for (std::size_t index = 0; index < results.size(); ++index) {
    REQUIRE(statuses[index] == daedalus::ControlStatus::kOk);
    REQUIRE((results[index] - expected).norm() < 1e-12);
  }
}

TEST_CASE("Realtime model APIs reject invalid input without throwing") {
  const auto model = makeModel();
  auto context = model->createContext();
  JointVector output = JointVector::Ones(2);
  REQUIRE(model->gravityRealtime(context, JointVector::Zero(1), output)
              .status == daedalus::ControlStatus::kInvalidDimension);
  REQUIRE_FALSE(output.isZero());

  JointVector invalid = JointVector::Zero(2);
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  REQUIRE(model->gravityRealtime(context, invalid, output).status ==
          daedalus::ControlStatus::kNonFiniteInput);

  REQUIRE_THROWS_AS(model->gravity(context, JointVector::Zero(1)),
                    std::invalid_argument);
  REQUIRE_THROWS_AS(model->gravity(context, invalid), std::invalid_argument);
}

TEST_CASE("Pinocchio model exposes link2 kinematics") {
  const auto model = makeModel();
  REQUIRE(model->hasFrame("link2"));
  REQUIRE_FALSE(model->hasFrame("missing_frame"));

  const auto body_frames = model->bodyFrameNames();
  REQUIRE(body_frames.size() >= 2);
  REQUIRE(body_frames.back() == "link2");

  const JointVector q = JointVector::Zero(2);
  const daedalus::CartesianPose pose = framePose(model, q, "link2");
  REQUIRE((pose.position - Eigen::Vector3d(1.0, 0.0, 0.0)).norm() < 1e-12);

  const Eigen::MatrixXd jacobian = frameJacobian(model, q, "link2");
  REQUIRE(jacobian.rows() == 6);
  REQUIRE(jacobian.cols() == 2);
  REQUIRE_THROWS_AS(framePose(model, q, "missing_frame"),
                    std::invalid_argument);
}

TEST_CASE("Local and world-aligned Jacobians differ at a bent pose") {
  const auto model = makeModel();
  JointVector q(2);
  q << 0.4, -0.7;
  const Eigen::MatrixXd world = frameJacobian(
      model, q, "link2", daedalus::JacobianReference::kLocalWorldAligned);
  const Eigen::MatrixXd local =
      frameJacobian(model, q, "link2", daedalus::JacobianReference::kLocal);
  REQUIRE(world.rows() == 6);
  REQUIRE(world.cols() == 2);
  REQUIRE(local.rows() == 6);
  REQUIRE(local.cols() == 2);
  REQUIRE((world - local).norm() > 1e-6);
}
