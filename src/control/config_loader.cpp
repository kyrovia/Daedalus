#include "daedalus/control/config_loader.hpp"

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace daedalus {
namespace {

std::string joinKey(const std::string& parent, const char* key) {
  if (parent.empty()) {
    return key;
  }
  return parent + "." + key;
}

void requireMap(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsMap()) {
    throw std::invalid_argument(name + " must be a map");
  }
}

YAML::Node requireChild(const YAML::Node& parent, const char* key,
                        const std::string& parent_name) {
  const YAML::Node child = parent[key];
  if (!child || child.IsNull()) {
    throw std::invalid_argument("missing key '" + joinKey(parent_name, key) +
                                "'");
  }
  return child;
}

template <typename T>
T requireAs(const YAML::Node& node, const std::string& name) {
  try {
    return node.as<T>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(name + " has invalid type");
  }
}

JointVector parseVector(const YAML::Node& node, const Eigen::Index size,
                        const std::string& name) {
  if (node.IsScalar()) {
    return JointVector::Constant(size, requireAs<double>(node, name));
  }
  if (!node.IsSequence()) {
    throw std::invalid_argument(name + " must be a scalar or sequence");
  }
  if (static_cast<Eigen::Index>(node.size()) != size) {
    throw std::invalid_argument(name + " has incorrect size");
  }
  JointVector value(size);
  for (Eigen::Index index = 0; index < size; ++index) {
    value[index] = requireAs<double>(
        node[static_cast<std::size_t>(index)],
        name + "[" + std::to_string(index) + "]");
  }
  return value;
}

void loadOptional(const YAML::Node& parent, const char* key,
                  const std::string& parent_name, double& dest) {
  const YAML::Node node = parent[key];
  if (!node || node.IsNull()) {
    return;
  }
  dest = requireAs<double>(node, joinKey(parent_name, key));
}

void loadOptional(const YAML::Node& parent, const char* key,
                  const std::string& parent_name, bool& dest) {
  const YAML::Node node = parent[key];
  if (!node || node.IsNull()) {
    return;
  }
  dest = requireAs<bool>(node, joinKey(parent_name, key));
}

void loadOptional(const YAML::Node& parent, const char* key,
                  const std::string& parent_name, std::string& dest) {
  const YAML::Node node = parent[key];
  if (!node || node.IsNull()) {
    return;
  }
  dest = requireAs<std::string>(node, joinKey(parent_name, key));
}

void loadOptionalVector(const YAML::Node& parent, const char* key,
                        const std::string& parent_name, const Eigen::Index size,
                        JointVector& dest) {
  const YAML::Node node = parent[key];
  if (!node || node.IsNull()) {
    return;
  }
  dest = parseVector(node, size, joinKey(parent_name, key));
}

JacobianReference parseJacobianReference(const YAML::Node& node,
                                         const std::string& name) {
  const std::string value = requireAs<std::string>(node, name);
  if (value == "local") {
    return JacobianReference::kLocal;
  }
  if (value == "local_world_aligned") {
    return JacobianReference::kLocalWorldAligned;
  }
  throw std::invalid_argument("unknown jacobian_reference '" + value + "'");
}

NullspaceProjector parseNullspaceProjector(const YAML::Node& node,
                                           const std::string& name) {
  const std::string value = requireAs<std::string>(node, name);
  if (value == "dynamic") {
    return NullspaceProjector::kDynamic;
  }
  if (value == "kinematic") {
    return NullspaceProjector::kKinematic;
  }
  if (value == "none") {
    return NullspaceProjector::kNone;
  }
  throw std::invalid_argument("unknown nullspace_projector '" + value + "'");
}

SafetyLimits loadSafety(const YAML::Node& node, const int nv) {
  requireMap(node, "safety");
  SafetyLimits limits;
  limits.q_lower =
      parseVector(requireChild(node, "q_lower", "safety"), nv, "safety.q_lower");
  limits.q_upper =
      parseVector(requireChild(node, "q_upper", "safety"), nv, "safety.q_upper");
  limits.dq_max =
      parseVector(requireChild(node, "dq_max", "safety"), nv, "safety.dq_max");
  limits.ddq_max =
      parseVector(requireChild(node, "ddq_max", "safety"), nv, "safety.ddq_max");
  limits.tau_max =
      parseVector(requireChild(node, "tau_max", "safety"), nv, "safety.tau_max");
  limits.tau_rate_max = parseVector(
      requireChild(node, "tau_rate_max", "safety"), nv, "safety.tau_rate_max");
  return limits;
}

ComputedTorqueConfig loadComputedTorque(const YAML::Node& node, const int nv) {
  requireMap(node, "computed_torque");
  ComputedTorqueConfig config;
  config.kp = parseVector(requireChild(node, "kp", "computed_torque"), nv,
                          "computed_torque.kp");
  config.kd = parseVector(requireChild(node, "kd", "computed_torque"), nv,
                          "computed_torque.kd");
  return config;
}

JointImpedanceConfig loadJointImpedance(const YAML::Node& node, const int nv) {
  requireMap(node, "joint_impedance");
  JointImpedanceConfig config;
  config.stiffness = parseVector(
      requireChild(node, "stiffness", "joint_impedance"), nv,
      "joint_impedance.stiffness");
  config.damping = parseVector(requireChild(node, "damping", "joint_impedance"),
                               nv, "joint_impedance.damping");
  return config;
}

CartesianImpedanceConfig loadCartesianImpedance(const YAML::Node& node) {
  requireMap(node, "cartesian_impedance");
  CartesianImpedanceConfig config;
  config.stiffness = parseVector(
      requireChild(node, "stiffness", "cartesian_impedance"), 6,
      "cartesian_impedance.stiffness");
  loadOptionalVector(node, "damping", "cartesian_impedance", 6, config.damping);
  loadOptional(node, "nullspace_stiffness", "cartesian_impedance",
               config.nullspace_stiffness);
  loadOptional(node, "nullspace_damping", "cartesian_impedance",
               config.nullspace_damping);
  loadOptional(node, "end_effector_frame", "cartesian_impedance",
               config.end_effector_frame);
  return config;
}

OperationalSpaceConfig loadOperationalSpace(const YAML::Node& node,
                                            const int nv) {
  requireMap(node, "operational_space");
  OperationalSpaceConfig config;
  config.stiffness = parseVector(
      requireChild(node, "stiffness", "operational_space"), 6,
      "operational_space.stiffness");
  loadOptionalVector(node, "damping", "operational_space", 6, config.damping);
  loadOptional(node, "nullspace_stiffness", "operational_space",
               config.nullspace_stiffness);
  loadOptional(node, "nullspace_damping", "operational_space",
               config.nullspace_damping);
  loadOptionalVector(node, "nullspace_weights", "operational_space", nv,
                     config.nullspace_weights);
  loadOptional(node, "end_effector_frame", "operational_space",
               config.end_effector_frame);
  if (const YAML::Node reference = node["jacobian_reference"];
      reference && !reference.IsNull()) {
    config.jacobian_reference = parseJacobianReference(
        reference, "operational_space.jacobian_reference");
  }
  loadOptional(node, "operational_space_regularization", "operational_space",
               config.operational_space_regularization);
  loadOptional(node, "nullspace_regularization", "operational_space",
               config.nullspace_regularization);
  if (const YAML::Node projector = node["nullspace_projector"];
      projector && !projector.IsNull()) {
    config.nullspace_projector = parseNullspaceProjector(
        projector, "operational_space.nullspace_projector");
  }
  loadOptional(node, "limit_error", "operational_space", config.limit_error);
  loadOptionalVector(node, "error_clip", "operational_space", 6,
                     config.error_clip);
  loadOptional(node, "use_friction", "operational_space", config.use_friction);
  loadOptionalVector(node, "friction_fp1", "operational_space", nv,
                     config.friction_fp1);
  loadOptionalVector(node, "friction_fp2", "operational_space", nv,
                     config.friction_fp2);
  loadOptionalVector(node, "friction_fp3", "operational_space", nv,
                     config.friction_fp3);
  loadOptional(node, "use_coriolis", "operational_space", config.use_coriolis);
  loadOptional(node, "use_gravity", "operational_space", config.use_gravity);
  loadOptional(node, "use_joint_limit_repulsion", "operational_space",
               config.use_joint_limit_repulsion);
  loadOptional(node, "joint_limit_safe_range", "operational_space",
               config.joint_limit_safe_range);
  loadOptional(node, "joint_limit_max_torque", "operational_space",
               config.joint_limit_max_torque);
  loadOptional(node, "nullspace_max_torque", "operational_space",
               config.nullspace_max_torque);
  loadOptional(node, "target_filter_alpha", "operational_space",
               config.target_filter_alpha);
  return config;
}

}  // namespace

DaedalusConfig loadDaedalusConfig(const std::string& yaml_path, const int nv) {
  if (yaml_path.empty()) {
    throw std::invalid_argument("yaml path must not be empty");
  }
  if (nv <= 0) {
    throw std::invalid_argument("nv must be positive");
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("failed to load '" + yaml_path +
                             "': " + error.what());
  }
  requireMap(root, "config");

  DaedalusConfig config;
  config.safety = loadSafety(requireChild(root, "safety", ""), nv);
  config.computed_torque =
      loadComputedTorque(requireChild(root, "computed_torque", ""), nv);
  config.joint_impedance =
      loadJointImpedance(requireChild(root, "joint_impedance", ""), nv);
  config.cartesian_impedance = loadCartesianImpedance(
      requireChild(root, "cartesian_impedance", ""));
  config.operational_space = loadOperationalSpace(
      requireChild(root, "operational_space", ""), nv);
  return config;
}

}  // namespace daedalus
