#pragma once

#include <Eigen/Core>

namespace daedalus {

using JointVector = Eigen::VectorXd;

struct JointState {
  JointVector q;
  JointVector dq;
};

struct JointReference {
  JointVector q;
  JointVector dq;
  JointVector ddq;
};

}  // namespace daedalus
