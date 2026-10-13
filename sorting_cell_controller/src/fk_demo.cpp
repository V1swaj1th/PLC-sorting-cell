#include <iostream>
#include "sorting_cell_controller/ur5e_kinematics.hpp"

int main() {
  using namespace sorting_cell;
  const Vector6d q = Vector6d::Zero();
  const Eigen::Matrix4d T = forwardKinematics(q);
  std::cout << "FK at q = 0, position:\n" << T.block<3, 1>(0, 3).transpose() << "\n";
  std::cout << "Rotation:\n" << T.block<3, 3>(0, 0) << "\n";
  return 0;
}
