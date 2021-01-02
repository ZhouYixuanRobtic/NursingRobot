#define EIGEN_MKL_USE_ALL

#include "RobotModel/RobotModel.h"
#include "StateSpace/SE3.hpp"
#include "StateSpace/JointSpace.hpp"
#include "planner/Planner.hpp"
#include "Kinematics/Kinematics.h"

int main() {
    Eigen::Matrix<double, 7, 1> pose_with_quaternion;

    pose_with_quaternion << 0.000, 0.000, 0.122, 0, 0, 1, 0;
    state_space::SE3 mount_configuration{pose_with_quaternion};
    pose_with_quaternion << -0.0405, 0.0, 0.14775, 0.000, -0.000, sqrt(2) / 2, sqrt(2) / 2;
    state_space::SE3 ee_configuration{pose_with_quaternion};

    robot_model::RobotModel aubo_model("../config/aubo_i5.yaml", state_space::JointSpace());
    aubo_model.setEndEffector(ee_configuration);
    aubo_model.setMount(mount_configuration);

    kinematics::Kinematics aubo_kinematics("../config/aubo_i5.yaml");
    aubo_kinematics.setEndEffector(ee_configuration);
    aubo_kinematics.setMount(mount_configuration);

    auto result = aubo_model.computeTransform();
    auto ee_pose = *(result.end() - 1);
    if (planner::distance(ee_pose, aubo_kinematics.fk(state_space::JointSpace())) < 1e-3) {
        std::cout << "success " << std::endl;
    }

    auto mesh_paths = aubo_model.getLinkMeshPaths();
    for (const auto &mesh_path: mesh_paths) {
        std::cout << mesh_path << std::endl;
    }

}
