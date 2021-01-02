#include "RobotModel/RobotModel.h"

using namespace robot_model;

RobotModel::RobotModel(std::string model_config, const state_space::JointSpace& joint_angles)
        : _model_config(std::move(model_config))
{
    _current_joint_angles = joint_angles;
    YAML::Node doc = YAML::LoadFile(_model_config);
    try {
        _link_mesh_path = doc["aubo_i5"]["link_paths"].as<std::map<std::string, std::string>>();

        std::map<std::string, std::vector<double>> config_map;
        config_map = doc["aubo_i5"]["joint_configuration"].as<std::map<std::string, std::vector<double>>>();

        for (const auto& it : config_map) {
            if (it.second.size() != 7) {
                char buf[100];
                sprintf(buf, "wrong pose, should have 7 elements but have %d elements", (int) it.second.size());
                throw std::invalid_argument(buf);
            } else {
                Eigen::Matrix<double, 7, 1> temp_pose;
                memcpy(temp_pose.data(), it.second.data(), temp_pose.size() * sizeof(double));
                _joint_configurations.insert(std::make_pair(it.first, state_space::SE3(temp_pose)));
            }
        }

    }
    catch (YAML::InvalidScalar) {
        perror("tagParam.yaml is invalid.");
    }
}

state_space::vector_SE3 RobotModel::computeTransform() const
{
    state_space::vector_SE3 result{};
    state_space::SE3 temp_SE3 = _mount_configuration;
    for (auto it = _joint_configurations.begin(); it != _joint_configurations.end(); it++) {
        size_t i = std::distance(_joint_configurations.begin(), it);
        temp_SE3 = temp_SE3 + _single_axis(_current_joint_angles[i]) + it->second;
        result.emplace_back(temp_SE3);
    }
    result.emplace_back(temp_SE3 + _ee_configuration);
    return result;
}

state_space::vector_SE3 RobotModel::computeTransform(const state_space::JointSpace& joint_angles) const
{
    state_space::vector_SE3 result{};
    state_space::SE3 temp_SE3 = _mount_configuration;
    for (auto it = _joint_configurations.begin(); it != _joint_configurations.end(); it++) {
        size_t i = std::distance(_joint_configurations.begin(), it);
        temp_SE3 = temp_SE3 + _single_axis(joint_angles[i]) + it->second;
        result.emplace_back(temp_SE3);
    }
    result.emplace_back(temp_SE3 + _ee_configuration);
    return result;
}

std::vector<std::string> RobotModel::getLinkMeshPaths() const
{
    std::vector<std::string> paths;
    auto iter = _link_mesh_path.begin();
    while (iter != _link_mesh_path.end()) {
        paths.emplace_back(iter->second);
        iter++;
    }
    return paths;
}