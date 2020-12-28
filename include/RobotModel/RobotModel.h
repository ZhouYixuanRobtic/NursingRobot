//
// Created by xcy on 2020/12/25.
//

#ifndef NURSINGROBOT_ROBOTMODEL_H
#define NURSINGROBOT_ROBOTMODEL_H

#include "StateSpace/SE3.hpp"
#include "StateSpace/rn.hpp"
#include "StateSpace/JointSpace.hpp"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
namespace robot_model{

    /**
     * @brief computes transform of every link
     * @note only supports revolute joint
     */
    class RobotModel {
    protected:
        /**@brief the name of yaml file containing the model information*/
        const std::string _model_config;

        /**@brief mount configuration describing how the robot was installed*/
        state_space::SE3 _mount_configuration{};

        /**@brief end effector configuration*/
        state_space::SE3 _ee_configuration{};

        /**@brief single home configuration for every joint*/
        std::map<std::string,state_space::SE3> _joint_configurations;

        /**@brief single screw axis using for computing transforms*/
        const state_space::SE3 _single_axis{state_space::SE3::UnitZ()};

        /**@brief the current joint angles */
        state_space::JointSpace _current_joint_angles;

        /**@brief stored path of every link*/
        std::map<std::string,std::string> _link_mesh_path;

    public:
        explicit RobotModel(std::string model_config);

        ~RobotModel() = default;

        /**@brief set current joint angles*/
        void setCurrentJointAngles(const state_space::JointSpace& joint_angles){
            _current_joint_angles = joint_angles;
        }
        /**@brief get stored current joint angles*/
        state_space::JointSpace getCurrentJointAngles()const{
            return _current_joint_angles;
        }

        /**@brief set end effector configuration*/
        void setEndEffector(const state_space::SE3 & ee_configuration){ _ee_configuration = ee_configuration;};
        /**@brief get stored end effector configuration*/
        state_space::SE3 getEndEffector() const{return _ee_configuration;};

        /**@brief set mount configuration*/
        void setMount(const state_space::SE3 & mount_configuration){ _mount_configuration = mount_configuration;};
        /**@brief get stored mount configuration*/
        state_space::SE3 getMount() const{return _mount_configuration;};

        /**@brief get stored mesh path of every link*/
        std::vector<std::string> getLinkMeshPaths() const;

        /**@brief compute transform for every link using current joint angles*/
        std::vector<state_space::SE3> computeTransform() const;

        /**@brief compute transform for every link using given joint angles*/
        std::vector<state_space::SE3>  computeTransform(const state_space::JointSpace& joint_angles) const;

    };
}



#endif //NURSINGROBOT_ROBOTMODEL_H
