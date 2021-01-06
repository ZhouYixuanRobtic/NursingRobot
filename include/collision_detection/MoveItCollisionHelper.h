#ifndef NURSINGROBOT_MOVEITCOLLISIONHELPER_HPP
#define NURSINGROBOT_MOVEITCOLLISIONHELPER_HPP

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/kinematic_constraints/utils.h>

#include "Kinematics/Kinematics.h"
#include "planner/Planner.hpp"
#include "Kinematics/custom_kinematics.hpp"

namespace my_collision_detection {

    class MoveItCollisionHelper {
    private:
        robot_model_loader::RobotModelLoaderPtr _robot_model_loader_ptr;

        robot_model::RobotModelPtr  _robot_model_ptr;

        planning_scene::PlanningScenePtr _planning_scene_ptr;

        collision_detection::CollisionRequest _collision_request;

        collision_detection::CollisionResult _collision_result;

        kinematics::KinematicsPtr _kinematics_ptr;

        robot_model::JointModelGroup* _joint_model_group_ptr;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        MoveItCollisionHelper(const std::string &yaml_name, const std::string &group_name);

        ~MoveItCollisionHelper() = default;

        bool isStateValid(const state_space::JointSpace &state);

        bool isStateValid(const state_space::SE3 &state);

        template<typename T>
        bool isPathValid(const T &from,
                         const T &to)
        {
            double step = 0;
            bool result =true;
            while(step<=1){
                T temp_state = planner::interpolate(from,to,step);
                result &= isStateValid(temp_state);
                step += 0.25;
            }
            return result;
        }



    };
}


#endif
