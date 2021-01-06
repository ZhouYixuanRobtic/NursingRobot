#include "collision_detection/MoveItCollisionHelper.h"

namespace my_collision_detection{
    MoveItCollisionHelper::MoveItCollisionHelper(const std::string& yaml_name, const std::string &group_name)
        : _robot_model_loader_ptr(new robot_model_loader::RobotModelLoader("robot_description")),
          _robot_model_ptr(_robot_model_loader_ptr->getModel()),
          _planning_scene_ptr(new planning_scene::PlanningScene(_robot_model_ptr)),
          _kinematics_ptr(new kinematics::Kinematics(yaml_name)),
          _joint_model_group_ptr(_robot_model_ptr->getJointModelGroup(group_name))
    {

        _kinematics_ptr->setAnalyticalIK(kinematics::aubo_i5_analytical_IK);
    }

    bool MoveItCollisionHelper::isStateValid(const state_space::JointSpace &state)
    {
         moveit::core::RobotState current_state = _planning_scene_ptr->getCurrentState();
         _collision_result.clear();
         current_state.setJointGroupPositions(_joint_model_group_ptr,state.Vector());
         _planning_scene_ptr->checkSelfCollision(_collision_request,_collision_result,current_state);
         return !_collision_result.collision;
    }

    bool MoveItCollisionHelper::isStateValid(const state_space::SE3 &state)
    {
        //IK if one ik is valid then the state is valid
        state_space::vector_JointSpace allSolutions = _kinematics_ptr->allValidIKSolutions(state, nullptr);
        bool result = false;
        for(const auto &iter: allSolutions){
            result |= isStateValid(iter);
        }
        return result;
    }



}