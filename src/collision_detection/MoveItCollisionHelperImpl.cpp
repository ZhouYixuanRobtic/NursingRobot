#include "collision_detection/MoveItCollisionHelperImpl.h"

namespace my_collision_detection {

    MoveItCollisionHelperImpl::MoveItCollisionHelperImpl(const std::string &group_name,
                                                         const std::string &yaml_name,
                                                         const my_kinematics::analytical_ik_handled_t &analytical_ik_func,
                                                         JointStatesSubscriberPtr joint_state_subscribe_ptr)
            : _robot_model_loader_ptr(std::make_shared<robot_model_loader::RobotModelLoader>("robot_description")),
              _robot_model_ptr(_robot_model_loader_ptr->getModel()),
              _planning_scene_ptr(std::make_shared<planning_scene::PlanningScene>(_robot_model_ptr)),
              _kinematics_ptr(std::allocate_shared<my_kinematics::Kinematics>(Eigen::aligned_allocator<my_kinematics::Kinematics>(),yaml_name,analytical_ik_func)),
              _joint_model_group(_planning_scene_ptr->getCurrentState().getJointModelGroup(group_name)),
              _joint_state_subscribe_ptr(std::move(joint_state_subscribe_ptr))
    {

    }

    bool MoveItCollisionHelperImpl::isStateValid(const state_space::JointSpace &state) const
    {
        moveit::core::RobotState current_state = _planning_scene_ptr->getCurrentState();
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        current_state.setJointGroupPositions(_joint_model_group, state.Vector());
        _planning_scene_ptr->checkCollision(collision_request, collision_result, current_state);
        return !collision_result.collision;
    }

    bool MoveItCollisionHelperImpl::isStateValid(const state_space::SE3 &state) const
    {
        //IK if one ik is valid then the state is valid
        state_space::vector_JointSpace allSolutions = allValidSolutions(state, nullptr,true);
        return !allSolutions.empty();
    }

    state_space::vector_JointSpace
    MoveItCollisionHelperImpl::allValidSolutions(const state_space::SE3 &desired_pose,
                                                 const state_space::JointSpace *reference_ptr,
                                                 bool check_collision) const
    {
        Eigen::MatrixXd joint_solutions;
        _kinematics_ptr->allValidIkSolutions(joint_solutions, desired_pose.SE3Matrix(), reference_ptr);
        state_space::vector_JointSpace final_results{};
        for (int i = 0; i < joint_solutions.cols(); ++i) {
            auto temp = state_space::JointSpace(joint_solutions.col(i));
            if (check_collision) {
                if (isStateValid(temp)){
                    final_results.emplace_back(temp);
                }
            } else {
                final_results.emplace_back(temp);
            }
        }
        return final_results;
    }

    bool MoveItCollisionHelperImpl::nearestSolution(state_space::JointSpace& solution,
                                                    const state_space::SE3 &desired_pose,
                                                    const state_space::JointSpace &reference,
                                                    bool isConsecutive,
                                                    bool check_collision) const
    {
        //return the nearest ik solution
        return _kinematics_ptr->nearestIkSolution(solution,desired_pose,reference,isConsecutive)&&
               isStateValid(solution);
    }



}