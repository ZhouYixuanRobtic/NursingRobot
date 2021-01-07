#include "planner/CartesianPlanner.h"
namespace planner{
    static const std::size_t MIN_STEPS_FOR_JUMP_THRESH = 10;
    double CartesianPlanner::computeCartesianPath(state_space::JointSpace &current_joint_angles,
                                                  const state_space::SE3 &start,
                                                  const state_space::SE3 &target,
                                                  state_space::vector_JointSpace &seg_traj,
                                                  const std::string &reference_frame,
                                                  const my_collision_detection::MoveItCollisionHelper &moveItCollisionHelper)
    {
        const auto& kinematics_ptr = moveItCollisionHelper.getKinematicsPtr();
        state_space::SE3 start_pose;
        kinematics_ptr->getLinkTransform(start_pose,reference_frame,current_joint_angles);

        auto rotated_target = reference_frame==kinematics_ptr->getBaseName() ? target : start_pose+target;
        auto rotated_start = reference_frame==kinematics_ptr->getBaseName() ? start: start_pose+start;
        /*
        double translation_distance = planner::distance(rotated_start.translationPart(),rotated_target.translationPart());
        double rotation_distance = planner::distance(rotated_start.SO3Part(),rotated_target.SO3Part());
        std::size_t translation_steps = 0;
        if (max_step.translation > 0.0)
            translation_steps = floor(translation_distance / max_step.translation);

        std::size_t rotation_steps = 0;
        if (max_step.rotation > 0.0)
            rotation_steps = floor(rotation_distance / max_step.rotation);

        // If we are testing for relative jumps, we always want at least MIN_STEPS_FOR_JUMP_THRESH steps
        std::size_t steps = std::max(translation_steps, rotation_steps) + 1;*/
        //TODO: adaptive step control;
        std::size_t steps = MIN_STEPS_FOR_JUMP_THRESH;

        seg_traj.clear();
        //start
        state_space::JointSpace current_joints;
        if(!moveItCollisionHelper.nearestSolution(current_joints, rotated_start, current_joint_angles, false))
            return 0.0;
        else
            seg_traj.emplace_back(current_joints);

        double last_valid_percentage = 0.0;
        for (std::size_t i = 1; i <= steps; ++i){
            double percentage = (double)i / (double)steps;
            auto intermediate_state = planner::sphereInterpolate(rotated_start,rotated_target,percentage);
            if(moveItCollisionHelper.nearestSolution(current_joints,intermediate_state,current_joints,true))
                seg_traj.emplace_back(current_joints);
            else
                break;
            last_valid_percentage = percentage;
        }

        return last_valid_percentage;
    }

     double CartesianPlanner::computeCartesianPath(const state_space::JointSpace &current_joint_angles,
                                                  const state_space::vector_SE3 &waypoints,
                                                  state_space::vector_JointSpace &trajectory,
                                                  const std::string &reference_frame,
                                                  const my_collision_detection::MoveItCollisionHelper &moveItCollisionHelper)
    {
        double percentage_solved = 0.0;
        state_space::SE3 start_point = *waypoints.begin();
        state_space::JointSpace current_joints = current_joint_angles;
        for (std::size_t i = 1; i < waypoints.size(); ++i)
        {
            state_space::vector_JointSpace waypoint_traj;
            double wp_percentage_solved = computeCartesianPath(current_joints,waypoints[i-1],waypoints[i],waypoint_traj,
                                         reference_frame,moveItCollisionHelper);
            if (fabs(wp_percentage_solved - 1.0) < std::numeric_limits<double>::epsilon())
            {
                percentage_solved = (double)(i + 1) / (double)waypoints.size();
                auto start = waypoint_traj.begin();
                if (i > 0 && !waypoint_traj.empty())
                    std::advance(start, 1);
                trajectory.insert(trajectory.end(),start,waypoint_traj.end());

            }
            else
            {
                percentage_solved += wp_percentage_solved / (double)waypoints.size();
                auto start = waypoint_traj.begin();
                if (i > 0 && !waypoint_traj.empty())
                    std::advance(start, 1);
                trajectory.insert(trajectory.end(),start,waypoint_traj.end());
                break;
            }
        }
        return percentage_solved;
    }
}