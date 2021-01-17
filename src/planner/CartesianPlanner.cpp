#include "planner/CartesianPlanner.h"

namespace planner {
    static const std::size_t MIN_STEPS_FOR_JUMP_THRESH = 10;

    double CartesianPlanner::computeCartesianPath(state_space::JointSpace &current_joint_angles,
                                                  const state_space::SE3 &start,
                                                  const state_space::SE3 &target,
                                                  state_space::vector_JointSpace &seg_traj,
                                                  const std::string &reference_frame,
                                                  const my_collision_detection::MoveItCollisionHelper &moveItCollisionHelper)
    {
        const auto &kinematics_ptr = moveItCollisionHelper.getKinematicsPtr();
        state_space::SE3 start_pose;
        kinematics_ptr->getLinkTransform(start_pose, reference_frame, current_joint_angles);

        auto rotated_target = reference_frame == kinematics_ptr->getBaseName() ? target : start_pose + target;
        auto rotated_start = reference_frame == kinematics_ptr->getBaseName() ? start : start_pose + start;
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
        if (!moveItCollisionHelper.nearestSolution(current_joints, rotated_start, current_joint_angles, false))
            return 0.0;
        else
            seg_traj.emplace_back(current_joints);

        double last_valid_percentage = 0.0;
        for (std::size_t i = 1; i <= steps; ++i) {
            double percentage = (double) i / (double) steps;
            auto intermediate_state = planner::sphereInterpolate(rotated_start, rotated_target, percentage);
            if (moveItCollisionHelper.nearestSolution(current_joints, intermediate_state, current_joints, true))
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
        for (std::size_t i = 1; i < waypoints.size(); ++i) {
            state_space::vector_JointSpace waypoint_traj;
            double wp_percentage_solved = computeCartesianPath(current_joints, waypoints[i - 1], waypoints[i],
                                                               waypoint_traj,
                                                               reference_frame, moveItCollisionHelper);
            if (fabs(wp_percentage_solved - 1.0) < std::numeric_limits<double>::epsilon()) {
                percentage_solved = (double) (i + 1) / (double) waypoints.size();
                auto start = waypoint_traj.begin();
                if (i > 0 && !waypoint_traj.empty())
                    std::advance(start, 1);
                trajectory.insert(trajectory.end(), start, waypoint_traj.end());

            } else {
                percentage_solved += wp_percentage_solved / (double) waypoints.size();
                auto start = waypoint_traj.begin();
                if (i > 0 && !waypoint_traj.empty())
                    std::advance(start, 1);
                trajectory.insert(trajectory.end(), start, waypoint_traj.end());
                break;
            }
        }
        return percentage_solved;
    }

    void CartesianPlanner::getCartesianLine(state_space::vector_SE3 &waypoints,
                                            const state_space::SE3 &start_point,
                                            const state_space::SE3 &goal_point,
                                            const double cartesian_step)
    {
        waypoints.clear();
        double length = (start_point.translationPart()-goal_point.translationPart()).norm();

        //minimum 0.5 cm;
        double relative_step = cartesian_step < 0.005 ? 0.005/length : cartesian_step/length;
        if (relative_step >= 1) {
            waypoints.emplace_back(start_point);
            waypoints.emplace_back(goal_point);
        }
        double r_length = 0;
        while (r_length <= 1) {
            waypoints.emplace_back(planner::sphereInterpolate(start_point, goal_point, r_length));
            r_length += relative_step;
        }
    }

    void CartesianPlanner::getCartesianLine(state_space::vector_SE3 &waypoints,
                                            const state_space::SE3 &start_point,
                                            const Eigen::Vector3d &direction_vector,
                                            const double cartesian_step)
    {
        waypoints.clear();
        double length = direction_vector.norm();
        auto rotation = start_point.SO3Part();
        state_space::SE3 end_point = start_point+state_space::SE3(state_space::SO3::Zero(),direction_vector);
        //minimum 0.5 cm;
        double relative_step = cartesian_step < 0.005 ? 0.005/length : cartesian_step/length;
        if (relative_step >= 1) {
            waypoints.emplace_back(start_point);
            waypoints.emplace_back(end_point);
        }
        double r_length = 0;
        while (r_length <= 1) {
            waypoints.emplace_back(state_space::SE3(rotation,planner::interpolate(start_point.translationPart(),
                                                                                  end_point.translationPart(),
                                                                                  r_length)));
            r_length += relative_step;
        }
    }

    void CartesianPlanner::getCartesianCircle(state_space::vector_SE3 &waypoints,
                                              const state_space::SE3 &start_point,
                                              const state_space::SE3 &center,
                                              const double radian,
                                              const double step)
    {
        waypoints.clear();
        //change reference frame
        auto start = start_point-center;
        auto rotation_axis = state_space::SE3::UnitZ();
        //minimum 0.01 rad
        double relative_step = ((radian > 0.0) - (radian < 0.0))*std::max(0.01,std::abs(step));
        double r_radian = 0;
        while(fabs(r_radian)<fabs(radian))
        {
            auto temp = center+(rotation_axis(r_radian)+start);
            waypoints.emplace_back(temp);
            r_radian += relative_step;
        }
    }

    void CartesianPlanner::getCartesianCircle(state_space::vector_SE3 &waypoints,
                                              const Eigen::Vector3d &start_point,
                                              const Eigen::Vector3d &inter_point,
                                              const Eigen::Vector3d &end_point,
                                              double step)
    {
        //compute center;
        Eigen::Vector3d norm_vector = (end_point-inter_point).cross(inter_point-start_point);
        //when three points are collinear no solution;
        if(norm_vector.norm() < 1e-3)
            return;
        Eigen::Matrix3d T;
        T.row(0)= norm_vector.transpose();
        T.row(1)= (2*(inter_point-start_point)).transpose();
        T.row(2)= (2*(end_point-inter_point)).transpose();

        Eigen::Vector3d P{ norm_vector.dot(start_point),
                           inter_point.dot(inter_point)-start_point.dot(start_point),
                           end_point.dot(end_point)-inter_point.dot(inter_point)};

        Eigen::Vector3d center = T.inverse().eval()*P;
        double angle =  atan2((end_point-center)[1],(end_point-center)[0])-
                        atan2((start_point-center)[1],(start_point-center)[0]);

        return getCartesianCircle(waypoints,
                                  state_space::SE3(state_space::SO3::Zero(),start_point),
                                  state_space::SE3(state_space::SO3::Zero(),center),
                                  angle,
                                  step);
    }

}
