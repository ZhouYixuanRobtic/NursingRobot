#include "planner/CartesianPlanner.h"

namespace planner {
    static const std::size_t MIN_STEPS_FOR_JUMP_THRESH = 10;

    double CartesianPlanner::checkJointSpaceJump(state_space::vector_JointSpace &traj,
                                                 const JumpThreshold &jump_threshold)
    {
        double percentage_solved{1.0};
        if (traj.size() <= 2)
            return percentage_solved;

        if (jump_threshold.factor > 0.0)
            percentage_solved *= checkRelativeJointSpaceJump(traj, jump_threshold.factor);

        if (jump_threshold.revolute > 0.0 || jump_threshold.prismatic > 0.0)
            percentage_solved *= checkAbsoluteJointSpaceJump(traj, jump_threshold.revolute, jump_threshold.prismatic);

        return percentage_solved;
    }

    double CartesianPlanner::checkAbsoluteJointSpaceJump(state_space::vector_JointSpace &traj,
                                                         double revolute_threshold,
                                                         double prismatic_threshold)
    {
        //TODO: add prismatic threshold
        double joint_threshold = revolute_threshold;
        std::size_t joint_nums = traj[0].size();
        bool still_valid = true;
        for (std::size_t traj_ix = 0, ix_end = traj.size() - 1; traj_ix != ix_end; ++traj_ix) {
            for (std::size_t index; index < joint_nums; ++index) {
                if (revolute_threshold > 0.0) {
                    double distance = fabs((traj[traj_ix] - traj[traj_ix + 1])[index]);
                    if (distance > joint_threshold) {
                        DLOG(WARNING)   << "Truncating Cartesian path due to detected jump of "
                                        <<distance<< ">" << joint_threshold<<" in joint "<< index;
                        still_valid = false;
                        break;
                    }
                }
            }
            if (!still_valid) {
                double percent_valid = (double) (traj_ix + 1) / (double) (traj.size());
                traj.resize(traj_ix + 1);
                return percent_valid;
            }
        }
        return 1.0;
    }

    double CartesianPlanner::checkRelativeJointSpaceJump(state_space::vector_JointSpace &traj,
                                                         double jump_threshold_factor)
    {
        if (traj.size() < MIN_STEPS_FOR_JUMP_THRESH) {
            LOG(WARNING) << "The computed trajectory is too short to detect jumps in joint-space "
                            "Need at least " << MIN_STEPS_FOR_JUMP_THRESH << " steps, only got "
                            <<traj.size()<< " Try a lower max_step.";
        }

        std::vector<double> dist_vector;
        dist_vector.reserve(traj.size() - 1);
        double total_dist = 0.0;
        for (std::size_t i = 1; i < traj.size(); ++i) {
            double dist_prev_point = planner::distance(traj[i], traj[i - 1]);
            dist_vector.push_back(dist_prev_point);
            total_dist += dist_prev_point;
        }

        double percentage = 1.0;
        // compute the average distance between the states we looked at
        double thres = jump_threshold_factor * (total_dist / (double) dist_vector.size());
        for (std::size_t i = 0; i < dist_vector.size(); ++i)
            if (dist_vector[i] > thres) {
                DLOG(INFO) << "Truncating Cartesian path due to detected jump in joint-space distance";
                percentage = (double) (i + 1) / (double) (traj.size());
                traj.resize(i + 1);
                break;
            }

        return percentage;
    }

    double CartesianPlanner::computeCartesianPath(state_space::JointSpace &current_joint_angles,
                                                  const state_space::SE3 &start,
                                                  const state_space::SE3 &target,
                                                  state_space::vector_JointSpace &seg_traj,
                                                  const MaxEEFStep &max_step,
                                                  const JumpThreshold &jump_threshold,
                                                  const std::string &reference_frame,
                                                  const my_collision_detection::MoveItCollisionHelperPtr &moveItCollisionHelper)
    {

        double translation_distance = (start.translationPart() -
                                       target.translationPart()).norm();
        double rotation_distance = planner::distance(start.SO3Part(), target.SO3Part());

        std::size_t translation_steps =
                max_step.translation > 0.0 ? floor(translation_distance / max_step.translation) : 0;

        std::size_t rotation_steps =
                max_step.rotation > 0.0 ? floor(rotation_distance / max_step.rotation) : 0;

        double step = (double) 1 / (double) (std::max(translation_steps, rotation_steps) + 1);

        // If we are testing for relative jumps, we always want at least MIN_STEPS_FOR_JUMP_THRESH steps
        if (jump_threshold.factor > 0.0 && step > ((double) 1 / (double) MIN_STEPS_FOR_JUMP_THRESH))
            step = ((double) 1 / (double) MIN_STEPS_FOR_JUMP_THRESH);

        seg_traj.clear();
        //start
        state_space::JointSpace current_joints;
        if (!moveItCollisionHelper->nearestSolution(current_joints, start, current_joint_angles, false))
            return 0.0;
        seg_traj.emplace_back(current_joints);
        current_joint_angles = current_joints;

        double last_valid_percentage{step}, r_percentage{step};
        while (r_percentage <= 1.0) {

            auto intermediate_state = planner::sphereInterpolate(start, target, r_percentage);
            if (moveItCollisionHelper->nearestSolution(current_joints, intermediate_state, current_joints, true))
                seg_traj.emplace_back(current_joints);
            else
                break;

            last_valid_percentage = r_percentage;
            r_percentage += step;
        }

        last_valid_percentage *= checkJointSpaceJump(seg_traj, jump_threshold);
        return last_valid_percentage;
    }

    double CartesianPlanner::computeCartesianPath(const state_space::JointSpace &current_joint_angles,
                                                  const state_space::vector_SE3 &waypoints,
                                                  state_space::vector_JointSpace &trajectory,
                                                  const MaxEEFStep &max_step,
                                                  const JumpThreshold &jump_threshold,
                                                  const std::string &reference_frame,
                                                  const my_collision_detection::MoveItCollisionHelperPtr &moveItCollisionHelper)
    {
        double percentage_solved = 0.0;

        const auto &kinematics_ptr = moveItCollisionHelper->getKinematicsPtr();
        state_space::SE3 start_pose;
        kinematics_ptr->getLinkTransform(start_pose, reference_frame, current_joint_angles);
        bool need_rotate = reference_frame == kinematics_ptr->getBaseName();

        state_space::JointSpace current_joints = current_joint_angles;

        for (std::size_t i = 1; i < waypoints.size(); ++i) {
            const auto &start = need_rotate ? waypoints[i - 1] : start_pose + waypoints[i - 1];
            const auto &target = need_rotate ? waypoints[i] : start_pose + waypoints[i];
            state_space::vector_JointSpace waypoint_traj;
            double wp_percentage_solved = computeCartesianPath(current_joints, start, target,
                                                               waypoint_traj,
                                                               max_step, jump_threshold,
                                                               reference_frame, moveItCollisionHelper);
            if (fabs(wp_percentage_solved - 1.0) < std::numeric_limits<double>::epsilon()) {
                percentage_solved = (double) (i + 1) / (double) waypoints.size();
                auto start_iter = waypoint_traj.begin();
                if (i > 0 && !waypoint_traj.empty())
                    std::advance(start_iter, 1);
                trajectory.insert(trajectory.end(), start_iter, waypoint_traj.end());
            } else {
                percentage_solved += wp_percentage_solved / (double) waypoints.size();
                auto start_iter = waypoint_traj.begin();
                if (i > 0 && !waypoint_traj.empty())
                    std::advance(start_iter, 1);
                trajectory.insert(trajectory.end(), start_iter, waypoint_traj.end());
                break;
            }
        }
        percentage_solved *= checkJointSpaceJump(trajectory, jump_threshold);
        return percentage_solved;
    }

    void CartesianPlanner::getCartesianLine(state_space::vector_SE3 &waypoints,
                                            const state_space::SE3 &start_point,
                                            const state_space::SE3 &goal_point,
                                            const double cartesian_step)
    {
        waypoints.clear();
        double length = (start_point.translationPart() - goal_point.translationPart()).norm();

        //minimum 0.5 cm;
        double relative_step = cartesian_step < 0.005 ? 0.005 / length : cartesian_step / length;
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
        state_space::SE3 end_point = start_point + state_space::SE3(state_space::SO3::Zero(), direction_vector);
        //minimum 0.5 cm;
        double relative_step = cartesian_step < 0.005 ? 0.005 / length : cartesian_step / length;
        if (relative_step >= 1) {
            waypoints.emplace_back(start_point);
            waypoints.emplace_back(end_point);
        }
        double r_length = 0;
        while (r_length <= 1) {
            waypoints.emplace_back(state_space::SE3(rotation, planner::interpolate(start_point.translationPart(),
                                                                                   end_point.translationPart(),
                                                                                   r_length)));
            r_length += relative_step;
        }
    }

    void CartesianPlanner::getCartesianCircle(state_space::vector_SE3 &waypoints,
                                              const state_space::SE3 &start_point,
                                              const state_space::SE3 &center,
                                              const double radian,
                                              bool const_orientation,
                                              const double step)
    {
        waypoints.clear();
        //change reference frame
        const auto & initial_orientation = start_point.SO3Part();
        auto start = start_point - center;
        auto rotation_axis = state_space::SE3::UnitZ();
        //minimum 0.01 rad
        double relative_step = ((radian > 0.0) - (radian < 0.0)) * std::max(0.01, std::abs(step));
        double r_radian = 0;
        while (fabs(r_radian) < fabs(radian)) {
            auto full_temp = center + (rotation_axis(r_radian) + start);
            const auto & temp = !const_orientation ? full_temp : state_space::SE3(initial_orientation, full_temp.translationPart());
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
        Eigen::Vector3d norm_vector = (end_point - inter_point).cross(inter_point - start_point);
        //when three points are collinear no solution;
        if (norm_vector.norm() < 1e-3){
            LOG(INFO)<<"Given three points are collinear";
            return;
        }

        Eigen::Matrix3d T;
        T.row(0) = norm_vector.transpose();
        T.row(1) = (2 * (inter_point - start_point)).transpose();
        T.row(2) = (2 * (end_point - inter_point)).transpose();

        Eigen::Vector3d P{norm_vector.dot(start_point),
                          inter_point.dot(inter_point) - start_point.dot(start_point),
                          end_point.dot(end_point) - inter_point.dot(inter_point)};

        Eigen::Vector3d center = T.inverse().eval() * P;
        double angle = atan2((end_point - center)[1], (end_point - center)[0]) -
                       atan2((start_point - center)[1], (start_point - center)[0]);

        return getCartesianCircle(waypoints,
                                  state_space::SE3(state_space::SO3::Zero(), start_point),
                                  state_space::SE3(state_space::SO3::Zero(), center),
                                  angle,
                                  true,
                                  step);
    }

}
