#ifndef NURSINGROBOT_CARTESIANPLANNER_H
#define NURSINGROBOT_CARTESIANPLANNER_H
#include "macros/class_forward.h"
#include "StateSpace/SE3.hpp"
#include "StateSpace/JointSpace.hpp"
#include "collision_detection/MoveItCollisionHelper.h"
namespace planner {

    class CartesianPlanner;
    MOVEIT_CLASS_FORWARD(CartesianPlanner)

    class CartesianPlanner {
    private:

    public:
        CartesianPlanner() = default;

        ~CartesianPlanner() = default;


        static double computeCartesianPath(state_space::JointSpace &current_joint_angles,
                                    const state_space::SE3 &start,
                                    const state_space::SE3 &target,
                                    state_space::vector_JointSpace &seg_traj,
                                    const std::string &reference_frame,
                                    const my_collision_detection::MoveItCollisionHelper &moveItCollisionHelper);

        static double computeCartesianPath(const state_space::JointSpace &current_joint_angles,
                                    const state_space::vector_SE3 &wayPoints,
                                    state_space::vector_JointSpace &trajectory,
                                    const std::string & reference_frame,
                                    const my_collision_detection::MoveItCollisionHelper &moveItCollisionHelper
                                    );

        static void getCartesianLine(state_space::vector_SE3 & waypoints,
                                     const state_space::SE3 &start_point,
                                     const state_space::SE3& goal_point,
                                     double cartesian_step =0.01);

        static void getCartesianLine(state_space::vector_SE3 &waypoints,
                                     const state_space::SE3 &start_point,
                                     const Eigen::Vector3d &direction_vector,
                                     double cartesian_step = 0.01);

        /**
         * \brief only consider position
         * @param waypoints
         * @param start_point
         * @param center
         * @param radian
         */
        static void getCartesianCircle(state_space::vector_SE3 &waypoints,
                                       const state_space::SE3 &start_point,
                                       const state_space::SE3 &center,
                                       double radian,
                                       double step);

        static void getCartesianCircle(state_space::vector_SE3 &waypoints,
                                       const Eigen::Vector3d &start_point,
                                       const Eigen::Vector3d &inter_point,
                                       const Eigen::Vector3d &end_point,
                                       double step);
    };
}


#endif //NURSINGROBOT_CARTESIANPLANNER_H
