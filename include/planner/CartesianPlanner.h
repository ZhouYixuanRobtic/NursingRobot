#ifndef NURSINGROBOT_CARTESIANPLANNER_H
#define NURSINGROBOT_CARTESIANPLANNER_H

#include "macros/class_forward.h"
#include "StateSpace/SE3.hpp"
#include "StateSpace/JointSpace.hpp"
#include "collision_detection/MoveItCollisionHelper.h"

namespace planner {

    /** \brief Struct for containing jump_threshold.

    For the purposes of maintaining API, we support both \e jump_threshold_factor which provides a scaling factor for
    detecting joint space jumps and \e revolute_jump_threshold and \e prismatic_jump_threshold which provide absolute
    thresholds for detecting joint space jumps. */
    struct JumpThreshold {
        double factor;
        double revolute;   // Radians
        double prismatic;  // Meters

        explicit JumpThreshold() : factor(0.0), revolute(0.0), prismatic(0.0)
        {
        }

        explicit JumpThreshold(double jt_factor) : JumpThreshold()
        {
            factor = jt_factor;
        }

        explicit JumpThreshold(double jt_revolute, double jt_prismatic) : JumpThreshold()
        {
            revolute = jt_revolute;    // Radians
            prismatic = jt_prismatic;  // Meters
        }
        static JumpThreshold DISABLE_JUMP_CHECK()
        {
            return JumpThreshold();
        }
        static JumpThreshold JUMP_FREE()
        {
            return JumpThreshold(2.0);
        }

        static JumpThreshold MIN()
        {
            return JumpThreshold(0.005, 0.0);
        }
    };

    /** \brief Struct for containing max_step for computeCartesianPath
    Setting translation to zero will disable checking for translations and the same goes for rotation */
    struct MaxEEFStep {
        MaxEEFStep(double translation = 0.0, double rotation = 0.0) : translation(translation), rotation(rotation)
        {
        }

        double translation;  // Meters
        double rotation;     // Radians
    };

    class CartesianPlanner {
    private:

    public:
        CartesianPlanner() = default;

        ~CartesianPlanner() = default;

        static double checkJointSpaceJump(state_space::vector_JointSpace &traj,
                                          const JumpThreshold &jump_threshold);

        static double checkAbsoluteJointSpaceJump(state_space::vector_JointSpace &traj,
                                                  double revolute_threshold,
                                                  double prismatic_threshold = 0.0);

        static double checkRelativeJointSpaceJump(state_space::vector_JointSpace &traj,
                                                  double jump_threshold_factor = 0.0);

        static double computeCartesianPath(state_space::JointSpace &current_joint_angles,
                                           const state_space::SE3 &start,
                                           const state_space::SE3 &target,
                                           state_space::vector_JointSpace &seg_traj,
                                           const MaxEEFStep &max_step,
                                           const JumpThreshold &jump_threshold,
                                           const std::string &reference_frame,
                                           const my_collision_detection::MoveItCollisionHelper &moveItCollisionHelper);

        static double computeCartesianPath(const state_space::JointSpace &current_joint_angles,
                                           const state_space::vector_SE3 &wayPoints,
                                           state_space::vector_JointSpace &trajectory,
                                           const MaxEEFStep &max_step,
                                           const JumpThreshold &jump_threshold,
                                           const std::string &reference_frame,
                                           const my_collision_detection::MoveItCollisionHelper &moveItCollisionHelper
        );

        static void getCartesianLine(state_space::vector_SE3 &waypoints,
                                     const state_space::SE3 &start_point,
                                     const state_space::SE3 &goal_point,
                                     double cartesian_step = 0.01);

        static void getCartesianLine(state_space::vector_SE3 &waypoints,
                                     const state_space::SE3 &start_point,
                                     const Eigen::Vector3d &direction_vector,
                                     double cartesian_step = 0.01);

        static void getCartesianCircle(state_space::vector_SE3 &waypoints,
                                       const state_space::SE3 &start_point,
                                       const state_space::SE3 &center,
                                       double radian,
                                       bool const_orientation = true,
                                       double step = 0.01);
        /**
        * \brief only consider position
        * @param waypoints
        * @param start_point
        * @param center
        * @param radian
        */
        static void getCartesianCircle(state_space::vector_SE3 &waypoints,
                                       const Eigen::Vector3d &start_point,
                                       const Eigen::Vector3d &inter_point,
                                       const Eigen::Vector3d &end_point,
                                       double step = 0.01);
    };
}


#endif //NURSINGROBOT_CARTESIANPLANNER_H
