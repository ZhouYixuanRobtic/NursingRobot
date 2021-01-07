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
    };
}


#endif //NURSINGROBOT_CARTESIANPLANNER_H
