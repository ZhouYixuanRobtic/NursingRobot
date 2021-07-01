#ifndef NURSINGROBOT_MOVEITCOLLISIONHELPER_H
#define NURSINGROBOT_MOVEITCOLLISIONHELPER_H

#include "MoveItCollisionHelperImpl.h"

namespace my_collision_detection {

    MOVEIT_CLASS_FORWARD(MoveItCollisionHelper)

    class MoveItCollisionHelper {
    private:
        ros::NodeHandlePtr _nh;
        MoveItCollisionHelperImplPtr _moveit_collision_helper_impl_ptr;

    public:

        MoveItCollisionHelper(const std::string &group_name, const std::string &yaml_name,
                              const my_kinematics::analytical_ik_handled_t &analytical_ik_func)
                : _nh(boost::make_shared<ros::NodeHandle>("~")),
                  _moveit_collision_helper_impl_ptr(
                          new MoveItCollisionHelperImpl(group_name, yaml_name, analytical_ik_func,
                                                        std::make_shared<JointStatesSubscriber>(_nh, "/joint_states",
                                                                                                100)))
        {

        }

        ~MoveItCollisionHelper() = default;

        bool isStateValid(const state_space::JointSpace &state) const
        { return _moveit_collision_helper_impl_ptr->isStateValid(state); }

        bool isStateValid(const state_space::SE3 &state) const
        { return _moveit_collision_helper_impl_ptr->isStateValid(state); }

        template<typename T>
        bool isPathValid(const T &from,
                         const T &to) const
        { return _moveit_collision_helper_impl_ptr->isPathValid(from, to); }

        const my_kinematics::KinematicsPtr &getKinematicsPtr() const
        { return _moveit_collision_helper_impl_ptr->getKinematicsPtr(); }

        bool allValidSolutions(state_space::vector_JointSpace &final_results, const state_space::SE3 &desired_pose,
                               const state_space::JointSpace *reference_ptr,
                               bool check_collision = true) const
        {
            return _moveit_collision_helper_impl_ptr->allValidSolutions(final_results, desired_pose, reference_ptr,
                                                                        check_collision);
        }

        bool nearestSolution(state_space::JointSpace &solution,
                             const state_space::SE3 &desired_pose,
                             const state_space::JointSpace &reference,
                             bool isConsecutive = true,
                             bool check_collision = true) const
        {
            return _moveit_collision_helper_impl_ptr->nearestSolution(solution, desired_pose, reference, isConsecutive,
                                                                      check_collision);
        }

        state_space::JointSpace getCurrentJointAngles() const
        { return _moveit_collision_helper_impl_ptr->getCurrentJointAngles(); }

        bool getCurrentLinkTransform(state_space::SE3 &LinkTransform, const std::string &link_name,
                                     const state_space::JointSpace &joint_angles) const
        {
            return _moveit_collision_helper_impl_ptr->getCurrentLinkTransform(LinkTransform, link_name,
                                                                              getCurrentJointAngles());
        }

        state_space::SE3 getEndEffectorPose() const
        {
            state_space::SE3 ee_transform{};
            _moveit_collision_helper_impl_ptr->getEndEffectorPose(ee_transform);
            return ee_transform;
        }
        std::size_t getCCTimes()const{
            return _moveit_collision_helper_impl_ptr->getCCTimes();
        }
        void clearCCTimes(){
            _moveit_collision_helper_impl_ptr->clearCCTimes();
        }
    };
}

#endif //NURSINGROBOT_MOVEITCOLLISIONHELPER_H
