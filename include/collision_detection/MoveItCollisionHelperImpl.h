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
#include <ros/subscriber.h>
#include <memory>
#include <boost/thread/shared_mutex.hpp>

namespace my_collision_detection {

    MOVEIT_CLASS_FORWARD(JointStatesSubscriber)

    class JointStatesSubscriber {
    public:
        JointStatesSubscriber(ros::NodeHandlePtr &nh, const std::string &topic_name, size_t buff_size)
        {
            subscriber_ = nh->subscribe(topic_name, buff_size, &JointStatesSubscriber::msg_callback, this);
        }

        ~JointStatesSubscriber()
        {
            data_.clear();
        }

        void ParseData(state_space::deque_JointSpace &deque_joint_state_data)
        {
            boost::unique_lock<boost::shared_mutex> write_lock(_data_mutex);
            if (!data_.empty()) {
                deque_joint_state_data.insert(deque_joint_state_data.end(), data_.begin(), data_.end());
                data_.clear();
            }
        }

        state_space::JointSpace getNewestData()
        {
            //only accessible when reading
            boost::shared_lock<boost::shared_mutex> read_lock(_data_mutex);
            return _newest_data;
        }

    private:
        void msg_callback(const sensor_msgs::JointStateConstPtr &joint_state_ptr)
        {
            boost::unique_lock<boost::shared_mutex> write_lock(_data_mutex);
            state_space::JointSpace temp_data(joint_state_ptr->position.data(), joint_state_ptr->position.size());
            data_.emplace_back(temp_data);
            _newest_data = temp_data;
        }

    private:
        ros::Subscriber subscriber_;

        state_space::JointSpace _newest_data;

        boost::shared_mutex _data_mutex;

        state_space::deque_JointSpace data_;
    };

    MOVEIT_CLASS_FORWARD(MoveItCollisionHelperImpl)

    class MoveItCollisionHelperImpl {
    private:
        robot_model_loader::RobotModelLoaderPtr _robot_model_loader_ptr;

        robot_model::RobotModelPtr _robot_model_ptr;

        planning_scene::PlanningScenePtr _planning_scene_ptr;

        my_kinematics::KinematicsPtr _kinematics_ptr;

        const robot_model::JointModelGroup *_joint_model_group;

        JointStatesSubscriberPtr _joint_state_subscribe_ptr;
    public:

        MoveItCollisionHelperImpl(const std::string &group_name,
                                  const std::string &yaml_name,
                                  const my_kinematics::analytical_ik_handled_t &analytical_ik_func,
                                  JointStatesSubscriberPtr joint_state_subscribe_ptr);

        ~MoveItCollisionHelperImpl() = default;

        bool isStateValid(const state_space::JointSpace &state) const;

        bool isStateValid(const state_space::SE3 &state) const;

        template<typename T>
        bool isPathValid(const T &from,
                         const T &to) const
        {
            double step = 0;
            bool result = true;
            while (step <= 1) {
                T temp_state = planner::interpolate(from, to, step);
                result &= isStateValid(temp_state);
                step += 0.25;
            }
            return result;
        }

        const my_kinematics::KinematicsPtr &getKinematicsPtr() const
        {
            return _kinematics_ptr;
        }

        bool allValidSolutions(state_space::vector_JointSpace &final_results,
                               const state_space::SE3 &desired_pose,
                               const state_space::JointSpace *reference_ptr,
                               bool check_collision) const;

        bool nearestSolution(state_space::JointSpace &solution,
                             const state_space::SE3 &desired_pose,
                             const state_space::JointSpace &reference,
                             bool isConsecutive,
                             bool check_collision) const;

        void setJointSubscriber(const JointStatesSubscriberPtr &joint_subscriber)
        {
            _joint_state_subscribe_ptr = joint_subscriber;
        }

        state_space::JointSpace getCurrentJointAngles() const
        {
            if (_joint_state_subscribe_ptr)
                return _joint_state_subscribe_ptr->getNewestData();
            else {
                LOG(ERROR) << "No joint subscriber, please set one";
                return state_space::JointSpace::Zero();
            }
        }

        bool getCurrentLinkTransform(state_space::SE3 &LinkTransform, const std::string &link_name,
                                     const state_space::JointSpace &joint_angles) const
        {
            return _kinematics_ptr->getLinkTransform(LinkTransform, link_name, getCurrentJointAngles());
        }


        bool getEndEffectorPose(state_space::SE3 &ee_transform) const
        {
            return getCurrentLinkTransform(ee_transform, _kinematics_ptr->getEndEffectorName(),
                                           getCurrentJointAngles());
        }
    };


}


#endif
