/*Author ZHOU Yixuan*/
#ifndef NURSINGROBOT_KINEMATICS_H
#define NURSINGROBOT_KINEMATICS_H

#include "StateSpace/SE3.hpp"
#include "StateSpace/rn.hpp"
#include "StateSpace/JointSpace.hpp"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <macros/class_forward.h>

namespace kinematics {
    enum IK_SINGULAR_CODE {
        SUCCESS,
        NO_SOLUTIONS,
        ELBOW_SINGULAR,
        WRIST_SINGULAR,
    };

    /**
    * @brief analytical inverse solutions for aubo i5(aubo series);
    * @param joint_solutions the solutions
    * @param home_configuration the relative pose of the last joint with respect to the first joint
    * @param desired_pose must be converted to pose with respect to the first joint coordinate frame
    * @param references_ptr reference for several joints, For AUBO/UR, it's the sixth joint
    * @return IK SINGULAR CODE
    */
    typedef std::function<IK_SINGULAR_CODE(Eigen::MatrixXd&,
                                           const state_space::SE3&,
                                           const state_space::SE_3&,
                                           const state_space::JointSpace* ik_references_ptr)> analytical_ik_handled_t;

    /**@brief in case of zero denominator*/
    static double _nearZero(const double& val) { return (std::abs(val) < 1e-8) ? 0.0 : val; };

    /**@brief judge the sign of val*/
    static int SIGN(double val) { return (val > 0) - (val < 0); };


    MOVEIT_CLASS_FORWARD(Kinematics);

    /**
     * @brief Kinematics class computes fk and ik.
     */
    class Kinematics {
    protected:
        //screw axes
        state_space::vector_SE3 all_screw_axes_{};
        //boolean, true for describing screw axes based on body frame, false for space frame.
        bool IN_BODY_{};
        //home configuration describing the pose of the last joint coordinates frame with respect to the first joint coordinates frame
        state_space::SE3 home_configuration_{};
        //mount configuration describing how the robot was installed
        state_space::SE3 mount_configuration_{};
        //end effector configuration
        state_space::SE3 ee_configuration_{};
        //analytical ik solution function, which could be set outside.
        analytical_ik_handled_t analytical_ik_func_;

        //virtual void eulerStep()=0;

        /**@brief computes forward kinematics with respect to space frame*/
        state_space::SE_3 _fkInSpace(const state_space::JointSpace& joint_angles);

        /**@brief computes forward kinematics with respect to body frame*/
        state_space::SE_3 _fkInBody(const state_space::JointSpace& joint_angles);

        /**
         * @brief computes numerical forward kinematics (Jacobian iterative method) with respect to space frame
         * @param eomg: error of omega (rotation)
         * @param ev: error of velocity (linear)
         */
        bool _nIkInSpace(const state_space::SE_3& desired_pose, state_space::JointSpace& joint_angles, double eomg,
                         double ev);

        /**@brief computes numerical forward kinematics (Jacobian iterative method) with respect to body frame
         * @param eomg: error of omega (rotation)
         * @param ev: error of velocity (linear)
         */
        bool _nIkInBody(const state_space::SE_3& desired_pose, state_space::JointSpace& joint_angles, double eomg,
                        double ev);

        /**@brief load a yaml file describing basic config of a specific robot*/
        void _loadModel(const std::string& yaml_name);

        /**@brief converts pose with respect to world frame into pose with respect to the first joint coordinate frame*/
        state_space::SE_3 _get_pose_in_first(const state_space::SE_3& pose_in_world) {
            return (mount_configuration_.SE3Matrix().inverse()) * pose_in_world *
                   (ee_configuration_.SE3Matrix().inverse());
        }

        /**@brief converts pose with respect to first coordinate frame into pose with respect to the world frame*/
        state_space::SE_3 _get_pose_in_world(const state_space::SE_3& pose_in_first) {
            return (mount_configuration_.SE3Matrix()) * pose_in_first * (ee_configuration_.SE3Matrix());
        }


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit Kinematics(const std::string& yaml_name);

        explicit Kinematics(const state_space::vector_SE3& all_screw_axes, const state_space::SE3& home_configuration,
                            bool isInBodyFrame = false);

        ~Kinematics() = default;

        /**@brief set home configuration*/
        void
        setHomeConfiguration(const state_space::SE3& home_configuration) { home_configuration_ = home_configuration; };

        /**@brief get stored home configuration*/
        state_space::SE3 getHomeConfiguration() const { return home_configuration_; };

        /**@brief set end effector configuration*/
        void setEndEffector(const state_space::SE3& ee_configuration) { ee_configuration_ = ee_configuration; };

        /**@brief get stored end effector configuration*/
        state_space::SE3 getEndEffector() const { return ee_configuration_; };

        /**@brief set mount configuration*/
        void setMount(const state_space::SE3& mount_configuration) { mount_configuration_ = mount_configuration; };

        /**@brief get stored mount configuration*/
        state_space::SE3 getMount() const { return mount_configuration_; };

        /**
         * @brief set an analytical Ik from outside
         * @param ik_func an analytical IK function;
         */
        void setAnalyticalIK(analytical_ik_handled_t ik_func) { analytical_ik_func_ = std::move(ik_func); };

        /**@brief call analytical ik method if exists or return NO_SOLUTIONS*/
        IK_SINGULAR_CODE analyticalIkSolutions(Eigen::MatrixXd& joint_solutions, const state_space::SE_3& desired_pose,
                                               const state_space::JointSpace* ik_references_ptr);

        /**@brief combine numerical method and analytical method
         * @return all valid IK solutions
         * @note only use with a reference
         * */
        bool allValidIkSolutions(Eigen::MatrixXd& joint_solutions, const state_space::SE_3& desired_pose,
                                 const state_space::JointSpace* reference_ptr);

        /**@brief computes jacobian matrix with respect to space frame*/
        Eigen::MatrixXd jacobianSpace(const state_space::JointSpace& joint_angles);

        /**@brief computes jacobian matrix with respect to body frame*/
        Eigen::MatrixXd jacobianBody(const state_space::JointSpace& joint_angles);

        /**@brief computes forward kinematics using the given joint angles*/
        state_space::SE3 fk(const state_space::JointSpace& joint_angles) {
            return IN_BODY_ ? state_space::SE3{_get_pose_in_world(_fkInBody(joint_angles))} :
                   state_space::SE3{_get_pose_in_world(_fkInSpace(joint_angles))};
        }

        /**@brief computes inverse kinematics using numerical method
         * @param joint_angles are references for numerical method
         * @param desired_pose is a SE_3 value
         * */
        bool nIk(const state_space::SE_3& desired_pose, state_space::JointSpace& joint_angles, double eomg = 1e-7,
                 double ev = 5e-7) {
            if (joint_angles.size() == 0)
                throw std::invalid_argument("numerical ik method must have a reference, now it's empty");
            return IN_BODY_ ? _nIkInBody(_get_pose_in_first(desired_pose), joint_angles, eomg, ev) :
                   _nIkInSpace(_get_pose_in_first(desired_pose), joint_angles, eomg, ev);
        };

        /**@brief an overload version of nIK(SE_3)*/
        bool nIk(const Eigen::Isometry3d& desired_pose, state_space::JointSpace& joint_angles, double eomg = 1e-7,
                 double ev = 5e-7) {
            return nIk(desired_pose.matrix(), joint_angles, eomg, ev);
        };

        /**@brief an overload version of nIK(SE_3)*/
        bool nIk(const state_space::SE3& desired_pose, state_space::JointSpace& joint_angles, double eomg = 1e-7,
                 double ev = 5e-7) {
            return nIk(desired_pose.SE3Matrix(), joint_angles, eomg, ev);
        };

        state_space::JointSpace nearestIkSolution(const state_space::SE_3& desired_pose,
                                                  const state_space::JointSpace& reference, bool isConsecutive = false);

        state_space::JointSpace nearestIkSolution(const Eigen::Isometry3d& desired_pose,
                                                  const state_space::JointSpace& reference,
                                                  bool isConsecutive = false) {
            return nearestIkSolution(desired_pose.matrix(), reference, isConsecutive);
        };

        state_space::JointSpace nearestIkSolution(const state_space::SE3& desired_pose,
                                                  const state_space::JointSpace& reference,
                                                  bool isConsecutive = false) {
            return nearestIkSolution(desired_pose.SE3Matrix(), reference, isConsecutive);
        };

        state_space::JointSpace
        directedNearestIkSolution(const state_space::SE_3& desired_pose, const state_space::JointSpace& reference,
                                  const state_space::JointSpace& tangent_reference);

        /*
        virtual state_space::R6 changeReference() =0;
        virtual Eigen::VectorXd inverseDynamics() = 0;
        virtual Eigen::Vector3d gravityForces() = 0 ;
        virtual Eigen::MatrixXd massMatrix() = 0;
        virtual Eigen::VectorXd forwardDynamics() = 0;
        virtual Eigen::MatrixXd inverseDynamicsTrajectory()=0;
        virtual std::vector<Eigen::MatrixXd> forwardDynamicsTrajectory() = 0;
        virtual Eigen::VectorXd velQuadraticForce() = 0;
        virtual Eigen::VectorXd endEffectorForces()=0;
        virtual Eigen::VectorXd computedTorque()=0;
        */

    };
}


#endif //NURSINGROBOT_KINEMATICS_H
