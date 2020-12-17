//
// Created by xcy on 2020/12/2.
//

#ifndef NURSINGROBOT_ROBOTMODEL_H
#define NURSINGROBOT_ROBOTMODEL_H

#include "StateSpace/SE3.hpp"
#include "StateSpace/rn.hpp"
#include "StateSpace/JointSpace.hpp"
#include <algorithm>
#include <yaml-cpp/yaml.h>

class RobotModel {
protected:

    std::vector<state_space::SE3> all_screw_axes_{};
    state_space::JointSpace current_joint_angles_{};
    bool IN_BODY_{};
    state_space::SE3 home_configuration_{};
    state_space::SE3 mount_configuration_{};
    state_space::SE3 ee_configuration_{};
    //virtual void eulerStep()=0;
    inline state_space::SE_3 fkInSpace(const state_space::JointSpace & joint_angles);
    inline state_space::SE_3 fkInBody(const state_space::JointSpace  &joint_angles);
    inline bool nIkInSpace(const state_space::SE_3 & desired_pose, state_space::JointSpace& joint_angles, double eomg, double ev);
    inline bool nIkInBody(const state_space::SE_3 & desired_pose, state_space::JointSpace& joint_angles, double eomg, double ev);

    static double nearZero(const double& val){return (std::abs(val)<1e-8) ? 0.0: val;};
    static int SIGN(double val){return (val>0) - (val<0);}


public:
    enum IK_SINGULAR_CODE{
        SUCCESS,
        NO_SOLUTIONS,
        ELBOW_SINGULAR,
        WRIST_SINGULAR,
    };
    explicit RobotModel(const std::string & yaml_name);
    explicit RobotModel(const std::vector<state_space::SE3> & all_screw_axes, const state_space::SE3 &home_configuration, bool isInBodyFrame=false);
    ~RobotModel() = default;
    void loadModel(const std::string & yaml_name);
    void setCurrentJointAngles(const state_space::JointSpace & joint_angles){current_joint_angles_ = joint_angles;};
    void addEndEffector(const state_space::SE3 & ee_configuration){ ee_configuration_ = ee_configuration;};
    void addMount(const state_space::SE3 & mount_configuration){ mount_configuration_ = mount_configuration;};

    //ToDo: move this to planner, and rename it as nearestIkSolutionsList
    inline state_space::JointSpace nearestIkSolution(const Eigen::Affine3d & desired_pose,const state_space::JointSpace & reference,bool isConsecutive = false);
    inline state_space::JointSpace directedNearestIkSolution(const Eigen::Affine3d & desired_pose,const state_space::JointSpace & reference, const state_space::JointSpace & tangent_reference);

    inline IK_SINGULAR_CODE allIkSolutions(Eigen::MatrixXd &joint_solutions, const state_space::SE_3 & desire_pose, double theta6_ref =0.0);
    inline bool allValidIkSolutions(Eigen::MatrixXd & joint_soultions, const state_space::SE_3 & desired_pose, const state_space::JointSpace & reference);

    inline Eigen::MatrixXd jacobianSpace(const state_space::JointSpace & joint_angles);
    inline Eigen::MatrixXd jacobianBody(const state_space::JointSpace &joint_angles);

    Eigen::Affine3d fk(const state_space::JointSpace  &joint_angles)
    { return IN_BODY_ ? Eigen::Affine3d{mount_configuration_.SE3Matrix()*fkInBody(joint_angles)*ee_configuration_.SE3Matrix()} :
                        Eigen::Affine3d{mount_configuration_.SE3Matrix()*fkInSpace(joint_angles)*ee_configuration_.SE3Matrix()};};
    bool nIk(Eigen::Affine3d desired_pose,state_space::JointSpace& joint_angles, double eomg=1e-7, double ev=5e-7)
    {
        desired_pose.matrix() = (mount_configuration_.SE3Matrix().inverse())*desired_pose.matrix()*(ee_configuration_.SE3Matrix().inverse());
        return IN_BODY_ ? nIkInBody(desired_pose.matrix(),joint_angles,eomg,ev): nIkInSpace(desired_pose.matrix(),joint_angles,eomg,ev);
    };
    bool nIk(state_space::SE_3 desired_pose, state_space::JointSpace& joint_angles, double eomg=1e-7, double ev=5e-7)
    {
        desired_pose.matrix() = (mount_configuration_.SE3Matrix().inverse())*desired_pose.matrix()*(ee_configuration_.SE3Matrix().inverse());
        return IN_BODY_ ? nIkInBody(desired_pose,joint_angles,eomg,ev): nIkInSpace(desired_pose,joint_angles,eomg,ev);
    };
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


#endif //NURSINGROBOT_ROBOTMODEL_H
