//
// Created by xcy on 2020/12/2.
//

#ifndef NURSINGROBOT_ROBOTMODEL_H
#define NURSINGROBOT_ROBOTMODEL_H

#include "SE3.h"
#include <yaml-cpp/yaml.h>
//TODO: confirm the data type
class RobotModel {
protected:
    std::vector<LieGroup::SE3> all_screw_axes_{};
    Eigen::VectorXd current_joint_angles_{};
    bool IN_BODY_{};
    LieGroup::SE3 home_configuration_{};
    //virtual void eulerStep()=0;
    inline Eigen::Affine3d fkInSpace(const Eigen::VectorXd & joint_angles);
    inline Eigen::Affine3d fkInBody(const Eigen::VectorXd  &joint_angles);
    inline bool nIkInSpace(const Eigen::Affine3d & desired_pose,Eigen::VectorXd& joint_angles, double eomg, double ev);
    inline bool nIkInBody(const Eigen::Affine3d & desired_pose,Eigen::VectorXd& joint_angles, double eomg, double ev);
public:
    explicit RobotModel(const std::string & yaml_name);
    explicit RobotModel(const std::vector<LieGroup::SE3> & all_screw_axes,const LieGroup::SE3 &home_configuration,bool isInBodyFrame=false);
    ~RobotModel() = default;
    void loadModel(const std::string & yaml_name);
    void setCurrentJointAngles(const Eigen::VectorXd & joint_angles){current_joint_angles_ = joint_angles;};


    inline Eigen::VectorXd nearestIkSolution(const Eigen::Affine3d & desired_pose);
    inline Eigen::MatrixXd allIkSolutions(const Eigen::Affine3d & desired_pose);

    inline Eigen::MatrixXd jacobianSpace(const Eigen::VectorXd & joint_angles);
    inline Eigen::MatrixXd jacobianBody(const Eigen::VectorXd &joint_angles);

    Eigen::Affine3d fk(const Eigen::VectorXd  &joint_angles){return IN_BODY_ ? fkInBody(joint_angles) : fkInSpace(joint_angles);};
    bool nIk(const Eigen::Affine3d & desired_pose,Eigen::VectorXd& joint_angles, double eomg=1e-7, double ev=5e-7)
    {return IN_BODY_ ? nIkInBody(desired_pose,joint_angles,eomg,ev): nIkInSpace(desired_pose,joint_angles,eomg,ev);};
    /*
    virtual LieGroup::R6 changeReference() =0;
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
