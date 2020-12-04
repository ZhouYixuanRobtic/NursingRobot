//
// Created by xcy on 2020/12/2.
//

#ifndef NURSINGROBOT_ROBOTMODEL_H
#define NURSINGROBOT_ROBOTMODEL_H

#include "SE3.h"
//TODO: confirm the data type
class RobotModel {
private:
    std::vector<LieGroup::SE3> all_screw_axes_;
    Eigen::VectorXd current_joint_angles_{};
    const bool IN_BODY_;
    LieGroup::SE_3 home_configuration_{};
    virtual void eulerStep()=0;

public:
    explicit RobotModel(const std::string & urdf_name,bool isInBodyFrame = false);
    explicit RobotModel(const std::vector<LieGroup::SE3> & all_screw_axes,const LieGroup::SE_3 &home_configuration,bool isInBodyFrame=false);
    ~RobotModel() = default;
    //TODO: implement a URDF reader
    virtual void loadModel(const std::string & urdf_name) =0;
    void setCurrentJointAngles(const Eigen::VectorXd & joint_angles){current_joint_angles_ = joint_angles;};
    inline Eigen::Affine3d fkInSpace(const Eigen::VectorXd & joint_angles);
    inline Eigen::Affine3d fkInBody(const Eigen::VectorXd  &joint_angles);
    inline bool nIkInSpace(const Eigen::Affine3d & desired_pose,Eigen::VectorXd& joint_angles, double eomg=0.01, double ev=0.001);
    inline bool nIkInBody(const Eigen::Affine3d & desired_pose,Eigen::VectorXd& joint_angles, double eomg=0.01, double ev=0.001);
    inline Eigen::VectorXd nearestIkSolution(const Eigen::Affine3d & desired_pose);
    inline Eigen::MatrixXd allIkSolutions(const Eigen::Affine3d & desired_pose);

    inline Eigen::MatrixXd jacobianSpace(const Eigen::VectorXd & joint_angles);
    inline Eigen::MatrixXd jacobianBody(const Eigen::VectorXd &joint_angles);

    virtual Eigen::VectorXd inverseDynamics() = 0;
    virtual Eigen::Vector3d gravityForces() = 0 ;
    virtual Eigen::MatrixXd massMatrix() = 0;
    virtual Eigen::VectorXd forwardDynamics() = 0;
    virtual Eigen::MatrixXd inverseDynamicsTrajectory()=0;
    virtual std::vector<Eigen::MatrixXd> forwardDynamicsTrajectory() = 0;
    virtual Eigen::VectorXd velQuadraticForce() = 0;
    virtual Eigen::VectorXd endEffectorForces()=0;
    virtual Eigen::VectorXd computedTorque()=0;


};


#endif //NURSINGROBOT_ROBOTMODEL_H
