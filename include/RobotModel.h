//
// Created by xcy on 2020/12/2.
//

#ifndef NURSINGROBOT_ROBOTMODEL_H
#define NURSINGROBOT_ROBOTMODEL_H

#include "SE3.h"
//TODO: confirm the data type
class RobotModel {
private:
    const std::vector<LieGroup::SE3> all_screw_axes_;
    std::vector<double> current_joint_angles_;
    bool isInBodyFrame_;
    LieGroup::SE_3 home_configuration_;
    virtual void eulerStep()=0;

public:
    RobotModel();
    explicit RobotModel(const std::vector<LieGroup::SE3> & all_screw_axes,bool isInBodyFrame);
    ~RobotModel() = default;
    void loadModel(const std::string & urdf_name);
    void setCurrentJointAngles(std::vector<double> joint_angles);
    inline Eigen::Affine3d fkInSpace(std::vector<double> joint_angles);
    inline Eigen::Affine3d fkInBody(std::vector<double> joint_angles);
    bool nIkInSpace(const Eigen::Affine3d & desired_pose,const std::vector<double> & initial_guess,std::vector<double>& joint_angles, double emog, double ev);
    bool nIkInBody(const Eigen::Affine3d & desired_pose,const std::vector<double> & initial_guess,std::vector<double>& joint_angles, double emog, double ev);
    std::vector<double> bestIkSolution(const Eigen::Affine3d & desired_pose);
    Eigen::MatrixXd  allIkSolutions(const Eigen::Affine3d & desired_pose);

    virtual Eigen::MatrixXd jacobianSpace() = 0;
    virtual Eigen::MatrixXd jacobianBody() = 0;

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
