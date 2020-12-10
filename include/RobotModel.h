//
// Created by xcy on 2020/12/2.
//

#ifndef NURSINGROBOT_ROBOTMODEL_H
#define NURSINGROBOT_ROBOTMODEL_H

#include "SE3.h"
#include <yaml-cpp/yaml.h>
class RobotModel {
protected:
    std::vector<LieGroup::SE3> all_screw_axes_{};
    Eigen::VectorXd current_joint_angles_{};
    bool IN_BODY_{};
    LieGroup::SE3 home_configuration_{};
    LieGroup::SE3 mount_configuration_{};
    LieGroup::SE3 ee_configuration_{};
    //virtual void eulerStep()=0;
    inline LieGroup::SE_3 fkInSpace(const Eigen::VectorXd & joint_angles);
    inline LieGroup::SE_3 fkInBody(const Eigen::VectorXd  &joint_angles);
    inline bool nIkInSpace(const Eigen::Affine3d & desired_pose,Eigen::VectorXd& joint_angles, double eomg, double ev);
    inline bool nIkInBody(const Eigen::Affine3d & desired_pose,Eigen::VectorXd& joint_angles, double eomg, double ev);

    inline double nearZero(double val){return (std::abs(val)<1e-8) ? 0: val;};

    static int SIGN(double x){return (x>0)-(x<0);};
    static inline double customAtan2(double y,double x);


public:
    explicit RobotModel(const std::string & yaml_name);
    explicit RobotModel(const std::vector<LieGroup::SE3> & all_screw_axes,const LieGroup::SE3 &home_configuration,bool isInBodyFrame=false);
    ~RobotModel() = default;
    void loadModel(const std::string & yaml_name);
    void setCurrentJointAngles(const Eigen::VectorXd & joint_angles){current_joint_angles_ = joint_angles;};
    void addEndEffector(const LieGroup::SE3 & ee_configuration){ee_configuration_ = ee_configuration;};
    void addMount(const LieGroup::SE3 & mount_configuration){mount_configuration_ = mount_configuration;};

    inline Eigen::VectorXd nearestIkSolution(const Eigen::Affine3d & desired_pose);
    inline int allIkSolutions(Eigen::MatrixXd &joint_solutions,const LieGroup::SE_3 & desired_pose);

    inline Eigen::MatrixXd jacobianSpace(const Eigen::VectorXd & joint_angles);
    inline Eigen::MatrixXd jacobianBody(const Eigen::VectorXd &joint_angles);

    Eigen::Affine3d fk(const Eigen::VectorXd  &joint_angles)
    { return IN_BODY_ ? Eigen::Affine3d{mount_configuration_.SE3Matrix()*fkInBody(joint_angles)*ee_configuration_.SE3Matrix()} :
                        Eigen::Affine3d{mount_configuration_.SE3Matrix()*fkInSpace(joint_angles)*ee_configuration_.SE3Matrix()};};
    bool nIk(Eigen::Affine3d desired_pose,Eigen::VectorXd& joint_angles, double eomg=1e-7, double ev=5e-7)
    {
        desired_pose.matrix() = (mount_configuration_.inverse().SE3Matrix())*desired_pose.matrix()*(ee_configuration_.inverse().SE3Matrix());
        return IN_BODY_ ? nIkInBody(desired_pose,joint_angles,eomg,ev): nIkInSpace(desired_pose,joint_angles,eomg,ev);};
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
