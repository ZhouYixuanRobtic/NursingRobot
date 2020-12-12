//
// Created by xcy on 2020/12/2.
//

#ifndef NURSINGROBOT_ROBOTMODEL_H
#define NURSINGROBOT_ROBOTMODEL_H

#include "SE3.h"
#include <algorithm>
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
    inline bool nIkInSpace(const LieGroup::SE_3 & desired_pose,Eigen::VectorXd& joint_angles, double eomg, double ev);
    inline bool nIkInBody(const LieGroup::SE_3 & desired_pose,Eigen::VectorXd& joint_angles, double eomg, double ev);

    static double nearZero(const double& val){return (std::abs(val)<1e-8) ? 0.0: val;};
    static double restrict(double val){if(val>M_PI)val -= 2*M_PI;else if(val< -M_PI)val+=2*M_PI;return val;};
    static int SIGN(double val){return (val>0) - (val<0);}
    static double distanceInJointSpace(const LieGroup::R6 &start, const LieGroup::R6& end)
    {
        LieGroup::R6 joint_tangent{end - start};
        for (int j = 0; j < 6; ++j)
        {
            joint_tangent[j] =restrict(joint_tangent[j]);
        }
        return joint_tangent.norm();
    }

public:
    enum IK_SINGULAR_CODE{
        SUCCESS,
        NO_SOLUTIONS,
        ELBOW_SINGULAR,
        WRIST_SINGULAR,
    };
    explicit RobotModel(const std::string & yaml_name);
    explicit RobotModel(const std::vector<LieGroup::SE3> & all_screw_axes,const LieGroup::SE3 &home_configuration,bool isInBodyFrame=false);
    ~RobotModel() = default;
    void loadModel(const std::string & yaml_name);
    void setCurrentJointAngles(const Eigen::VectorXd & joint_angles){current_joint_angles_ = joint_angles;};
    void addEndEffector(const LieGroup::SE3 & ee_configuration){ee_configuration_ = ee_configuration;};
    void addMount(const LieGroup::SE3 & mount_configuration){mount_configuration_ = mount_configuration;};

    //ToDo: move this to planner, and rename it as nearestIkSolutionsList
    inline Eigen::VectorXd nearestIkSolution(const Eigen::Affine3d & desired_pose,const Eigen::VectorXd & reference,bool isConsecutive = false);
    inline Eigen::VectorXd directedNearestIkSolution(const Eigen::Affine3d & desired_pose,const Eigen::VectorXd & reference, const LieGroup::R6 & tangent_reference);

    inline IK_SINGULAR_CODE allIkSolutions(Eigen::MatrixXd &joint_solutions,const LieGroup::SE_3 & desire_pose,double theta6_ref =0.0);
    inline bool allValidIkSolutions(Eigen::MatrixXd & joint_soultions, const LieGroup::SE_3 & desired_pose,const Eigen::VectorXd & reference);

    inline Eigen::MatrixXd jacobianSpace(const Eigen::VectorXd & joint_angles);
    inline Eigen::MatrixXd jacobianBody(const Eigen::VectorXd &joint_angles);

    Eigen::Affine3d fk(const Eigen::VectorXd  &joint_angles)
    { return IN_BODY_ ? Eigen::Affine3d{mount_configuration_.SE3Matrix()*fkInBody(joint_angles)*ee_configuration_.SE3Matrix()} :
                        Eigen::Affine3d{mount_configuration_.SE3Matrix()*fkInSpace(joint_angles)*ee_configuration_.SE3Matrix()};};
    bool nIk(Eigen::Affine3d desired_pose,Eigen::VectorXd& joint_angles, double eomg=1e-7, double ev=5e-7)
    {
        desired_pose.matrix() = (mount_configuration_.SE3Matrix().inverse())*desired_pose.matrix()*(ee_configuration_.SE3Matrix().inverse());
        return IN_BODY_ ? nIkInBody(desired_pose.matrix(),joint_angles,eomg,ev): nIkInSpace(desired_pose.matrix(),joint_angles,eomg,ev);
    };
    bool nIk(LieGroup::SE_3 desired_pose,Eigen::VectorXd& joint_angles, double eomg=1e-7, double ev=5e-7)
    {
        desired_pose.matrix() = (mount_configuration_.SE3Matrix().inverse())*desired_pose.matrix()*(ee_configuration_.SE3Matrix().inverse());
        return IN_BODY_ ? nIkInBody(desired_pose,joint_angles,eomg,ev): nIkInSpace(desired_pose,joint_angles,eomg,ev);
    };
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
