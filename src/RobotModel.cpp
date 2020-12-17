//
// Created by xcy on 2020/12/2.
//

#include <iostream>
#include "RobotModel.h"
RobotModel::RobotModel(const std::string &yaml_name)
{
    ee_configuration_ = state_space::SE3();
    mount_configuration_ = state_space::SE3();
    loadModel(yaml_name);
}
RobotModel::RobotModel(const std::vector<state_space::SE3> &all_screw_axes, const state_space::SE3 &home_configuration, bool isInBodyFrame)
{
    all_screw_axes_ = all_screw_axes;
    home_configuration_ = home_configuration;
    IN_BODY_ = isInBodyFrame;
    ee_configuration_ = state_space::SE3();
    mount_configuration_ = state_space::SE3();
    current_joint_angles_ = state_space::JointSpace(all_screw_axes.size());
}
void RobotModel::loadModel(const std::string &yaml_name)
{
    YAML::Node doc = YAML::LoadFile(yaml_name);
    try
    {
        std::vector<double> pose_with_quaternion = doc["aubo_i5"]["home_configuration"].as<std::vector<double>>();
        Eigen::Matrix<double,7,1> home_pose;
        memcpy(home_pose.data(),pose_with_quaternion.data(), pose_with_quaternion.size()*sizeof(double));
        home_configuration_ = state_space::SE3(home_pose);
        IN_BODY_ = doc["aubo_i5"]["isInBodyFrame"].as<bool>();
        std::map<std::string,std::vector<double>> axes_map;
        axes_map = doc["aubo_i5"]["screw_axes"].as<std::map<std::string,std::vector<double>>>();

        for(const auto & it : axes_map)
        {
            if(it.second.size()!=6)
            {
                char buf[100];
                sprintf(buf,"wrong twist, should have 6 elements but have %d elements",(int)it.second.size());
                perror(buf);
            }
            else
            {
                state_space::R6 temp_twist;
                memcpy(temp_twist.data(),it.second.data(), temp_twist.size()*sizeof(double));
                all_screw_axes_.emplace_back(state_space::SE3(temp_twist));
            }
        }
    }
    catch (YAML::InvalidScalar)
    {
       perror("tagParam.yaml is invalid.");
    }
    current_joint_angles_ = state_space::JointSpace(all_screw_axes_.size());
}
inline state_space::SE_3 RobotModel::fkInSpace(const state_space::JointSpace &joint_angles)
{
    state_space::SE_3 ee_pose{state_space::SE_3::Identity()};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    ee_pose = ee_pose * home_configuration_.SE3Matrix();
    return ee_pose;
}
inline state_space::SE_3 RobotModel::fkInBody(const state_space::JointSpace &joint_angles)
{
    state_space::SE_3 ee_pose{home_configuration_.SE3Matrix()};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    return ee_pose;
}
inline Eigen::MatrixXd RobotModel::jacobianSpace(const state_space::JointSpace & joint_angles)
{
    Eigen::MatrixXd jacobian_matrix(6,6);
    jacobian_matrix.col(0) = all_screw_axes_[0].Axis();
    state_space::SE_3  tmp_matrix = state_space::SE_3::Identity();
    for (int i = 1; i < joint_angles.size(); i++)
    {
        tmp_matrix = tmp_matrix * all_screw_axes_[i-1](joint_angles[i-1]).SE3Matrix();
        jacobian_matrix.col(i) = state_space::GetAdjoint(tmp_matrix) * all_screw_axes_[i].Axis();
    }
    return jacobian_matrix;
}
inline Eigen::MatrixXd RobotModel::jacobianBody(const state_space::JointSpace & joint_angles)
{
    Eigen::MatrixXd jacobian_matrix(6,6);
    state_space::SE_3  tmp_matrix = state_space::SE_3::Identity();
    jacobian_matrix.col(joint_angles.size()-1) = all_screw_axes_[joint_angles.size()-1].Axis();
    for (int i = joint_angles.size() - 2; i >= 0; i--)
    {
        tmp_matrix = tmp_matrix * all_screw_axes_[i+1](-joint_angles[i+1]).SE3Matrix();
        jacobian_matrix.col(i) = state_space::GetAdjoint(tmp_matrix) * all_screw_axes_[i].Axis();
    }
    return jacobian_matrix;
}
inline bool RobotModel::nIkInSpace(const state_space::SE_3 &desired_pose, state_space::JointSpace &joint_angles, double eomg, double ev)
{
    /*
     * jacobian iterative method
     */
    int i = 0;
    int max_iterations = 50;
    state_space::SE_3 Tfk = fkInSpace(joint_angles);
    state_space::SE_3 Tdiff = Tfk.inverse() * desired_pose.matrix();
    state_space::R6 Vs = state_space::GetAdjoint(Tfk) * state_space::SE3(Tdiff).Vector();
    bool err = (Vs.block<3,1>(0,0).norm() > eomg || Vs.block<3,1>(3,0).norm() > ev);
    Eigen::MatrixXd Js;
    while (err && i < max_iterations)
    {
        Js = jacobianSpace( joint_angles);
        joint_angles += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
        ++i;
        // iterate
        Tfk = fkInSpace(joint_angles);
        Tdiff = Tfk.inverse() * desired_pose.matrix();
        Vs = state_space::GetAdjoint(Tfk) * state_space::SE3(Tdiff).Vector();
        err = (Vs.block<3,1>(0,0).norm() > eomg || Vs.block<3,1>(3,0).norm() > ev);
    }
    for(int j =0; j<joint_angles.size(); ++j)
    {
        joint_angles[j] = nearZero(joint_angles[j]);
    }
    return !err;
}
inline bool RobotModel::nIkInBody(const state_space::SE_3 &desired_pose, state_space::JointSpace &joint_angles, double eomg, double ev)
{
    int i = 0;
    int max_iterations = 50;
    state_space::SE_3 Tfk = fkInBody(joint_angles).matrix();
    state_space::SE_3 Tdiff = Tfk.inverse() * desired_pose.matrix();
    state_space::R6 Vb = state_space::SE3(Tdiff).Vector();
    bool err = (Vb.block<3,1>(0,0).norm() > eomg || Vb.block<3,1>(3,0).norm() > ev);
    Eigen::MatrixXd Jb;
    while (err && i < max_iterations)
    {
        Jb = jacobianBody( joint_angles);
        joint_angles += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
        ++i;
        // iterate
        Tfk = fkInBody(joint_angles).matrix();
        Tdiff = Tfk.inverse() * desired_pose.matrix();
        Vb = state_space::SE3(Tdiff).Vector();
        err = (Vb.block<3,1>(0,0).norm() > eomg || Vb.block<3,1>(3,0).norm() > ev);
    }
    for(int j =0; j<joint_angles.size(); ++j)
    {
        joint_angles[j] = nearZero(joint_angles[j]);
    }
    return !err;
}
inline RobotModel::IK_SINGULAR_CODE RobotModel::allIkSolutions(Eigen::MatrixXd &joint_solutions, const state_space::SE_3 &desired_motion, double theta6_ref)
{
    state_space::R6 upper_bound; upper_bound.setConstant(3.05);
    state_space::R6 lower_bound; lower_bound.setConstant(-3.05);
    state_space::SE_3 a{mount_configuration_.SE3Matrix().inverse() * desired_motion * ee_configuration_.SE3Matrix().inverse()};
    double h1=0.1215,d1=0.4080,d2=0.3760,d= 0.8865,d1p2 =0.7840,h2=0.21550;
    int num_solutions{};
    bool wrist_singular{},elbow_singular{};

    //step 1 calculate p1
    state_space::SE_3 M1{a * home_configuration_.SE3Matrix().inverse()};
    double p1[2]{a(0, 3) + a(0, 2) * (h1 - h2),
                       a(1, 3) + a(1, 2) * (h1 - h2)};

    //step 2 solving theta1
    //in case of very small negative value;
    double r1 = p1[0]*p1[0]+p1[1]*p1[1]-h1*h1;
    r1 = nearZero(r1);
    if(r1<0.0) return NO_SOLUTIONS;
    double solutions[8][6];
    r1 = sqrt(r1);
    double theta1[2] = {atan2(p1[1]*r1-h1*p1[0],p1[1]*h1+p1[0]*r1),
                        atan2(-p1[1]*r1-h1*p1[0],p1[1]*h1-p1[0]*r1)};

    //step 3 solving theta5 theta6
    double theta5[4];
    for(int i=0; i<2; ++i)
    {
        double r3 = a(1, 2) * cos(theta1[i]) - a(0, 2) * sin(theta1[i]);
        r3 = (SIGN(r3))*(nearZero(fabs(r3)-1)+1);
        theta5[i*2+0] =acos(r3);theta5[i*2+1]=-acos(r3);
    }

    double theta6, sum_theta;
    double theta2[2]{},theta3[8]{},theta4[8]{};
    for(int i=0; i<2; ++i)
    {
        for(int j=0; j<2; ++j)
        {
            int index = i*2+j;
            if(fabs(sin(theta5[index])) < 1e-8)
            {
                wrist_singular = true;
                theta6 = theta6_ref;
            }
            else
            {
                double r4 = a(1, 1) * cos(theta1[i]) - a(0, 1) * sin(theta1[i]);
                double r2 = a(0, 0) * sin(theta1[i]) - a(1, 0) * cos(theta1[i]);
                theta6 = atan2(r4/sin(theta5[index]),r2/sin(theta5[index]));
                theta6 = nearZero(theta6);
            }

            //step 4 solving theta2 theta3 theta4
            state_space::SE_3 e_i1,e_i5,e_i6;
            e_i1<<cos(theta1[i]),sin(theta1[i]),0,0,
                    -sin(theta1[i]),cos(theta1[i]),0,0,
                    0,0,1,0,
                    0,0,0,1;
            e_i6<<cos(theta6),0,-sin(theta6),sin(theta6)*d,
                    0,1,0,0,
                    sin(theta6),0,cos(theta6),(1-cos(theta6))*d,
                    0,0,0,1;
            e_i5<<cos(theta5[index]),sin(theta5[index]),0,-h1*sin(theta5[index]),
                    -sin(theta5[index]),cos(theta5[index]),0,h1*(1-cos(theta5[index])),
                    0,0,1,0,
                    0,0,0,1;
            state_space::SE_3 M3{e_i1 * M1 * e_i6 * e_i5};
            double r6 = M3(0,3)+d1p2*M3(0,2);
            double r7 = M3(2,3)+d1p2*M3(0,0);
            sum_theta = atan2(M3(0,2),M3(0,0));
            double s1 = (r6*r6+ r7*r7-d1*d1-d2*d2)/(2*d1*d2);
            s1 =SIGN(s1)*(nearZero(fabs(s1)-1)+1);
            if(fabs(s1)>1)
            {
                elbow_singular = true;
                continue;
            }
            elbow_singular =false;
            double alpha =atan2(r6,r7);
            double s2 = (r6*r6+r7*r7+d1*d1-d2*d2)/(2*d1*sqrt(r6*r6+r7*r7));
            s2 = (SIGN(s2))*(nearZero(fabs(s2)-1)+1);
            theta3[0] = acos(s1);
            theta3[1] = -acos(s1);
            theta2[0] = acos(s2)+ alpha;
            theta2[1] = -acos(s2)+ alpha;
            theta4[0] = sum_theta-theta2[0]+theta3[0];
            theta4[1] = sum_theta-theta2[1]+theta3[1];

            for(int k =0; k<2;++k)
            {
                solutions[num_solutions][0] = theta1[i];
                solutions[num_solutions][1] = theta2[k];
                solutions[num_solutions][2] = theta3[k];
                solutions[num_solutions][3] = theta4[k];
                solutions[num_solutions][4] = theta5[index];
                solutions[num_solutions][5] = theta6;

                //num_solutions++;

                //if any angle is outside the bound range do not accept this solution
                state_space::JointSpace check_vector{};
                memcpy(check_vector.data(),solutions[num_solutions],6*sizeof(double));
                if(check_vector.isValid(upper_bound,lower_bound))
                    num_solutions++;

            }
        }
    }

    if(num_solutions != 0)
    {
        joint_solutions.resize(6,num_solutions);
        int index = 0;
        for(int i=0;i<num_solutions;++i)
        {
            memcpy(joint_solutions.col(i).data(),solutions[i],6*sizeof(double));
        }
    }
    if(num_solutions == 0)
        return NO_SOLUTIONS;
    if(wrist_singular)
        return WRIST_SINGULAR;
    if(elbow_singular)
        return ELBOW_SINGULAR;

    return SUCCESS;
}
inline bool RobotModel::allValidIkSolutions(Eigen::MatrixXd &joint_solutions, const state_space::SE_3 &desired_pose, const state_space::JointSpace &reference)
{
    //only use in consecutive mode
    state_space::JointSpace solution{reference};
    RobotModel::IK_SINGULAR_CODE ret_code = allIkSolutions(joint_solutions,desired_pose);


    if(ret_code == NO_SOLUTIONS)
    {
        if(nIk(desired_pose,solution))
        {
            joint_solutions= solution.Vector();
        }
        else
            return false;
    }
    else if(ret_code == WRIST_SINGULAR)
    {
        if(nIk(desired_pose,solution))
        {
            ret_code = allIkSolutions(joint_solutions,desired_pose,solution[5]);
        }
        else
            return false;
    }
    return true;

}
inline state_space::JointSpace RobotModel::nearestIkSolution(const Eigen::Affine3d &desired_pose, const state_space::JointSpace & reference, bool isConsecutive)
{
    //return the nearest ik solution
    state_space::JointSpace solution{reference};
    Eigen::MatrixXd joint_solutions;
    RobotModel::IK_SINGULAR_CODE ret_code = allIkSolutions(joint_solutions,desired_pose.matrix());
    //TODO:: delete sel-collision, collision, solutions;
    if(!isConsecutive && joint_solutions.cols() == 0 ) return reference;
    else if(joint_solutions.cols() == 0 ||isConsecutive && ret_code == WRIST_SINGULAR)
    {
        if(nIk(desired_pose,solution,1e-8,1e-8))
            return solution;
        else
            return reference;
    }
    else
    {
        std::vector<double> norm_box;
        for (int i = 0; i < joint_solutions.cols(); ++i)
        {
            norm_box.push_back(reference.distance(joint_solutions.col(i)));
        }
        return state_space::JointSpace(joint_solutions.col(std::distance(norm_box.begin(), std::min_element(norm_box.begin(), norm_box.end()))));
    }
}
inline state_space::JointSpace RobotModel::directedNearestIkSolution(const Eigen::Affine3d &desired_pose,const state_space::JointSpace &reference,const state_space::JointSpace &tangent_reference)
{
    state_space::JointSpace solution{reference};
    Eigen::MatrixXd joint_solutions;
    RobotModel::IK_SINGULAR_CODE ret_code = allIkSolutions(joint_solutions,desired_pose.matrix());
    if(joint_solutions.cols() == 0 || ret_code == WRIST_SINGULAR)
    {
        if(nIk(desired_pose,solution,1e-8,1e-8))
            return solution;
        else
            return reference;
    }
    else
    {
        std::vector<double> norm_box;
        for (int i = 0; i < joint_solutions.cols(); ++i)
        {
            state_space::JointSpace joint_tangent{joint_solutions.col(i) - reference.Vector()};

            norm_box.push_back(joint_tangent.distance(tangent_reference));
        }
        return state_space::JointSpace(joint_solutions.col(std::distance(norm_box.begin(), std::min_element(norm_box.begin(), norm_box.end()))));
    }
}