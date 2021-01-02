//
// Created by xcy on 2020/12/26.
//

#ifndef NURSINGROBOT_CUSTOM_KINEMATICS_HPP
#define NURSINGROBOT_CUSTOM_KINEMATICS_HPP

#include "Kinematics.h"

namespace kinematics {
    /**
 * @brief analytical inverse solutions for aubo i5(aubo series);
 * @param joint_solutions the solutions
 * @param home_configuration the relative pose of the last joint with respect to the first joint
 * @param desired_pose must be converted to pose with respect to the first joint coordinate frame
 * @param references for singular joint, For AUBO/UR, it's the sixth joint
 * @return IK SINGULAR CODE
 */
    IK_SINGULAR_CODE aubo_i5_analytical_IK(Eigen::MatrixXd& joint_solutions, const state_space::SE3& home_configuration,
                                           const state_space::SE_3& desired_pose,
                                           const state_space::JointSpace* references_ptr) {
        const state_space::SE_3& a = desired_pose;
        state_space::R6 upper_bound;
        upper_bound.setConstant(3.05);
        state_space::R6 lower_bound;
        lower_bound.setConstant(-3.05);
        double h1 = 0.1215, d1 = 0.4080, d2 = 0.3760, d = 0.8865, d1p2 = 0.7840, h2 = 0.21550;
        int num_solutions{};
        bool wrist_singular{}, elbow_singular{};

        //step 1 calculate p1
        state_space::SE_3 M1(state_space::SE_3::Zero());
        M1.noalias() += a * home_configuration.SE3Matrix().inverse();

        double p1[2]{a(0, 3) + a(0, 2) * (h1 - h2),
                     a(1, 3) + a(1, 2) * (h1 - h2)};

        //step 2 solving theta1
        //in case of very small negative value;
        double r1 = p1[0] * p1[0] + p1[1] * p1[1] - h1 * h1;
        r1 = _nearZero(r1);
        if (r1 < 0.0) return NO_SOLUTIONS;
        double solutions[8][6]{};
        r1 = sqrt(r1);
        double theta1[2] = {atan2(p1[1] * r1 - h1 * p1[0], p1[1] * h1 + p1[0] * r1),
                            atan2(-p1[1] * r1 - h1 * p1[0], p1[1] * h1 - p1[0] * r1)};

        //step 3 solving theta5 theta6
        double theta5[4];
        for (int i = 0; i < 2; ++i) {
            double r3 = a(1, 2) * cos(theta1[i]) - a(0, 2) * sin(theta1[i]);
            r3 = (SIGN(r3)) * (_nearZero(fabs(r3) - 1) + 1);
            theta5[i * 2 + 0] = acos(r3);
            theta5[i * 2 + 1] = -acos(r3);
        }

        double theta6, sum_theta;
        double theta2[2]{}, theta3[8]{}, theta4[8]{};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                int index = i * 2 + j;
                if (fabs(sin(theta5[index])) < 1e-8) {
                    wrist_singular = true;
                    theta6 = references_ptr ? (*references_ptr)[5] : 0;
                } else {
                    double r4 = a(1, 1) * cos(theta1[i]) - a(0, 1) * sin(theta1[i]);
                    double r2 = a(0, 0) * sin(theta1[i]) - a(1, 0) * cos(theta1[i]);
                    theta6 = atan2(r4 / sin(theta5[index]), r2 / sin(theta5[index]));
                    theta6 = _nearZero(theta6);
                }

                //step 4 solving theta2 theta3 theta4
                state_space::SE_3 e_i1, e_i5, e_i6;
                e_i1 << cos(theta1[i]), sin(theta1[i]), 0, 0,
                        -sin(theta1[i]), cos(theta1[i]), 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
                e_i6 << cos(theta6), 0, -sin(theta6), sin(theta6) * d,
                        0, 1, 0, 0,
                        sin(theta6), 0, cos(theta6), (1 - cos(theta6)) * d,
                        0, 0, 0, 1;
                e_i5 << cos(theta5[index]), sin(theta5[index]), 0, -h1 * sin(theta5[index]),
                        -sin(theta5[index]), cos(theta5[index]), 0, h1 * (1 - cos(theta5[index])),
                        0, 0, 1, 0,
                        0, 0, 0, 1;
                state_space::SE_3 M3{state_space::SE_3::Zero()};
                M3.noalias() += e_i1 * M1 * e_i6 * e_i5;
                double r6 = M3(0, 3) + d1p2 * M3(0, 2);
                double r7 = M3(2, 3) + d1p2 * M3(0, 0);
                sum_theta = atan2(M3(0, 2), M3(0, 0));
                double s1 = (r6 * r6 + r7 * r7 - d1 * d1 - d2 * d2) / (2 * d1 * d2);
                s1 = SIGN(s1) * (_nearZero(fabs(s1) - 1) + 1);
                if (fabs(s1) > 1) {
                    elbow_singular = true;
                    continue;
                }
                elbow_singular = false;
                double alpha = atan2(r6, r7);
                double s2 = (r6 * r6 + r7 * r7 + d1 * d1 - d2 * d2) / (2 * d1 * sqrt(r6 * r6 + r7 * r7));
                s2 = (SIGN(s2)) * (_nearZero(fabs(s2) - 1) + 1);
                theta3[0] = acos(s1);
                theta3[1] = -acos(s1);
                theta2[0] = acos(s2) + alpha;
                theta2[1] = -acos(s2) + alpha;
                theta4[0] = sum_theta - theta2[0] + theta3[0];
                theta4[1] = sum_theta - theta2[1] + theta3[1];

                for (int k = 0; k < 2; ++k) {
                    solutions[num_solutions][0] = theta1[i];
                    solutions[num_solutions][1] = theta2[k];
                    solutions[num_solutions][2] = theta3[k];
                    solutions[num_solutions][3] = theta4[k];
                    solutions[num_solutions][4] = theta5[index];
                    solutions[num_solutions][5] = theta6;

                    //num_solutions++;

                    //if any angle is outside the bound range do not accept this solution
                    state_space::JointSpace check_vector{};
                    memcpy(check_vector.data(), solutions[num_solutions], 6 * sizeof(double));
                    if (check_vector.isValid(upper_bound, lower_bound))
                        num_solutions++;

                }
            }
        }

        if (num_solutions != 0) {
            joint_solutions.resize(6, num_solutions);
            for (int i = 0; i < num_solutions; ++i) {
                memcpy(joint_solutions.col(i).data(), solutions[i], 6 * sizeof(double));
            }
        }
        if (num_solutions == 0)
            return NO_SOLUTIONS;
        if (wrist_singular)
            return WRIST_SINGULAR;
        if (elbow_singular)
            return ELBOW_SINGULAR;

        return SUCCESS;
    }
}

#endif //NURSINGROBOT_CUSTOM_KINEMATICS_HPP
