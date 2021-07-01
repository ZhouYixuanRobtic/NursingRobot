#define EIGEN_MKL_USE_ALL

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "util/logger.hpp"
#include "Kinematics/custom_kinematics.hpp"
#include "Kinematics/Kinematics.h"
#include "planner/Planner.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <fstream>

TEST(KinematicsTest,POEFKTest)
{
    state_space::SE_3 ground_truth;
    ground_truth<<0,0,1,0.2155,1,0,0,0,0,1,0,0.8865,0,0,0,1;

    const state_space::JointSpace test_joints{std::vector<double>{M_PI_2,0,0,0,0,0}};

    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);


    auto fk_pose = aubo_i5_kinematics.fk(test_joints);
    std::cout<<sqrt((ground_truth.array()-fk_pose.SE3Matrix().array()).cwiseAbs().square().sum())<<std::endl;


}

TEST(KinematicsTest, SingleIKTest){
    const state_space::JointSpace random_joints{std::vector<double>{0,-0.64,-1.49,-1.25,0.0,0.40}};
    const state_space::JointSpace refer_joint{std::vector<double>{0,-0.64,-1.49,-1.25,-0.1,0.40}};
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);
    auto desired_pose = aubo_i5_kinematics.fk(random_joints);
    Eigen::MatrixXd joint_solutions;
    bool found_ik =  aubo_i5_kinematics.allValidIkSolutions(joint_solutions, desired_pose.SE3Matrix(), &refer_joint);

    double max_error=std::numeric_limits<double>::min();
    double min_error = std::numeric_limits<double>::max();
    if(found_ik){
        for(int i=0;i<joint_solutions.cols();++i){
            auto one_sol = state_space::JointSpace(joint_solutions.col(i));
            std::cout<<one_sol<<std::endl;
            auto fk_pose = aubo_i5_kinematics.fk(one_sol);
            double error = sqrt((desired_pose.SE3Matrix().array()-fk_pose.SE3Matrix().array()).cwiseAbs().square().sum());
            max_error = error>max_error? error : max_error;
            error = planner::distance(random_joints,one_sol);
            min_error = error<min_error?error:min_error;
        }
        std::cout<<"max_error: "<<max_error<<std::endl;
        std::cout<<"min_error: "<<min_error<<std::endl;
    }
}

TEST(KinematicsTest, StatisticTest)
{
    //output files
    const std::string base_path{"/home/xcy/KINE_SAT/"};
    const std::vector<std::string> txt_names{{"/time.txt"},{"/error.txt"},{"/invalidPose.txt"},{"/performance.txt"}};

    std::vector<std::ofstream> outfile_vector{};
    outfile_vector.resize(txt_names.size());
    for(int i=0;i<txt_names.size();++i){
        outfile_vector[i].open(base_path+txt_names[i],std::ios::out | std::ios::trunc);
    }
    //test body
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);
    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    std::size_t found_ik_counter{};
    const size_t max_iterations = 1E8;
    const double zero_thresh = 1e-3;
    double  all_min_error{},all_max_error{};
    std::size_t all_time{};

    for(size_t index=0; index<max_iterations;++index){
        //random joints
        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        auto direction_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        auto reference_joint = planner::extend(test_joint,direction_joint,0.01);
        auto desired_pose = aubo_i5_kinematics.fk(test_joint);
        Eigen::MatrixXd joint_solutions;
        clock_t start = clock();
        bool found_ik =  aubo_i5_kinematics.allValidIkSolutions(joint_solutions, desired_pose.SE3Matrix(), &reference_joint);
        clock_t end = clock();

        if (found_ik){
            //find the min error
            double max_error=std::numeric_limits<double>::min();
            double min_error = std::numeric_limits<double>::max();
            for(int i=0;i<joint_solutions.cols();++i){
                auto one_sol = state_space::JointSpace(joint_solutions.col(i));
                auto fk_pose = aubo_i5_kinematics.fk(one_sol);
                double error = sqrt((desired_pose.SE3Matrix().array()-fk_pose.SE3Matrix().array()).cwiseAbs().square().sum());
                max_error = error>max_error? error : max_error;
                error = planner::distance(test_joint,one_sol);
                min_error = error<min_error?error:min_error;
            }
            if(max_error<zero_thresh){
                found_ik_counter++;
                //output time unit us
                outfile_vector[0]<< 1E6*static_cast<double>((end-start))/CLOCKS_PER_SEC<<std::endl;
                //output error
                outfile_vector[1]<<max_error<<" "<<min_error<<std::endl;
                //accumulate
                all_time+=end-start;
                all_min_error+=min_error;
                all_max_error+=max_error;
            }else
            //output invalid joint
                outfile_vector[2]<<test_joint<<std::endl;
        }
    }
    //output overall performance
    double average_time = 1E6*static_cast<double>(all_time)/max_iterations/CLOCKS_PER_SEC;
    outfile_vector[3]<<"average time: "<<average_time<<" us"<<std::endl;
    double average_pose_error = all_max_error/max_iterations;
    outfile_vector[3]<<"average pose error: "<<average_pose_error<<std::endl;
    double average_joint_error = all_min_error/max_iterations;
    outfile_vector[3]<<"average joint error: "<<average_joint_error<<std::endl;
    double coverage_rate = static_cast<double>(found_ik_counter)/max_iterations;
    outfile_vector[3]<<"coverage rate: "<<coverage_rate<<std::endl;

    //close all output file
    for(auto &item:outfile_vector){
        if(item.is_open())
            item.close();
    }
}

TEST(KinematicsTest, AuboAnalyticalTest)
{
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);


    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t ik_analytical_only_counter{}, ik_analytical_with_right_reference{}, ik_analytical_exactly_the_same_counter{};
    const size_t max_iterations = 1E8;
    const double zero_thresh = 1e-3;
    double all_time = 0;

    for (size_t index = 0; index < max_iterations; ++index) {

        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        auto desired_pose = aubo_i5_kinematics.fk(test_joint);
        //check analytical ik without right reference (make \theta6 as 0)
        Eigen::MatrixXd joint_solutions;
        clock_t start = clock();
        auto ret_code = aubo_i5_kinematics.analyticalIkSolutions(joint_solutions, desired_pose.SE3Matrix(),
                                                                 nullptr);
        clock_t end = clock();
        if (joint_solutions.cols() != 0 &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(state_space::JointSpace(joint_solutions.col(0))))
            < zero_thresh)
            ik_analytical_only_counter++;
        //check analytical ik with right reference
        joint_solutions.resize(0, 0);
        auto refer_joint = test_joint;
        ret_code = aubo_i5_kinematics.analyticalIkSolutions(joint_solutions, desired_pose.SE3Matrix(),
                                                            &refer_joint);
        if (joint_solutions.cols() != 0 &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(state_space::JointSpace(joint_solutions.col(0))))
            < zero_thresh) {
            ik_analytical_with_right_reference++;
            for (size_t i = 0; i < joint_solutions.cols(); ++i) {
                if (planner::distance(test_joint, state_space::JointSpace(joint_solutions.col(i))) < zero_thresh) {
                    ik_analytical_exactly_the_same_counter++;
                    break;
                }
            }
        }

        all_time += double(end - start) / CLOCKS_PER_SEC;
    }
    EXPECT_GE(ik_analytical_with_right_reference, 0.9999973 * max_iterations)
                        << "Coverage rate of analytical IK solution (equals to fk) doesn't reach 5 sigma level, but"
                        << ik_analytical_with_right_reference << "/" << max_iterations;
    LOG(INFO)<<"Coverage rate of analytical IK solution"<< ik_analytical_with_right_reference << "/" << max_iterations<<(double )ik_analytical_with_right_reference/max_iterations<<std::endl;
    EXPECT_NEAR(ik_analytical_only_counter, 0.9999973 * max_iterations,3)
                        << "Coverage rate of analytical IK solution with zero reference doesn't reach 5 sigma level, but"
                        << ik_analytical_only_counter << "/" << max_iterations;
    LOG(INFO)<<"Coverage rate of analytical IK solution with zero reference"<< ik_analytical_only_counter << "/" << max_iterations<<(double )ik_analytical_only_counter/max_iterations<<std::endl;
    EXPECT_GE(ik_analytical_exactly_the_same_counter, 0.9973 * max_iterations)
                        << "Coverage rate of exactly the same analytical IK solution with zero reference doesn't reach 3 sigma level, but"
                        << ik_analytical_exactly_the_same_counter << "/" << max_iterations;
    LOG(INFO)<<"Coverage rate of exactly the same analytical IK solution with zero reference"<< ik_analytical_exactly_the_same_counter << "/" << max_iterations<<(double )ik_analytical_exactly_the_same_counter/max_iterations<<std::endl;
    LOG(INFO) << "the average time consumption of analytical ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
}

TEST(KinematicsTest, NumericalTest)
{
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml");

    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t ik_numerical_only_counter{}, ik_numerical_with_right_reference{}, ik_numerical_exactly_the_same_counter{};
    const size_t max_iterations = 1e6;
    const double zero_thresh = 1e-3;
    double all_time = 0;

    for (size_t index = 0; index < max_iterations; ++index) {

        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);

        auto desired_pose = aubo_i5_kinematics.fk(test_joint);

        //check analytical ik without right reference (make \theta6 as 0)
        state_space::JointSpace raw_solution;
        clock_t start = clock();
        bool found_ik = aubo_i5_kinematics.nIk(desired_pose, raw_solution);
        clock_t end = clock();

        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(raw_solution))
            < zero_thresh)
            ik_numerical_only_counter++;

        //check analytical ik with right reference
        auto direction_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        raw_solution = planner::extend(test_joint,direction_joint,0.01);
        start = clock();
        found_ik = aubo_i5_kinematics.nIk(desired_pose, raw_solution);
        end = clock();
        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(raw_solution))
            < zero_thresh) {
            ik_numerical_with_right_reference++;
            if (planner::distance(test_joint, raw_solution) < zero_thresh)
                ik_numerical_exactly_the_same_counter++;
        }
        all_time += double(end - start) / CLOCKS_PER_SEC;
    }
    EXPECT_EQ(max_iterations, ik_numerical_with_right_reference)
                        << "Coverage rate of numerical IK solution (equals to fk) with right reference is not 100%, but"
                        << ik_numerical_with_right_reference << "/" << max_iterations;
    LOG(INFO)<<"Coverage rate of numerical IK solution (equals to fk) with right reference"<< ik_numerical_with_right_reference << "/" << max_iterations<<(double )ik_numerical_with_right_reference/max_iterations<<std::endl;
    //EXPECT_GE(ik_numerical_only_counter, 0.9973*max_iterations)<<"Coverage rate of numerical IK solution with zero reference doesn't reach 3 sigma level, but"
    // << ik_numerical_only_counter <<"/"<<max_iterations;

    EXPECT_GE(ik_numerical_exactly_the_same_counter, 0.99 * max_iterations)
                        << "Coverage rate of numerical IK solution with zero reference doesn't reach 99 level, but"
                        << ik_numerical_exactly_the_same_counter << "/" << max_iterations;
    LOG(INFO)<<"Coverage rate of numerical IK solution with zero reference"<< ik_numerical_exactly_the_same_counter << "/" << max_iterations<<(double )ik_numerical_exactly_the_same_counter/max_iterations<<std::endl;
    LOG(INFO) << "the average time consumption of numerical ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
}


TEST(KinematicsTest, CombinedIKTest)
{
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);

    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t ik_combined_only_counter{},ik_combined_with_right_reference{}, ik_combined_exactly_the_same_counter{};
    const size_t max_iterations = 1E8;
    const double zero_thresh = 1e-3;
    double all_time = 0;

    for (size_t index = 0; index < max_iterations; ++index) {

        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);

        auto desired_pose = aubo_i5_kinematics.fk(test_joint);

        //check analytical ik without right reference (make \theta6 as 0)
        Eigen::MatrixXd joint_solutions;
        clock_t start = clock();
        bool found_ik =  aubo_i5_kinematics.allValidIkSolutions(joint_solutions, desired_pose.SE3Matrix(), nullptr);
        clock_t end = clock();

        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(state_space::JointSpace(joint_solutions.col(0))))
            < zero_thresh) {
            ik_combined_only_counter++;
        }
        //check analytical ik with right reference
        auto refer_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        refer_joint = planner::extend(test_joint,refer_joint,0.01);

        joint_solutions.resize(0,0);
        start = clock();
        found_ik =  aubo_i5_kinematics.allValidIkSolutions(joint_solutions, desired_pose.SE3Matrix(), &refer_joint);
        end = clock();

        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(state_space::JointSpace(joint_solutions.col(0))))
            < zero_thresh) {
            ik_combined_with_right_reference++;
            for (size_t i = 0; i < joint_solutions.cols(); ++i) {
                if (planner::distance(test_joint, state_space::JointSpace(joint_solutions.col(i))) < zero_thresh) {
                    ik_combined_exactly_the_same_counter++;
                    break;
                }
            }
        }

        all_time += double(end - start) / CLOCKS_PER_SEC;
    }
    EXPECT_GE(ik_combined_only_counter,0.9999943*max_iterations)<< "Coverage rate of combined IK solution with zero reference doesn't reach 5 sigma level, but"
            << ik_combined_only_counter << "/" << max_iterations;;
    LOG(INFO)<<"Coverage rate of combined IK solution with zero reference"<< ik_combined_only_counter << "/" << max_iterations<<(double )ik_combined_only_counter/max_iterations<<std::endl;
    EXPECT_EQ(max_iterations, ik_combined_with_right_reference)
                        << "Coverage rate of combined IK solution (equals to fk) with right reference is not 100%, but"
                        << ik_combined_with_right_reference << "/" << max_iterations;

    LOG(INFO)<<"Coverage rate of combined IK solution (equals to fk) with right reference"<< ik_combined_with_right_reference << "/" << max_iterations<<(double )ik_combined_with_right_reference/max_iterations<<std::endl;
    EXPECT_GE(ik_combined_exactly_the_same_counter, 0.9973 * max_iterations)
                        << "Coverage rate of exactly the same combined IK solution with right reference doesn't reach 3 sigma level, but"
                        << ik_combined_exactly_the_same_counter << "/" << max_iterations;
    LOG(INFO)<<"Coverage rate of exactly the same combined IK solution with right reference"<< ik_combined_exactly_the_same_counter<< "/" << max_iterations<<(double )ik_combined_exactly_the_same_counter/max_iterations<<std::endl;
    LOG(INFO) << "the average time consumption of combined ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
}

TEST(KinematicsTest, NearestIKTest)
{
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);

    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t ik_nearest_only_counter{}, ik_nearest_with_right_reference{}, ik_nearest_exactly_the_same_counter{};
    const size_t max_iterations = 1E8;
    const double zero_thresh = 1e-3;
    double all_time = 0;

    for (size_t index = 0; index < max_iterations; ++index) {

        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);

        auto desired_pose = aubo_i5_kinematics.fk(test_joint);

        auto refer_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        refer_joint = planner::extend(test_joint,refer_joint,0.01);

        //check analytical ik without right reference (make \theta6 as 0)
        state_space::JointSpace raw_solution;
        clock_t start = clock();
        bool found_ik = aubo_i5_kinematics.nearestIkSolution(raw_solution, desired_pose.SE3Matrix(), refer_joint,
                                                             false);
        clock_t end = clock();

        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(raw_solution))
            < zero_thresh)
            ik_nearest_only_counter++;

        //check analytical ik with right reference
        found_ik = aubo_i5_kinematics.nearestIkSolution(raw_solution, desired_pose.SE3Matrix(), refer_joint, true);
        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(raw_solution))
            < zero_thresh) {
            ik_nearest_with_right_reference++;
            if (planner::distance(test_joint, raw_solution) < zero_thresh) {
                ik_nearest_exactly_the_same_counter++;
            }
        }

        all_time += double(end - start) / CLOCKS_PER_SEC;
    }
    EXPECT_EQ(max_iterations, ik_nearest_with_right_reference)
                        << "Coverage rate of nearest IK solution (equals to fk) with right reference is not 100%, but"
                        << ik_nearest_with_right_reference << "/" << max_iterations;
    LOG(INFO)<<"Coverage rate of nearest IK solution (equals to fk) with right reference"<< ik_nearest_with_right_reference << "/" << max_iterations<<(double )ik_nearest_with_right_reference/max_iterations<<std::endl;
    EXPECT_GE(ik_nearest_only_counter, 0.9999973 * max_iterations)
                        << "Coverage rate of nearest IK solution with zero reference doesn't reach 5 sigma level, but"
                        << ik_nearest_only_counter << "/" << max_iterations;
    LOG(INFO)<<"Coverage rate of nearest IK solution with zero reference"<< ik_nearest_only_counter << "/" << max_iterations<<(double )ik_nearest_only_counter/max_iterations<<std::endl;
    EXPECT_GE(ik_nearest_exactly_the_same_counter, 0.9973 * max_iterations)
                        << "Coverage rate of exactly the same nearest IK solution with right reference doesn't reach 3 sigma level, but"
                        << ik_nearest_exactly_the_same_counter << "/" << ik_nearest_with_right_reference;
    LOG(INFO)<<"Coverage rate of exactly the same nearest IK solution with right reference"<< ik_nearest_exactly_the_same_counter << "/" << max_iterations<<(double )ik_nearest_exactly_the_same_counter/max_iterations<<std::endl;
    LOG(INFO) << "the average time consumption of nearest ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
}



int main(int argc, char **argv)
{
    logger lg(argv[0],"/home/xcy/log");
    logger::setFlagLogToStd(true);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc,argv,"kTest");
    return RUN_ALL_TESTS();
}



