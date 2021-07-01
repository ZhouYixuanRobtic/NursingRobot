/***
 * This file used to test interpolate method like linear in Rn, Tn, SO3,SE3
 *
 * @author Yixuan Zhou
 * @time 2011/05/18
 */
#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include "planner/PlannerMethod.hpp"
#include <fstream>

void visual_tool(moveit_visual_tools::MoveItVisualTools& visual_tools,
                 const std::vector<state_space::SO3>& test_points,
                 const state_space::Rn& start_position,
                 const state_space::Rn& end_position,
                 std::size_t max_iterations,
                 std::ofstream& out_file){

    EigenSTL::vector_Affine3d waypoints;
    std::vector<std::array<geometry_msgs::Point,2>> omega_points;
    waypoints.resize(max_iterations);
    omega_points.resize(max_iterations);

    for(int j=0;j<max_iterations-1;++j)
    {
        Eigen::Vector3d inter_position = planner::interpolate(start_position,end_position,(double)j/max_iterations).Vector();
        //orientation
        waypoints[j]=(Eigen::Affine3d{state_space::SE3(test_points[j],inter_position).SE3Matrix()});
        //omega
        geometry_msgs::Point omega_start=tf2::toMsg(inter_position);
        Eigen::Vector3d omega = (test_points[j+1]-test_points[j]).Vector();
        out_file<<omega.norm()<<std::endl;
        geometry_msgs::Point omega_end = tf2::toMsg(Eigen::Vector3d(inter_position+5.0*omega));
        std::array<geometry_msgs::Point,2> temp_array{omega_start,omega_end};
        omega_points[j]=temp_array;
    }
    waypoints[max_iterations-1]=(Eigen::Affine3d{state_space::SE3(test_points[max_iterations-1],end_position.Vector()).SE3Matrix()});
    omega_points[max_iterations-1]=omega_points[max_iterations-2];
    namespace rvt = rviz_visual_tools;

    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(waypoints[0], " start", rvt::LARGE);
    visual_tools.publishAxisLabeled(waypoints.back(), " end ", rvt::LARGE);
    //visual_tools.publishAxisLabeled(Eigen::Affine3d(center.SE3Matrix())," center ",rvt::LARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (int j=0;j<max_iterations;j++) {
        visual_tools.publishAxis(waypoints[j], rvt::SMALL);
        visual_tools.publishArrow(omega_points[j][0],omega_points[j][1],rviz_visual_tools::colors::YELLOW,rviz_visual_tools::scales::MEDIUM);
    }
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
}

TEST(interpTest,SO3Test){

    ros::AsyncSpinner spinner(2);
    spinner.start();
    //given two SO3 element
    auto start = planner::randomState<state_space::SO3>();
    auto end = planner::randomState<state_space::SO3>();
    const std::string output_file_name{"/home/xcy/InterpData/omega.txt"};
     std::ofstream out_file(output_file_name,std::ios::out | std::ios::trunc);
    int i=0;
    const int max_iterations=100;
    //SO3 linear interp and SO3 5th order bezier polynomial interpolation
    std::vector<state_space::SO3> SO3Lerp{},SO3Berp{};
    while(i++<max_iterations){
        auto inter_state = planner::interpolate(start,end,(double)i/max_iterations);
        auto binter_state = planner::bezierInterpolate(start,end,(double)i/max_iterations);

        SO3Lerp.emplace_back(inter_state);
        SO3Berp.emplace_back(binter_state);
    }

    //SO3 RPY interp
    auto rpy_start = state_space::JointSpace{start.RPY()};
    auto rpy_end = state_space::JointSpace{end.RPY()};
    i=0;
    std::vector<state_space::SO3>RPYLerp{};
    while(i++<max_iterations) {
        auto inter_state = planner::interpolate(rpy_start, rpy_end, (double) i / max_iterations);
        auto temp_SO3 = state_space::SO3(inter_state[0], inter_state[1], inter_state[2]);
        RPYLerp.emplace_back(temp_SO3);
    }
    //Visualization
    auto start_position=planner::randomState<state_space::Rn>(3, nullptr);
    auto end_position = planner::randomState<state_space::Rn>(3, nullptr);

    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.loadRemoteControl();
    visual_tool(visual_tools,SO3Lerp,start_position,end_position,max_iterations,out_file);
    visual_tool(visual_tools,SO3Berp,start_position,end_position,max_iterations,out_file);
    visual_tool(visual_tools,RPYLerp,start_position,end_position,max_iterations,out_file);
    spinner.stop();
    if(out_file.is_open())
        out_file.close();
}



int main(int argc, char **argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "InterpTest");
    return RUN_ALL_TESTS();
}