#define EIGEN_USE_MKL_ALL

#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include "Kinematics/custom_kinematics.hpp"
#include "planner/CartesianPlanner.h"
#include "planner/RRT.hpp"
int main(int argc, char **argv)
{

    logger a( argv[0]);
    ros::init(argc, argv, "NursingRobot");
    EigenSTL::vector_Affine3d waypoints;
    std::vector<std::array<geometry_msgs::Point,2>> angular_velocity_box;
    using SPECIFIC_STATE = state_space::SE3;
    auto start_state = planner::randomState<SPECIFIC_STATE>();
    auto goal_state = planner::randomState<SPECIFIC_STATE>();
    //sphere interpolate

    planner::RRT<SPECIFIC_STATE> rrt_2d(planner::hash<SPECIFIC_STATE>, start_state.Dimensions());


    rrt_2d.setStepLen(0.05);
    rrt_2d.setGoalMaxDist(0.05);
    rrt_2d.constructPlan(planner::RRT_REQUEST<SPECIFIC_STATE>(start_state, goal_state, 50));
    std::vector<SPECIFIC_STATE> path;
    if (rrt_2d.planning()) {
        path = rrt_2d.GetPath();
        for(const auto & it:path)
        {
            waypoints.push_back(Eigen::Affine3d(it.SE3Matrix()));
        }
    }

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.loadRemoteControl();

    visual_tools.publishAxisLabeled(waypoints[0], " start", rvt::LARGE);
    visual_tools.publishAxisLabeled(waypoints.back(), " end ", rvt::LARGE);
    //visual_tools.publishAxisLabeled(Eigen::Affine3d(center.SE3Matrix())," center ",rvt::LARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (const auto & waypoint : waypoints)
        visual_tools.publishAxis(waypoint, rvt::SMALL);
    visual_tools.trigger();

    return 0;
}
