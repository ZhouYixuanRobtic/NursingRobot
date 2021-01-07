#define EIGEN_USE_MKL_ALL
#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include "planner/RRT.hpp"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include "Kinematics/custom_kinematics.hpp"

int main(int argc, char **argv)
{
    logger a("/home/xcy/WorkSpace/src/NursingRobot/log/", argv[0]);
    ros::init(argc, argv, "NursingRobot");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    my_collision_detection::MoveItCollisionHelper moveItCollisionHelper(
            "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
            "manipulator_i5",
            kinematics::aubo_i5_analytical_IK);

    const auto& kinematicsPtr = moveItCollisionHelper.getKinematicsPtr();

    using SPCIFIC_STATE = state_space::JointSpace;
    Eigen::MatrixX2d bounds;
    bounds.resize(6, 2);
    bounds << state_space::R6().setConstant(3.05),
            state_space::R6().setConstant(-3.05);

    auto start_state = planner::randomState<SPCIFIC_STATE>();
    auto goal_state = planner::randomState<SPCIFIC_STATE>();

    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> waypoints;
    double step = 0;
    while (step <= 1) {
        auto temp_state = planner::bezierInterpolate(start_state, goal_state, step);
        step += 0.01;
        waypoints.emplace_back(Eigen::Affine3d(kinematicsPtr->fk(temp_state).SE3Matrix()));
    }
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();  // clear all old markers

    visual_tools.loadRemoteControl();
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(waypoints[0], "Joint Space start", rvt::LARGE);
    visual_tools.publishAxisLabeled(waypoints.back(), "Joint Space goal", rvt::LARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();

    /*
    while(true)
    {
        start_state = planner::randomState<SPCIFIC_STATE>();
        goal_state = planner::randomState<SPCIFIC_STATE>();
        if(moveItCollisionHelper.isStateValid(start_state)&&moveItCollisionHelper.isStateValid(goal_state))
            break;
    }

    std::shared_ptr<planner::RRT<SPCIFIC_STATE>> rrt_2d = std::make_shared<planner::RRT<SPCIFIC_STATE>>(planner::hash<SPCIFIC_STATE>,
                                                                                                        start_state.Dimensions());


    rrt_2d->setStateValidator( std::function<bool(const SPCIFIC_STATE &, const SPCIFIC_STATE &)>(
                                std::bind(&my_collision_detection::MoveItCollisionHelper::isPathValid<SPCIFIC_STATE>,
                                        &moveItCollisionHelper, std::placeholders::_1,std::placeholders::_2)
                                        ));
    rrt_2d->setGoalBias(0.2);
    rrt_2d->setStepLen(0.02);
    rrt_2d->setGoalMaxDist(0.02);
    rrt_2d->constructPlan(start_state,goal_state);
    clock_t start(clock());
    if (rrt_2d->planning()) {
        clock_t end(clock());
        std::cout << (double) (end - start) / CLOCKS_PER_SEC << std::endl;
        std::vector<SPCIFIC_STATE, Eigen::aligned_allocator<SPCIFIC_STATE>> path = rrt_2d->GetPath();
        std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> waypoints;
        for (const auto& it: path) {
            std::cout << it << std::endl;
            waypoints.emplace_back(Eigen::Affine3d(kinematicsPtr->fk(it).SE3Matrix()));
        }
        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
        visual_tools.deleteAllMarkers();  // clear all old markers

        visual_tools.loadRemoteControl();
        Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(waypoints[0], "Joint Space start",  rvt::LARGE);
        visual_tools.publishAxisLabeled(waypoints.back(), "Joint Space goal",  rvt::LARGE);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
        visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
        for (std::size_t i = 0; i < waypoints.size(); ++i)
            visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
        visual_tools.trigger();
    }*/

    ros::waitForShutdown();
    return 0;


}
