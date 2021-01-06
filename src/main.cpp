#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include "planner/RRT.hpp"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
int main(int argc, char** argv)
{
    logger a("/home/xcy/WorkSpace/src/NursingRobot/log/",argv[0]);
    ros::init(argc, argv, "NursingRobot");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    my_collision_detection::MoveItCollisionHelper moveItCollisionHelper("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                                        "manipulator_i5");

    using SPCIFIC_STATE = state_space::SE3;
    Eigen::MatrixX2d bounds;
    bounds.resize(3,2);
    bounds<<state_space::R3().setConstant(0.88),
            state_space::R3().setConstant(-0.88);

    auto start_state = planner::randomState<SPCIFIC_STATE>();
    auto goal_state = planner::randomState<SPCIFIC_STATE>();
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> waypoints;
    std::vector<std::array<geometry_msgs::Point,2>> twist_arrow;
    double step =0;
    while(step<=1)
    {
        auto temp_state = planner::interpolate(start_state,goal_state,step);
        waypoints.emplace_back(Eigen::Affine3d(temp_state.SE3Matrix()));
        std::array<geometry_msgs::Point,2> temp_array{tf2::toMsg(temp_state.translationPart()),
                                                      tf2::toMsg(Eigen::Vector3d(temp_state.translationPart()+0.1*temp_state.Vector().block<3,1>(0,0)))};
        twist_arrow.emplace_back(temp_array);
        step+= 0.01;
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
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
        visual_tools.publishArrow(twist_arrow[i][0],twist_arrow[i][1],rvt::colors::ORANGE);
    }
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    waypoints.clear();
    twist_arrow.clear();
    step =0;
    while(step<=1)
    {
        auto temp_state = planner::interpolate(state_space::Rn(start_state.translationPart()),state_space::Rn(goal_state.translationPart()),step);
        Eigen::Quaterniond temp_quater1(start_state.SO3Part().Quaternion());
        Eigen::Quaterniond temp_quater2(goal_state.SO3Part().Quaternion());
        auto temp_quater = temp_quater1.slerp(step,temp_quater2);
        Eigen::Affine3d temp_transform;
        temp_transform.translation() = temp_state.Vector();
        temp_transform.linear() = temp_quater.toRotationMatrix();
        waypoints.emplace_back(temp_transform);
        std::array<geometry_msgs::Point,2> temp_array{tf2::toMsg(Eigen::Vector3d(temp_state.Vector())),
                                                      tf2::toMsg(Eigen::Vector3d(temp_state.Vector()+
                                                      0.1*state_space::SE3(temp_transform.matrix()).Vector().block<3,1>(0,0)))};

        twist_arrow.emplace_back(temp_array);
        step+= 0.01;
    }
    visual_tools.publishPath(waypoints, rvt::colors::RED, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        visual_tools.publishAxisLabeled(waypoints[i], "apt" + std::to_string(i), rvt::SMALL);
        visual_tools.publishArrow(twist_arrow[i][0],twist_arrow[i][1],rvt::colors::ORANGE);
    }
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    waypoints.clear();
    twist_arrow.clear();
    step =0;
    while(step<=1)
    {
        auto temp_state = planner::bezierInterpolate(start_state,goal_state,step);
        waypoints.emplace_back(Eigen::Affine3d(temp_state.SE3Matrix()));
        std::array<geometry_msgs::Point,2> temp_array{tf2::toMsg(temp_state.translationPart()),
                                                      tf2::toMsg(Eigen::Vector3d(temp_state.translationPart()+0.1*temp_state.Vector().block<3,1>(0,0)))};
        twist_arrow.emplace_back(temp_array);
        step+= 0.01;
    }
    visual_tools.publishPath(waypoints, rvt::colors::BLUE, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        visual_tools.publishAxisLabeled(waypoints[i], "apt" + std::to_string(i), rvt::SMALL);
        visual_tools.publishArrow(twist_arrow[i][0],twist_arrow[i][1],rvt::colors::ORANGE);
    }
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

    rrt_2d->setStepLen(0.05);
    rrt_2d->setGoalMaxDist(0.05);
    rrt_2d->constructPlan(start_state,goal_state);
    clock_t start(clock());
    if (rrt_2d->planning()) {
        clock_t end(clock());
        std::cout << (double) (end - start) / CLOCKS_PER_SEC << std::endl;
        std::vector<SPCIFIC_STATE, Eigen::aligned_allocator<SPCIFIC_STATE>> path = rrt_2d->GetPath();
        std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> waypoints;
        for (const auto& it: path) {
            std::cout << it << std::endl;
            waypoints.emplace_back(Eigen::Affine3d(it.SE3Matrix()));
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
