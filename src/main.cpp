#define EIGEN_USE_MKL_ALL

#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include "Kinematics/custom_kinematics.hpp"
int main(int argc, char **argv)
{

    logger a( argv[0],"/home/xcy/WorkSpace/src/NursingRobot/log/");
    ros::init(argc, argv, "NursingRobot");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    my_collision_detection::MoveItCollisionHelper moveItCollisionHelper(
            "manipulator_i5",
            "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
            my_kinematics::aubo_i5_analytical_IK);


    auto current_pose = moveItCollisionHelper.getEndEffectorPose();


    Eigen::Vector3d path_vector{0,-0.3,0};
    EigenSTL::vector_Affine3d waypoints;
    const std::size_t NUM =30;
    for(std::size_t i=0; i<NUM; ++i){
        double percentage = (double) i / (double) NUM;
        auto temp_state = current_pose+state_space::SE3(state_space::SO3::Zero(), percentage * path_vector);
        waypoints.emplace_back(Eigen::Affine3d(temp_state.SE3Matrix()));
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
    ros::waitForShutdown();
    return 0;


}
