#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include <fstream>
#include "planner/PlanningInterface.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>

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
TEST(planning_interface_test,compute_joint_path_test)
{
    ros::AsyncSpinner spinner(2);
    spinner.start();

    const std::size_t max_iterations = 10;

    const std::string base_path{"/home/xcy/RRT_EXP/"};
    const std::string goal_list{"Goals.txt"};
    const std::vector<std::string> txt_names{{"/time.txt"},{"/pathLength.txt"},{"/ccTimes.txt"},
                                             {"/nodes.txt"},{"/invalidGoals.txt"},{"/performance.txt"}};
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);
    std::string final_path = base_path+goal_list;

    for(int j=0; j<4;++j){

        std::size_t iter_index = 0;

        //average use
        std::size_t total_time{};
        std::size_t valid_times=0;
        double total_length{};
        std::size_t total_path_size{};
        std::size_t total_nodes{};
        std::size_t total_ccTimes{};
        std::ifstream goal_src(final_path.c_str());
        planner::PlanningInterface planningInterface(planner::PLANNER_TYPE(j),
                                                     "manipulator_i5",
                                                     "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                     my_kinematics::aubo_i5_analytical_IK);
        planningInterface.getRandomValidJointState();
        //open txt
        const std::string planner_name = planningInterface.getPlannerName();
        std::cout<<"planner name: "<<planner_name<<std::endl;
        std::vector<std::ofstream> outfile_vector{};
        outfile_vector.resize(txt_names.size());
        for(int i=0;i<txt_names.size();++i){
            outfile_vector[i].open(base_path+planner_name+txt_names[i],std::ios::out | std::ios::trunc);
        }

        auto start_valid = state_space::JointSpace(std::vector<double>{0,0,0,0,0,0});
        EigenSTL::vector_Affine3d waypoints;
        moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
        visual_tools.deleteAllMarkers();  // clear all old markers
        visual_tools.loadRemoteControl();
        namespace rvt = rviz_visual_tools;
        while(++iter_index<=max_iterations){

            //get goal
            /*
            state_space::JointSpace goal_valid{};
            std::string s;getline(goal_src,s);
            std::istringstream stringGet(s);
            stringGet>>goal_valid[0]>>goal_valid[1]>>goal_valid[2]>>goal_valid[3]>>goal_valid[4]>>goal_valid[5];*/
            //std::cout<<iter_index<<std::endl;
            state_space::JointSpace goal_valid = state_space::JointSpace{std::vector<double>{  1.22946,-0.433116,2.55407,-0.404748 ,1.44322,-0.204923}};

            //planning
            state_space::vector_JointSpace path;
            auto plan_request = planner::PLAN_REQUEST<state_space::JointSpace>(start_valid,
                                                                               goal_valid,
                                                                               5,
                                                                               7,false,0.3,0.05);
            clock_t start(clock());
            bool found_path = planningInterface.computeJointPath(path,plan_request);
            if(found_path){
                //valid times
                valid_times++;
                //single time
                outfile_vector[0]<<(double)(clock()-start)/CLOCKS_PER_SEC<<std::endl;
                //single length
                double path_length{};
                for(int i=0; i<path.size()-1;++i){
                    path_length += planner::distance(path[i],path[i+1]);
                }
                outfile_vector[1]<<path.size()<<" "<<path_length<<std::endl;
                //single cc times
                outfile_vector[2]<<planningInterface.getCCTimes()<<std::endl;
                //single iter times
                outfile_vector[3]<<planningInterface.getTotalNodes()<<std::endl;

                //average usage
                total_time += clock() - start;
                total_length += path_length;
                total_path_size += path.size();
                total_ccTimes += planningInterface.getCCTimes();
                total_nodes+=planningInterface.getTotalNodes();

                planningInterface.clearCCTimes();
            }else{
                outfile_vector[4]<<goal_valid<<std::endl;
            }

            //ASSERT_TRUE(found_path)<<"No path found at "<<iter_index<<"th try"<<goal_valid<<std::endl<<start_valid;
        }
        //overall performance
        outfile_vector[5]<<"average planning time: "<<(double)total_time / valid_times / CLOCKS_PER_SEC<<"s"<<std::endl;
        outfile_vector[5]<<"average path length: "<<total_length/valid_times<<std::endl;
        outfile_vector[5]<<"average path size: "<<(double)total_path_size/valid_times<<std::endl;
        outfile_vector[5]<<"average cc Times: "<<(long double)total_ccTimes/valid_times<<std::endl;
        outfile_vector[5] << "average nodes: " << (long double)total_nodes / valid_times << std::endl;
        outfile_vector[5]<<"valid percent: "<<valid_times<<"/"<<max_iterations<<": "<<(double)valid_times/max_iterations<<std::endl;

        if(goal_src.is_open()) {
            goal_src.close();
        }

        for(int i=0; i<txt_names.size();++i){
            if(outfile_vector[i].is_open()){
                outfile_vector[i].close();
            }
        }
    }
    spinner.stop();
}



int main(int argc, char**argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "carTest");
    return RUN_ALL_TESTS();
}