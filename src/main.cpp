#include <iostream>
#include "SE3.h"

int main() {
    LieGroup::R6 a;
    a<<1,2,3,4,5,6;
    LieGroup::SE3 pose(a);
    LieGroup::SE3 check_pose(pose.SE3Matrix());

    std::cout<<"SO3 Axis check: "<<pose.Axis()<<std::endl;
    std::cout<<"SO3 Vector check: "<<pose.Vector()<<std::endl;
    std::cout<<"SO3 Theta check: "<<pose.Theta()<<std::endl;
    std::cout<<"SO3 so3 check: "<<pose.se3Matrix()<<std::endl;
    std::cout<<"SO3 unit so3 check: "<<pose.unitse3Matrix()<<std::endl;
    std::cout<<"SO3 SO3 check:" <<pose.SE3Matrix()<<std::endl;

    std::cout<<"SO3 Axis check: "<<check_pose.Axis()<<std::endl;
    std::cout<<"SO3 Vector check: "<<check_pose.Vector()<<std::endl;
    std::cout<<"SO3 Theta check: "<<check_pose.Theta()<<std::endl;
    std::cout<<"SO3 so3 check: "<<check_pose.se3Matrix()<<std::endl;
    std::cout<<"SO3 unit so3 check: "<<check_pose.unitse3Matrix()<<std::endl;
    std::cout<<"SO3 SO3 check:" <<check_pose.SE3Matrix()<<std::endl;

}
