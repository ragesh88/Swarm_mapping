//
// Created by Ragesh on 4/3/18.
//


/**
 * This is a test cpp file for intermediate testing
 *  of classes in myPlanner namespace
 */

#include "robot/robot.h"
#include "planner/base_planner.h"

using namespace myPlanner;


int main(){
    base_planner planner(10, 0, Stg::Pose(0,0,0,0),Stg::Velocity(.3, 0, 0, 0.2));
    planner.generate_path(10);
    Path* path = planner.get_path();
    while(!path->empty()){
        std::cout<<"\n The control laws are \n";
        std::cout<<"\n x : "<<path->front().vel_control.x;
        std::cout<<"\n y : "<<path->front().vel_control.y;
        std::cout<<"\n z : "<<path->front().vel_control.z;
        std::cout<<"\n a : "<<path->front().vel_control.a;
        std::cout<<"\n enable : "<<path->front().computed_desPose<<std::endl;
        path->pop();
    }

    std::cout<<"\n The Levy walk planner \n";

    levyWalk_planner levyWalkPlanner(0, Stg::Pose(10, 2, 0, 2.4), Stg::Velocity(0.6, 0, 0, 0.2));
    try {
        levyWalkPlanner.generate_path(10);
    }
    catch (const char* s){
        std::cerr<<s<<std::endl;
    }
    path = levyWalkPlanner.get_path();
    auto count = 0;
    while(!path->empty()){
        std::cout<<"\n The via point "<<++count<<" is \n";
        if(path->front().modes == START)
            std::cout<<"\n The point is start point \n";
        if(path->front().modes == ROTATION_Z)
            std::cout<<"\n The point is pure Rotation \n";
        if(path->front().modes == TRANSLATION)
            std::cout<<"\n The point is pure Translation \n";
        std::cout<<"\n velocity control : ";
        path->front().vel_control.Print(" ");
        std::cout<<"\n Enable : "<<path->front().computed_desPose<<std::endl;
        std::cout<<"\n Desired Pose : ";
        path->front().des_pose.Print(" ");
        std::cout<<"\n The time of motion : "<<path->front().motion_end_time<<std::endl;
        path->pop();
    }

}