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
    Path* path = planner.generate_path();
    while(!path->empty()){
        std::cout<<"\n The control laws are \n";
        std::cout<<"\n x : "<<path->front().vel_control.x;
        std::cout<<"\n y : "<<path->front().vel_control.y;
        std::cout<<"\n z : "<<path->front().vel_control.z;
        std::cout<<"\n a : "<<path->front().vel_control.a;
        std::cout<<"\n enable : "<<path->front().computed_desPose<<std::endl;
        path->pop();
    }

}