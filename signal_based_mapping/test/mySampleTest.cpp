//
// Created by Ragesh on 3/22/18.
//

/**
 * This is a test cpp file for intermediate testing
 */


#include "robot/robot.h"


int main() {

    myRobot::robot robot;
    std::cout<<"robot id :"<<robot.get_robot_id()<<std::endl;
    occupancy_grid::Prob_occupancyGrid2D<double,int> map;
    cv::Vec2d final_pos;
    bool reflect;
    //occupancy_grid::ray_trace_iterator<double, int> it{0,0,2,2,2,2,2,2};
    map.ray_trace(0,0,1.5,2,final_pos,reflect);



}