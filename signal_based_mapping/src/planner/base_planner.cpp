//
// Created by Ragesh on 4/3/18.
//

#include "planner/base_planner.h"

using namespace myPlanner;


 Path* base_planner::generate_path(){
    via_points point;

    // first point in the path
    point.modes=MOTION_MODES::ROTATION_Z;
    point.motion_end_time=planStartTime+planTime/2;
    point.vel_control.a=startVelocity.a;
    point.des_pose=startPose+Stg::Pose(0.0, 0.0, 0.0, M_PI/4);
    point.computed_desPose=true;

    path.push(point); //  pushed the first point

    // second point in the path
    point.modes=MOTION_MODES::TRANSLATION_X;
    point.motion_end_time+=planTime/2;
    point.vel_control.x=startVelocity.x;
    point.des_pose=Stg::Pose(0.0, 0.0, 0.0, 0.0);
    point.computed_desPose = false;

    path.push(point);

     //std::cout<<__PRETTY_FUNCTION__ << ": " <<std::endl ;

    return &path;



}


Path* levyWalk_planner::generate_path() {
    /**
     * The function generates a path for the robot to perform levy walk in an unbounded domain
     */

    via_points point;

    // first point in the path
    point.modes=MOTION_MODES::ROTATION_Z;
    //uniform random number generator
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution(min, max);
    // generate a uniform random number between min and max
    desired_levy_direction = distribution(generator);

}

