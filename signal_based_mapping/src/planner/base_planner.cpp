//
// Created by Ragesh on 4/3/18.
//

#include "planner/base_planner.h"

using namespace myPlanner;


 void base_planner::generate_path(double start_time){
    via_points point;

     set_planStartTime(start_time);

    // first point in the path
    point.modes=MOTION_MODES::ROTATION_Z;
    point.motion_end_time=planStartTime+planTime/2;
    point.vel_control.a=robotTwist.a;
    point.des_pose=startPose+Stg::Pose(0.0, 0.0, 0.0, M_PI/4);
    point.computed_desPose=true;

    path.push(point); //  pushed the first point

    // second point in the path
    point.modes=MOTION_MODES::TRANSLATION_X;
    point.motion_end_time+=planTime/2;
    point.vel_control.x=robotTwist.x;
    point.des_pose=Stg::Pose(0.0, 0.0, 0.0, 0.0);
    point.computed_desPose = false;

    path.push(point);

     // Print the signature of the function in gcc or g++ compiler
     //std::cout<<__PRETTY_FUNCTION__ << ": " <<std::endl ;



}

 Stg::meters_t levyWalk_planner::generate_levy_dist() {
    /**
     * Generate the length from the distribution
     */
    //uniform random number generator
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    // generate a uniform random number between 0 and 1
    auto u = distribution(generator);


    if(alpha == 0){
        throw "\nLevy alpha should be non zero\n";
    }


    double speed = std::sqrt(get_velocity()->x*get_velocity()->x +
                    get_velocity()->y*get_velocity()->y +
                    get_velocity()->z*get_velocity()->z);


    if(speed == 0){
        throw "\nSpeed should be non zero\n";
    }


    return levy_min * pow((1-u),-1/alpha); // levy walk distance


}


Stg::radians_t levyWalk_planner::generate_random_direction(){
    /// Generates a random angle between min_angle and max_angle
    //uniform random number generator
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution(min_ang, max_ang);
    // generate a uniform random number between min and max
    return distribution(generator);
}




 void levyWalk_planner::generate_path(double start_time) {
    /**
     * The function generates a path for the robot to perform levy walk in an unbounded domain
     */

     via_points point;

     set_planStartTime(start_time);


     // computing first point in the path
     point.modes=MOTION_MODES::ROTATION_Z;
     auto w = std::fabs(get_velocity()->a); // omega in magnitude
     const Stg::radians_t& des_dir=point.des_pose.a; // ref to the desired direction
     const Stg::radians_t& cur_dir=get_startPose()->a; // ref to the current direction
     point.des_pose = Stg::Pose(get_startPose()->x, get_startPose()->y, get_startPose()->z, generate_random_direction());

     // find the smallest angle to rotate, to find the right \omega
     // case 1 : when both of them are ++ or --
     if(des_dir * cur_dir >= 0){
         // time to complete the motion
         point.motion_end_time=std::fabs(des_dir - cur_dir)/std::fabs(w);
         if(des_dir > cur_dir){
             // rotate in anticlockwise direction(left turn)
             point.vel_control=Stg::Velocity(0.0, 0.0, 0.0, std::fabs(w));
         } else {
             // rotate in clockwise direction(right turn)
             point.vel_control=Stg::Velocity(0.0, 0.0, 0.0, -std::fabs(w));
         }
     }
     // case 2 : when cur_dir is positive and des_dir is negative
     if(cur_dir > 0 && des_dir < 0){
         // find the smallest angle between the current and the desired directions
         if (std::fabs(cur_dir - des_dir) < std::fabs(M_PI - cur_dir) + std::fabs(-M_PI - des_dir)){
             // rotate in clockwise direction(right turn)
             point.vel_control=Stg::Velocity(0.0, 0.0, 0.0, -std::fabs(w));
             // time to complete the motion
             point.motion_end_time=std::fabs(cur_dir - des_dir)/std::fabs(w);
         } else {
             // rotate in anticlockwise direction(left turn)
             point.vel_control=Stg::Velocity(0.0, 0.0, 0.0, std::fabs(w));
             // time to complete the motion
             point.motion_end_time=(std::fabs(M_PI - cur_dir)+std::fabs(-M_PI - des_dir))/std::fabs(w);
         }
     }
     // case 3 : when cur_dir is negative and des_dir is positive
     if(cur_dir < 0 && des_dir > 0){
         // find the smallest angle between the current and desired directions
         if (std::fabs(cur_dir - des_dir) < std::fabs(-M_PI - cur_dir) + std::fabs(M_PI - des_dir)){
             // rotate in anticlockwise direction(left turn)
             point.vel_control=Stg::Velocity(0.0, 0.0, 0.0, std::fabs(w));
             // time to complete the motion
             point.motion_end_time=std::fabs(cur_dir - des_dir)/std::fabs(w);
         } else {
             // rotate in clockwise direction(right turn)
             point.vel_control=Stg::Velocity(0.0, 0.0, 0.0, -std::fabs(w));
             // time to complete the motion
             point.motion_end_time=(std::fabs(-M_PI - cur_dir)+std::fabs(M_PI - des_dir))/std::fabs(w);
         }
     }
     point.motion_end_time+=start_time;
     point.computed_desPose=true;


     path.push(point); // pushing the first point to the path

     // computing the second point in the path
     point.modes=MOTION_MODES::TRANSLATION;
     auto speed = std::sqrt(get_velocity()->x*get_velocity()->x +
                            get_velocity()->y*get_velocity()->y +
                            get_velocity()->z*get_velocity()->z);
     if(speed == 0){
         throw "The speed can not be zero";
     }
     point.motion_end_time+=generate_levy_dist()/speed;
     point.vel_control=Stg::Velocity(get_velocity()->x, get_velocity()->y, get_velocity()->z, 0);
     point.computed_desPose= false;

     path.push(point); // pushing the second point to the path


}

