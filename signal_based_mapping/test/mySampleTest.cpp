//
// Created by Ragesh on 3/22/18.
//

/**
 * This is a test cpp file for intermediate testing
 */


#include "robot/robot.h"


int main() {

    std::string name{"robot"};
    Stg::Pose i_pose{0, 0, 0, 0};
    Stg::Velocity i_vel{1, 1, 0, 0};
    const double TOL = 0.0001;
    double l_min{1.0};
    double l_alpha{1.5};
    double l_start_time{0};
    myRobot::robot ro{name, i_pose, i_vel, l_min, l_alpha, l_start_time};
    // Robot name test
    assert(ro.get_robot_name() == name);
    std::cout<<"\nRobot name test passed";
    // Levy dist generation test
    ro.generate_levy_dist_time();
    std::cout<<"\n levy distance generated : "<<ro.get_levy_dis()<<std::endl;
    std::cout<<"\n levy time of flight : "<<ro.get_levy_total_time()<<std::endl;
    assert(std::abs(ro.get_levy_dis() / ro.get_levy_total_time() - ro.get_current_linear_speed()) < TOL);
    std::cout<<"\nSpeed = dist / time test passed";
    // Alpha value zero test
    ro.set_levy_alpha(0);
    assert(!ro.generate_levy_dist_time());
    std::cout<<"\nAlpha value zero test passed\n";
    // Zero speed test
    ro.set_current_velocity(0, 0, 0, 2);
    ro.set_levy_alpha(l_alpha);
    assert(!ro.generate_levy_dist_time());
    std::cout<<"\nSpeed zero test passed\n";
    // Random angle test
    assert(ro.generate_random_direction() < 3.14);
    std::cout<<"\nangle less than PI test passed\n";
    assert(ro.generate_random_direction() > -3.14);
    std::cout<<"\nangle greater than -PI test passed\n";
    assert(ro.generate_random_direction(0, M_PI/2) < 3.14/2.0);
    std::cout<<"\nangle less than PI/2 test passed\n";
    assert(ro.generate_random_direction(0, M_PI/2) > 0);
    std::cout<<"\nangle greater than 0 test passed\n";


}