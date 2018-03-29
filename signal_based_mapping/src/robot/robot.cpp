//
// Created by Ragesh on 3/23/18.
//

/**
 * This is the implementation of the member functions
 * of myRobot::robot class
 *
 */

#include "robot/robot.h"

using namespace myRobot;


bool robot::generate_levy_dist(){
    /// Generate the length from the distribution
    //uniform random number generator
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    // generate a uniform random number between 0 and 1
    double u = distribution(generator);

    try {
        if(levy_alpha == 0){
            throw "\nLevy alpha should be non zero\n";
        }
    }
    catch (const char* a){
        std::cout<<a;
        return FAILED;
    }

    double speed = get_current_linear_speed();

    try {
        if(speed == 0){
            throw "\nSpeed should be non zero\n";
        }
    }
    catch (const char* a){
        std::cout<<a;
        return FAILED;
    }

    levy_dis = levy_min * pow((1-u),-1/levy_alpha); // levy flight distance
    return SUCCESS;
}


Stg::radians_t robot::generate_random_direction(Stg::radians_t min, Stg::radians_t max){
    /// Generates a random angle between min and max and stores in
    /// desired_levy_direction
    /**
     * \param min : The minimum angle in radians
     * \param max : The maximum angle in radians
     *
     * The default values :
     * min : -M_PI
     * max : M_PI
     */
    //uniform random number generator
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution(min, max);
    // generate a uniform random number between min and max
    desired_levy_direction = distribution(generator);
    return desired_levy_direction;
}
