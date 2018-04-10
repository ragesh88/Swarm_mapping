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
using namespace myPlanner;
using namespace occupancy_grid;

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

 void robot::move(){
     /// The function moves the robot according to the planner object

     // Tolerance to check the various angle and time conditions
     const double rad_tol = 2*M_PI/180;
     const double time_tol = 0.1;
     static MOTION_MODES currentMode=MOTION_MODES::START;
     // variables for obstacle avoidance
     static const double stopDist = 0.3; // stopping distance of the robot
     static const int avoidDuration = 10; // duration to perform obstacle avoidance
     static const double avoidSpeed = 0.05;
     static const double avoidTurn = 0.5;
     static const double minFrontDistance = 1.0;
     bool obstruction = false; //flag for detecting obstruction
     bool stop = false; // flag to stop the robot

     // get the laser data
     const auto &scan = laser->GetSensors()[0].ranges;
     auto sample_count = scan.size();
     if (verbose)
         printf("\n sample count laser :%lu \n", sample_count);
     if (sample_count < 1)
         throw "There is no laser data";


     // find the closest distance to the left and right and also check if
     // there's anything in front
     double minleft = 1e6;
     double minright = 1e6;

     for (uint32_t i = 0; i < sample_count; i++) {
         if (verbose&& 0)
             printf("%.3f ", scan[i]);

         if ((i > (sample_count / 3)) && (i < (sample_count - (sample_count / 3)))
             && scan[i] < minFrontDistance) {
             if (verbose)
                 puts("  obstruction!");
             obstruction = true;
         }

         if (scan[i] < stopDist) {
             if (verbose)
                 puts("  stopping!");
             stop = true;
         }

         if (i > sample_count / 2)
             minleft = std::min(minleft, scan[i]);
         else
             minright = std::min(minright, scan[i]);
     }

     if (verbose) {
         puts("");
         printf("min left %.3f \n", minleft);
         printf("min right %.3f\n ", minright);
     }

     if (obstruction || stop || (avoidCount > 0)) {
         // delete the existing planned path
         planner->delete_path();
         currentMode=MOTION_MODES::START;
         if (verbose)
             printf("Avoid : %lu\n", avoidCount);

         position->SetXSpeed(stop ? 0.0 : avoidSpeed);

         // once we start avoiding, select a turn direction and stick
         // with it for a few iterations
         if (avoidCount < 1) {
             if (verbose)
                 puts("Avoid START");
             avoidCount = random() % avoidDuration + avoidDuration;

             if (minleft < minright) {
                 position->SetTurnSpeed(-avoidTurn);
                 if (verbose)
                     printf("turning right %.2f\n", -avoidTurn);
             } else {
                 position->SetTurnSpeed(+avoidTurn);
                 if (verbose)
                     printf("turning left %2f\n", +avoidTurn);
             }
         }

         avoidCount--;

     } else {

         // if there is no obstruction follow the planned path
         if (verbose)
             puts("\n Cruise \n");

         // check if the a path exist
         if(!planner->get_path()->empty()){
             if(verbose)
                 std::cout<<"\n path exist \n";
             if (verbose){
                 switch (planner->get_path()->front().modes){
                     case MOTION_MODES::ROTATION_Z: printf("\nRotation\n");
                         planner->get_path()->front().vel_control.Print("");
                         position->GetVelocity().Print("Actual ");
                         std::cout<<"\n The current angle(deg): "<<position->GetPose().a*(180/M_PI)<<std::endl;
                         std::cout<<"\n The desired angle(deg):"<<planner->get_path()->front().des_pose.a*(180/M_PI)<<std::endl;
                         break;
                     case MOTION_MODES::TRANSLATION_X: printf("\nTRANSLATION_X\n");
                         planner->get_path()->front().vel_control.Print("");
                         break;
                     default: printf("\nUndefined mode\n");
                 }
             }
             // check if the mode is changed
             if(planner->get_path()->front().modes!=currentMode){
                 // instructing the robot in stage to move according to the control
                 //position->SetVelocity(planner->get_path()->front().vel_control);
                 currentMode = planner->get_path()->front().modes;
                 switch (planner->get_path()->front().modes){
                     case MOTION_MODES::ROTATION_Z: position->SetXSpeed(0);
                         position->SetTurnSpeed(planner->get_path()->front().vel_control.a);
                         break;
                     case MOTION_MODES::TRANSLATION_X: position->SetTurnSpeed(0);
                         position->SetXSpeed(planner->get_path()->front().vel_control.x);
                         break;
                     default: printf("\nUndefined mode\n");
                 }
             }
             if (planner->get_path()->front().computed_desPose){
                 // Move until the stage robot has reached the desired orientation up to a tolerance
                 if(std::fabs(planner->get_path()->front().des_pose.a - position->GetPose().a) < rad_tol){
                     planner->get_path()->pop();
                 }
             } else{
                 // Move until the desired motion time is reached up to a tolerance
                 if (verbose){
                     std::cout<<"\n Motion end time is : "<<planner->get_path()->front().motion_end_time<<std::endl;
                     std::cout<<"\n Sim time is : "<<world->SimTimeNow()/1000000.0<<std::endl;
                 }
                 if(std::fabs(planner->get_path()->front().motion_end_time-world->SimTimeNow()/1000000.0)<time_tol){
                     planner->get_path()->pop();
                 }
             }
         } else {
             if(verbose)
                 std::cout<<"\n path generated \n";
             planner->set_startPose(position->GetPose());
             try {
                 // try to generate a new path
                 planner->generate_path(world->SimTimeNow()/1000000.0);
             }
             catch (const char *error){
                 std::cerr<<error<<std::endl;
             }


         }

     }

 }


 void robot::build_map(const Stg::ModelRanger::Sensor& laser) {
   /**
    * The robot builds an occupancy map of the domain using the measurements
    * from the laser range sensor data.
    * \param laser : an object containing the laser range data
    */

   


 }