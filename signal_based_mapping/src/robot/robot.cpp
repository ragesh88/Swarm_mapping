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

// Define the static variables
int robot::gen_id=0;


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

//////////////////////////////////////////////////////////////////////////////////////////////////////////
double reflectance_model( double grid_range,  Stg::meters_t range, double max_range,  double noise_sd)
/**
 * The function computes probability of occupancy of the grid cell corresponding to grid_range when the
 * ray was reflected.
 * @param grid_range
 * @param range
 * @param max_range
 * @param noise_sd
 * @return
 */
{
  const double start_prob = 0.01;
  const double end_prob = 0.5;
  const double max_prob = 0.8;
  const double init_slope = (end_prob - start_prob)/max_range;
  if(grid_range <= range-2*noise_sd - 0.02){
    return (init_slope * grid_range + start_prob);
  } else{
    return (max_prob);
  }
}

double non_reflectance_model( double grid_range,  double max_range,  double noise_sd)
/**
 * The function computes probability of occupancy of the grid cell corresponding to the grid_range when the
 * ray wasn't reflected
 * @param grid_range
 * @param max_range
 * @param noise_sd
 * @return
 */
{
  const double start_prob = 0.01;
  const double end_prob = 0.5;
  const double init_slope = (end_prob - start_prob)/(max_range-2*noise_sd);
  if(grid_range <= max_range-2*noise_sd){
    return (init_slope * grid_range + start_prob);
  } else{
    return (end_prob);
  }

}
typedef Stg::ModelRanger::Sensor LaserSensor;

void probability_map_given_measurement_pose(const LaserSensor& sensor,
                                            const int& ray_index,
                                            std::map<double,cv::Vec<int,2>>& passed_grids_ranges,
                                            std::list<std::pair<cv::Vec<int,2>,double>>& probability)
/**
 * The function computes the probability of the map cells whose coordinates are given as values in the map
 * object passed_grids_ranges.
 * @param sensor
 * @param ray_index
 * @param passed_grids_ranges
 * @param probability
 */
{

  // find if reflectance occurred with the ray.
  // reflectance occurred if the range of the ray less than
  // max range - 2*sqrt(range_noise_const)
  bool reflectance = false;
  const double noise_variance = std::sqrt(sensor.range_noise_const);
  if (sensor.ranges[ray_index] < (sensor.range.max - 2 * std::sqrt(noise_variance))) {
    reflectance = true;
  }

  if (reflectance) {
    for (auto it = passed_grids_ranges.begin(); it != passed_grids_ranges.end(); it++) {
      double prob = reflectance_model(it->first, sensor.ranges[ray_index], sensor.range.max, noise_variance);
      probability.push_back(std::pair<cv::Vec<int, 2>, double>(it->second, prob));

    }
  } else {
    for (auto it = passed_grids_ranges.begin(); it != passed_grids_ranges.end(); it++) {
      double prob = non_reflectance_model(it->first, sensor.range.max, noise_variance);
      probability.push_back(std::pair<cv::Vec<int, 2>, double>(it->second, prob));
    }

  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////


 void robot::build_map() {
   /**
    * The robot builds an occupancy map of the domain using the measurements
    * from the laser range sensor data.
    */
    const bool verbose_local = false;
    const auto &laserSensor = laser->GetSensors()[0];
    //std::cout<<"\n Hello";
    //std::cout<<"\n angle noise : "<<laserSensor.angle_noise;
    //std::cout<<"\n range noise : "<<laserSensor.range_noise;
    //std::cout<<"\n range noise const : "<<laserSensor.range_noise_const;
    //laserSensor.pose.Print("Sensor pose ");

    const Stg::Pose base_pose = position->GetPose();

    const int ray_incre = 5; // interval in choosing the laser rays
    // Iterate through each ray in the interval ray_incre
    for(int i=0; i<laserSensor.sample_count; i+=ray_incre){

        // Get the grid cell coordinate for which the ray passed through
        std::map<double,cv::Vec<int, 2>> passed_grids_ranges;
        occ_grid_map->ray_trace_all(laserSensor.pose.x + base_pose.x, laserSensor.pose.y + base_pose.y,
                                    laserSensor.bearings[i] + base_pose.a, laserSensor.ranges[i],
                                    passed_grids_ranges);
      if(verbose){
        for (auto it = passed_grids_ranges.begin(); it!=passed_grids_ranges.end(); ++it){
          if (i==0){ // for debugging
            printf("\n (%d,%d) is at a distance of %f from the first point", it->second[0], it->second[1], it->first);
          }
        }
      }

      // compute the probability of occupancy for each grid cell using inverse sensor model for the ray
      std::list<std::pair<cv::Vec<int,2>,double>> occ_probability;
      probability_map_given_measurement_pose(laserSensor, i, passed_grids_ranges, occ_probability);
      // update the map using the probability value scaled between 0 - 255
      if(verbose_local){
        if(!occ_probability.size())
        std::cout<<"The size of probability measurements : "<<occ_probability.size()<<std::endl;
        std::cout<<"The size of passed grid ranges : "<<passed_grids_ranges.size()<<std::endl;
      }
      for (auto it = occ_probability.begin(); it != occ_probability.end(); ++it){
        double v = static_cast<double>(occ_grid_map->get(it->first[0], it->first[1]));
        occ_grid_map->set(it->first[0], it->first[1], static_cast<uint8_t>(occ_grid_map->OCCUPIED*(0.5*(it->second))+0.5*v));
        if(i == 10){
          //printf("\n the probability of ray %d : %d", i, static_cast<uint8_t>(occ_grid_map->OCCUPIED*(it->second)));
        }

      }



    }


 }

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void merger(cv::Mat& map1, const cv::Mat& map2)
  /**
   * The function combines map1 and map2 to store it in map1
   * @param map1 : occupancy grid map of robot 1
   * @param map2 : occupancy grid map of robot 2
   */
{
  // power protocol
//  cv::Mat tempMap1, tempMap2;
//  map1.convertTo(tempMap1, CV_32F);
//  map2.convertTo(tempMap2, CV_32F);
//  cv::sqrt(tempMap1, map1);
//  cv::sqrt(tempMap2, tempMap2);
//  map1.mul(tempMap2);
//  map1 +=map2;
//  map1 = map1/2;
  cv::addWeighted(map1,0.5, map2, 0.5, 0, map1);

}


void robot::merge_map(const std::vector<myRobot::robot*>& swarm)
/**
 * The function merge the map between robots
 * @param swarm
 */

{
  // Check if robot found any other robot on its fiducial sensor
  if(fiducial_sensor->GetFiducials().size()>0){
    const auto& fiducials = fiducial_sensor->GetFiducials(); // create a const reference to the sensor output vector
    // iterate through each fiducial in fiducials for merging the map with the robot found in the fidicual sensor
    for (auto fid : fiducials){
      // encountering this robot for the first time
      if(fid.id > last_communication.size()){
        last_communication.resize(static_cast<uint>(fid.id),0.0);
      }
      // check if ample time has past since the map merger
      if(last_communication[fid.id-1]+comm_delay < world->SimTimeNow()/1000000.0){
        // access the data of the robot in the swarm with fiducial id obtained from robot's fiducial sensor
        // and the merge map using the merger functions
        merger(occ_grid_map->og_, swarm[fid.id-1]->occ_grid_map->og_);
        last_communication[fid.id-1] = world->SimTimeNow()/1000000.0; // update the communication time
      }


    }
  }
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////


void robot::write_map()
/**
 *
 */
{
  std::string count=std::to_string(image_count);
  count = std::string(9 - count.length(), '0') + count;
  std::string filename = img_path + robot_name  + "_" + count + img_type;
  try {
    cv::imwrite(filename.c_str(), occ_grid_map->og_);
    image_count++;
  }catch (std::runtime_error& ex) {
    fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());

  }

}