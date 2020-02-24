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
int robot::gen_id = 0;

// Defining the static member vector containing the robot object pointers
std::vector<myRobot::robot*>myRobot::robot::swarm{nullptr};

// Defining the static member functions
void robot::swarm_update(myRobot::robot* member)
/**
 * The static member function for the class robot
 * to update the members of the swarm *
 * @param member : the pointer to the member of the swarm
 */
{
  if (member->get_robot_id() == 1) {
    myRobot::robot::swarm[0] = member;
  } else {
    if (myRobot::robot::swarm.size() < member->get_robot_id()){
      myRobot::robot::swarm.resize(static_cast<long unsigned>(member->get_robot_id()));
    }
    myRobot::robot::swarm[member->get_robot_id()-1] = member;
  }
}

void robot::move() {
  /// The function moves the robot according to the planner object

  // Tolerance to check the various angle and time conditions
  const double rad_tol = 2 * M_PI / 180;
  const double time_tol = 0.1;
  static MOTION_MODES currentMode = MOTION_MODES::START;
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
    if (verbose && 0)
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
    currentMode = MOTION_MODES::START;
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
    if (!planner->get_path()->empty()) {
      if (verbose)
        std::cout << "\n path exist \n";
      if (verbose) {
        switch (planner->get_path()->front().modes) {
          case MOTION_MODES::ROTATION_Z: printf("\nRotation\n");
            planner->get_path()->front().vel_control.Print("");
            position->GetVelocity().Print("Actual ");
            std::cout << "\n The current angle(deg): " << position->GetPose().a * (180 / M_PI) << std::endl;
            std::cout << "\n The desired angle(deg):" << planner->get_path()->front().des_pose.a * (180 / M_PI)
                      << std::endl;
            break;
          case MOTION_MODES::TRANSLATION_X: printf("\nTRANSLATION_X\n");
            planner->get_path()->front().vel_control.Print("");
            break;
          default: printf("\nUndefined mode\n");
        }
      }
      // check if the mode is changed
      if (planner->get_path()->front().modes != currentMode) {
        // instructing the robot in stage to move according to the control
        //position->SetVelocity(planner->get_path()->front().vel_control);
        currentMode = planner->get_path()->front().modes;
        switch (planner->get_path()->front().modes) {
          case MOTION_MODES::ROTATION_Z: position->SetXSpeed(0);
            position->SetTurnSpeed(planner->get_path()->front().vel_control.a);
            break;
          case MOTION_MODES::TRANSLATION_X: position->SetTurnSpeed(0);
            position->SetXSpeed(planner->get_path()->front().vel_control.x);
            break;
          default: printf("\nUndefined mode\n");
        }
      }
      if (planner->get_path()->front().computed_desPose) {
        // Move until the stage robot has reached the desired orientation up to a tolerance
        if ((std::fabs(planner->get_path()->front().des_pose.a - position->GetPose().a) < rad_tol &&
            planner->get_path()->front().modes==MOTION_MODES::ROTATION_Z) ||
            (std::fabs(planner->get_path()->front().des_pose.x - position->GetPose().x) +
                std::fabs(planner->get_path()->front().des_pose.y - position->GetPose().y) < 2*rad_tol &&
                planner->get_path()->front().modes==MOTION_MODES::TRANSLATION_X)) {
          planner->get_path()->pop();
        }
      } else {
        // Move until the desired motion time is reached up to a tolerance
        if (verbose) {
          std::cout << "\n Motion end time is : " << planner->get_path()->front().motion_end_time << std::endl;
          std::cout << "\n Sim time is : " << world->SimTimeNow() / 1000000.0 << std::endl;
        }
        if (std::fabs(planner->get_path()->front().motion_end_time < (world->SimTimeNow() / 1000000.0))) {
          planner->get_path()->pop();
        }
      }
    } else {
      if (verbose)
        std::cout << "\n path generated \n";
      planner->set_startPose(position->GetPose());
      try {
        // try to generate a new path
        if (planner->is_using_map()){ // call methods based if the planner uses map or not
          planner->generate_path(world->SimTimeNow() / 1000000.0, occ_grid_map);
        } else{
          planner->generate_path(world->SimTimeNow() / 1000000.0);
          //std::cout<<"\nnot using map\n";
          }

      }
      catch (const char *error) {
        std::cerr << error << std::endl;
      }

    }

  }

}

void robot::build_map() {
  /**
   * The robot builds an occupancy map of the domain using the measurements
   * from the laser range sensor data.
   */
  const bool verbose_local = false; // Turn on actions for debugging
  const auto &laserSensor = laser->GetSensors()[0];
  //std::cout<<"\n Hello";
  //std::cout<<"\n angle noise : "<<laserSensor.angle_noise;
  //std::cout<<"\n range noise : "<<laserSensor.range_noise;
  //std::cout<<"\n range noise const : "<<laserSensor.range_noise_const;
  //laserSensor.pose.Print("Sensor pose ");

  const Stg::Pose base_pose = position->GetPose();
  // uncomment the line below for debugging
  //printf("\n The measurement at a new pose \n");
  //base_pose.Print("base pose ");
  //std::cout<<std::endl;
  const int no_of_rays = 10;
  const int ray_incre = laserSensor.sample_count/no_of_rays; // interval in choosing the laser rays
  // Iterate through each ray in the interval ray_incre
  for (int i = 0; i < laserSensor.sample_count; i += ray_incre) {

    // Get the grid cell coordinate for which the ray passed through
    std::map<double, cv::Vec<int, 2>> passed_grids_ranges;
    occ_grid_map->ray_trace_all(laserSensor.pose.x + base_pose.x, laserSensor.pose.y + base_pose.y,
                                laserSensor.bearings[i] + base_pose.a, laserSensor.ranges[i],
                                passed_grids_ranges);
    if (verbose&&0) {
      for (auto it = passed_grids_ranges.begin(); it != passed_grids_ranges.end(); ++it) {
        if (i == 0) { // for debugging
          printf("\n (%d,%d) is at a distance of %f from the first point", it->second[0], it->second[1], it->first);
        }
      }
    }

    // compute the probability of occupancy for each grid cell using inverse sensor model for the ray
    std::list<std::pair<cv::Vec<int, 2>, double>> occ_probability;
    occupancy_grid::probability_map_given_measurement_pose(laserSensor, i, passed_grids_ranges, occ_probability);

    // compute the log odds of the probability for better numerical accuracy
    //std::list<std::pair<cv::Vec<int,2>,double>> occ_logOdds;
    //occupancy_grid::log_odds_map_given_measurement_pose(laserSensor, i, passed_grids_ranges, occ_logOdds);

    // update the map using the probability value scaled between 0 - 255
    if (verbose_local) {
      if (!occ_probability.size())
        std::cout << "The size of probability measurements : " << occ_probability.size() << std::endl;
      std::cout << "The size of passed grid ranges : " << passed_grids_ranges.size() << std::endl;
    }

    for (auto it = occ_probability.begin(); it != occ_probability.end(); ++it) {
      // In the case of combining probability values for occupancyGrid2D objects
      double v = static_cast<double>(occ_grid_map->get(it->first[0], it->first[1])) /
                  static_cast<double>(occupancyGrid2D<double, int>::OCCUPIED);
      //occ_grid_map->set(it->first[0], it->first[1], static_cast<uint8_t>(occ_grid_map->OCCUPIED*(0.5*(it->second))+0.5*v));
      // TODO delete the line below after debugging
//      if (static_cast<uint8_t>(occupancyGrid2D<double, int>::OCCUPIED * ((it->second) * v))>0){
//        std::cout<<"the value of it->second is : "<<(it->second)<<std::endl;
//        std::cout<<"the value of v is : "<<v<<std::endl;
//        std::cout<<"the value of OCCUPIED is : "<<static_cast<double>(occupancyGrid2D<double, int>::OCCUPIED)<<std::endl;
//        std::cout<<"the value of the point is : "<<static_cast<double>(occ_grid_map->get(it->first[0], it->first[1]))<<std::endl;
//        std::cout<<"the value computed is non zero which is : "<<(it->second * v)<<std::endl;
//      } else{
//        std::cout<<"the value when computed to be zero is "<<static_cast<uint8_t>(occupancyGrid2D<double, int>::OCCUPIED * ((it->second) * v))<<std::endl;
//      }

      occ_grid_map->set(it->first[0],
                        it->first[1],
                        static_cast<uint8_t>(occupancyGrid2D<double, int>::OCCUPIED * ((it->second) * v)));

      // In the case of combining log odds values for Prob_occupancyGrid objects
      //const int v = occ_grid_map->get(it->first[0], it->first[1]);
      //occ_grid_map->set(it->first[0], it->first[1], static_cast<int>(v + it->second));
      if (i == 10) {
        //printf("\n the probability of ray %d : %d", i, static_cast<uint8_t>(occ_grid_map->OCCUPIED*(it->second)));
      }

    }

  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void merger(cv::Mat &map1, const cv::Mat &map2)
/**
 * The function combines map1 and map2 to store it in map1
 * @param map1 : occupancy grid map of robot 1
 * @param map2 : occupancy grid map of robot 2
 */
{
  // power protocol for probability
  cv::Mat tempMap1, tempMap2;
  map1.convertTo(tempMap1, CV_32F);
  map2.convertTo(tempMap2, CV_32F);
  tempMap1 = tempMap1.mul(tempMap2);
  cv::sqrt(tempMap1, tempMap2);
  tempMap2.convertTo(map1, CV_8U);

  // arithmetic mean protocol for log odds
  //cv::addWeighted(map1,0.5, map2, 0.5, 0, map1);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void alberto_merger(cv:: Mat &img1, const cv::Mat &img2)
/**
 * This function merges the maps map1 and map2 according to
 * strategy developed by Alberto in "A real-time map merging
 * strategy for robust collaborative reconstruction of unknown environments"
 * @param map1 : occupancy grid map of robot 1
 * @param map2 : occupancy grid map of robot 2
 */
{
    bool pub_merged_images = true;
    double tx = 0.0;
    double ty = 0.0;
    double angT_Q = 0.0;// + (3.5*M_PI)/180;
    double sang = sin(angT_Q);
    double cang = cos(angT_Q);
//    std::cout << "       -> Transformed: (" << tx << "," << ty << "," << angT_Q << ") " << std::endl;

    int row_min=0, row_max=0, col_min=0, col_max=0;
    int row1=(int)img1.rows, row2=(int)img2.rows, col1=(int)img1.cols, col2=(int)img2.cols;
    int height = row2;
    int width = col2;
    int height_min = 0;
    int width_min  = 0;
    int height_max = row2;
    int width_max  = col2;

    int i,j;
    int x,y;
    for(i=0; i<row1; i++ ){
        for(j=0; j<col1; j++ ){
            x = j*cang - i*sang + tx;
            y = j*sang + i*cang + ty;
            //std::cout << "i " << i << " y j " << j << " x " << x << " y y " << y << std::endl;
            if(x < col_min){
                col_min = x;
            }else if(x > col_max){
                col_max = x;
            }
            if(y < row_min){
                row_min = y;
            }else if(y > row_max){
                row_max = y;
            }
        }
    }

    //int x = 106*cos(angT_Q) - 205*sin(angT_Q) + tx;
    //int y = 106*sin(angT_Q) + 205*cos(angT_Q) + ty;
    //std::cout << "EXTREMES " << x << " y " << y << std::endl;
//    std::cout << "       -> Lims: " << row_min << "," << row_max << " y " << col_min << "," << col_max << std::endl;


    if(row_min<0){	height_min = std::abs(row_min);	height+= height_min;	}
    if(row_max>row2){	height_max = (row_max-row2)+1;	height+= height_max;	};
    if(col_min<0){	width_min  = std::abs(col_min);	width+=  width_min;	};
    if(col_max>col2){	width_max  = (col_max-col2)+1;	width+=  width_max;	};

//    std::cout << "       -> Height: " << height_min << " y " << height_max <<  " -> Width: " <<
//    width_min << " y " << width_max << std::endl;
//    std::cout << "       -> Height: " << height << " H_min " << height_min <<  " -> Width: " <<
//    width << " W_min " << width_min << std::endl;

    cv::Mat img;
    if(pub_merged_images){	img = cv::Mat( height, width, CV_8U,
            cv::Scalar(occupancy_grid::occupancyGrid2D<double, int>::OCCUPIED) );
    }
    int8_t map_[height+1][width+1];
    std::fill(map_[0], map_[0] + (height+1)*(1+width), -1);
    int img_dat = 127;
    int img12_dat = 127;

//    wi_he.push_back(width_min);
//    wi_he.push_back(height_min);
//putting image 1 in img_merged

//    std::cout << "       -> Img1: " << img1.cols << "," << img1.rows << std::endl;
//    std::cout << "       -> Img2: " << img2.cols << "," << img2.rows << std::endl;
//    std::cout << "       -> Img: " <<  img.cols <<  "," << img.rows << std::endl;

    int rs =  sizeof map_ / sizeof map_[0]; // 2 rows
    int cs = sizeof map_[0] / sizeof(int8_t); // 5 cols

    std::vector<int8_t> data(height*width);
    std::fill (data.begin(),data.begin()+height*width,-1);
//    std::cout << "       -> putting image 1 in img_merged " << rs << " " << cs << std::endl;
    for(i=0; i<row1; i++ ){
        for(j=0; j<col1; j++ ){
            x = j*cang - i*sang + tx + width_min;
            y = j*sang + i*cang + ty + height_min;
            //std::cout << "       -> img1 (" << j << "," << i << ")" << " (" << x << "," << y << "): " << img1.at<uchar>(i,j)  << " " << x+y*width << std::endl;
            if(y>=height || x>=width) continue;

            img_dat = img1.at<uchar>(i,j);
            data.at(x+y*width) = (img_dat<10) ? 100: (img_dat>250) ? 0:-1;
            //map_[y][x] =  (img_dat<10) ? 100: (img_dat>250) ? 0:-1;
            if(pub_merged_images){	img.at<uchar>(y,x) = img_dat;	};
        }
    }

//Putting Image 2 in img_merged

//    std::cout << "       -> Putting Image 2 in img_merged "<< std::endl;
    for(i=0; i<row2; i++ ){
        for(j=0; j<col2; j++ ){
            x = j + width_min;
            y = i + height_min;
            img12_dat = img2.at<uchar>(i,j);
            img_dat = (img12_dat<10) ? 100: (img12_dat>250) ? 0:-1 ;

            switch ( img_dat ) {
                case 0:
                    if( /*map_[y][x]*/data.at(x+y*width) ==-1 ){
                        //map_[y][x] = img_dat;
                        data.at(x+y*width) = img_dat;
                        if(pub_merged_images){	img.at<uchar>(y,x) = img12_dat;	};
                    }
                    break;
                case 100:
                    if( /*map_[y][x]*/data.at(x+y*width) ==-1 ){
                        //map_[y][x] = img_dat;
                        data.at(x+y*width) = img_dat;
                        if(pub_merged_images){	img.at<uchar>(y,x) = img12_dat;	};
                    }else if( /*map_[y][x]*/data.at(x+y*width) ==0 ){
                        //map_[y][x] = img_dat;
                        data.at(x+y*width) = img_dat;
                        if(pub_merged_images){	img.at<uchar>(y,x) = img12_dat;	};
                    }
                    break;
                default:
                    break;
            }
        }
    }

//    std::cout << "       -> Building OccupancyGrid Object "<< std::endl;

    // copy the merged map to map1
    img.copyTo(img1);

    /*for(y=0; y<height; y++){
        for(x=0; x<width; x++){
            data.push_back(map_[y][x]);
        }
    }*/
//    map.data  = data;
//    map.info.width  = width;
//    map.info.height  = height;
//    map.info.origin.position.x  = -((map.info.resolution/2.f)+(width/2)*map.info.resolution);
//    map.info.origin.position.y  = -((map.info.resolution/2.f)+(height/2)*map.info.resolution);


//    if(pub_merged_images){
//        std::stringstream ss;
//        ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/merged_images1.jpg";
//        cv::imwrite( ss.str(), img1 );
//        ss.str("");
//        ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/merged_images2.jpg";
//        cv::imwrite( ss.str(), img2 );
//        ss.str("");
//        ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/merged_img" << merg << ".jpg";
//        cv::imwrite( ss.str(), img );
//        merg++;
//        std::cout << "       -> Save Merged images: " << merg << std::endl;
//
//    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void robot::merge_map(const std::vector<myRobot::robot *> &swarm)
/**
 * The function merge the map between robots
 * @param swarm
 */

{
  // Check if robot found any other robot on its fiducial sensor
  if (!fiducial_sensor->GetFiducials().empty()) {
    const auto &fiducials = fiducial_sensor->GetFiducials(); // create a const reference to the sensor output vector
    // iterate through each fiducial in fiducials for merging the map with the robot found in the fidicual sensor
    for (auto fid : fiducials) {
      // encountering this robot for the first time
      if (fid.id > last_communication.size()) {
        last_communication.resize(static_cast<uint>(fid.id), 0.0);
      }
      // check if ample time has past since the map merger
      if (last_communication[fid.id - 1] + comm_delay < world->SimTimeNow() / 1000000.0) {
        // access the data of the robot in the swarm with fiducial id obtained from robot's fiducial sensor
        // and the merge map using the merger functions
        merger(occ_grid_map->og_, swarm[fid.id - 1]->occ_grid_map->og_);
        last_communication[fid.id - 1] = world->SimTimeNow() / 1000000.0; // update the communication time
      }

    }
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////

void robot::merge_map()
/**
 * The function merge the map between robots
 */

{
  // Check if robot found any other robot on its fiducial sensor
  if (!fiducial_sensor->GetFiducials().empty()) {
    const auto &fiducials = fiducial_sensor->GetFiducials(); // create a const reference to the sensor output vector
    // iterate through each fiducial in fiducials for merging the map with the robot found in the fidicual sensor
    for (auto fid : fiducials) {
      // encountering this robot for the first time
      if (fid.id > last_communication.size()) {
        last_communication.resize(static_cast<uint>(fid.id), 0.0);
      }
      // check if ample time has past since the map merger
      if (last_communication[fid.id - 1] + comm_delay < world->SimTimeNow() / 1000000.0) {
        // access the data of the robot in the swarm with fiducial id obtained from robot's fiducial sensor
        // and the merge map using the merger functions
        merger(occ_grid_map->og_, myRobot::robot::swarm[fid.id - 1]->occ_grid_map->og_);
        last_communication[fid.id - 1] = world->SimTimeNow() / 1000000.0; // update the communication time
      }

    }
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////


void robot::alberto_merge_map()
/**
 * The function merge the map between robots using alberto merger
 */

{
    // Check if robot found any other robot on its fiducial sensor
    if (!fiducial_sensor->GetFiducials().empty()) {
        const auto &fiducials = fiducial_sensor->GetFiducials(); // create a const reference to the sensor output vector
        // iterate through each fiducial in fiducials for merging the map with the robot found in the fidicual sensor
        for (auto fid : fiducials) {
            // encountering this robot for the first time
            if (fid.id > last_communication.size()) {
                last_communication.resize(static_cast<uint>(fid.id), 0.0);
            }
            // check if ample time has past since the map merger
            if (last_communication[fid.id - 1] + comm_delay < world->SimTimeNow() / 1000000.0) {
                // access the data of the robot in the swarm with fiducial id obtained from robot's fiducial sensor
                // and the merge map using the merger functions
                alberto_merger(occ_grid_map->og_, myRobot::robot::swarm[fid.id - 1]->occ_grid_map->og_);
                last_communication[fid.id - 1] = world->SimTimeNow() / 1000000.0; // update the communication time
            }

        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void robot::write_map(std::string path, std::string prefix)
/**
 * write the map stored in the robot as an image
 */
{
  std::string count = std::to_string(image_count);
  count = std::string(9 - count.length(), '0') + count;
  std::string filename;
  if (path.length() == 0){
     filename = img_path + prefix + "_" +robot_name + "_" + count + img_type;
  } else{
     filename = path + prefix + "_" +robot_name + "_" + count + img_type;
  }

  //std::cout<<"\n writing map as "<<filename<<std::endl;
  try {
    //auto start = clock();
    occ_grid_map->map_write(filename, myRobot::robot::gen_id);
    //auto stop = clock();
    //std::cout<<"\n the time for image writing is : "<<(stop-start)/double(CLOCKS_PER_SEC)*1000 <<std::endl;

    image_count++;
  } catch (std::runtime_error &ex) {
    fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void robot::write_map_txt(std::string path, std::string prefix)
/**
 * write the map stored in the robot to a text file
 */
{
  static unsigned long image_txt_count = 0;
  std::string count = std::to_string(image_txt_count);
  count = std::string(9 - count.length(), '0') + count;
  std::string filename;
  if (path.length() == 0){
    filename = img_path + prefix + "_" +robot_name + "_" + count + ".txt";
  } else{
    filename = path + prefix + "_" +robot_name + "_" + count + ".txt";
  }


  // Writing as an image is at least 20 times faster than writing to an text file
  occ_grid_map->map_txt_write(filename,myRobot::robot::gen_id, true);
  image_txt_count++;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void robot::add_map_entropy()
/**
 * the method add the entropy of the map at each instant to a list
 */
{
  double time = world->SimTimeNow()/1000000.0;
  map_entropy.emplace_back(std::pair<double, double>{time, occ_grid_map->compute_map_entropy()});
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void robot::add_map_coverage()
/**
 * method to add the percentage of map covered at each instant to a list
 */
{
  double time = world->SimTimeNow()/1000000.0;
  map_coverage.emplace_back(std::pair<double, double>{time, occ_grid_map->compute_map_coverage()});
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////


void robot::write_map_entropy(std::string path, std::string prefix)
/**
 * write entropy list stored in map_entropy to a text file with file name prefix+robot_{id}_entropy.txt
 * @param path : Path to store the text file ending with a /
 * @param prefix : any prefix to be added to the file name
 */
{
  if (map_entropy.empty()){
    std::cout<<"\nEntropy is not computed\n";
    return;
  }
  std::string filename{path + prefix + "robot_" + std::to_string(robot_id) + "_entropy.txt"};
  if(verbose){
    std::cout<<"\n Writing to : "<<filename<<std::endl;
  }

  // write the list to a text file

  std::ofstream f_out(filename);

  if(!f_out) {
    std::cout<<"File not opened \n";
    return;
  }

  for(const auto& it : map_entropy){
    f_out<<it.first<<" "<<it.second<<std::endl;
  }
  // closing the file stream
  f_out.close();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////


void robot::write_map_coverage(std::string path, std::string prefix)
/**
 * write data stored in the list map_coverage to a text file with file name prefix+robot_{id}_coverage.txt
 * @param path : Path to store the text file ending with a /
 * @param prefix : any prefix to be added to the file name
 */
{
  if (map_coverage.empty()){
    std::cout<<"\nCoverage is not computed\n";
    return;
  }
  std::string filename{path + prefix + "robot_" + std::to_string(robot_id) + "_coverage.txt"};
  if(verbose){
    std::cout<<"\n Writing to : "<<filename<<std::endl;
  }


  // write the list to a text file

  std::ofstream f_out(filename);

  if(!f_out) {
    std::cout<<"File not opened \n";
    return;
  }

  for(const auto& it : map_coverage){
    f_out<<it.first<<" "<<it.second<<std::endl;
  }
  // closing the file stream
  f_out.close();
}