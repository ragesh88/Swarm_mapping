//
// Created by Ragesh on 4/3/18.
//

#include "planner/base_planner.h"

using namespace myPlanner;

void base_planner::generate_path(double start_time) {
  via_points point;

  set_planStartTime(start_time);

  // first point in the path
  point.modes = MOTION_MODES::ROTATION_Z;
  point.motion_end_time = planStartTime + planTime / 2;
  point.vel_control.a = robotTwist.a;
  point.des_pose = startPose + Stg::Pose(0.0, 0.0, 0.0, M_PI / 4);
  point.computed_desPose = true;

  path.push(point); //  pushed the first point

  // second point in the path
  point.modes = MOTION_MODES::TRANSLATION_X;
  point.motion_end_time += planTime / 2;
  point.vel_control.x = robotTwist.x;
  point.des_pose = Stg::Pose(0.0, 0.0, 0.0, 0.0);
  point.computed_desPose = false;

  path.push(point);

  // Print the signature of the function in gcc or g++ compiler
  //std::cout<<__PRETTY_FUNCTION__ << ": " <<std::endl ;



}

/***************************************************************************************************/

Stg::meters_t levyWalk_planner::generate_levy_dist() {
  /**
   * Generate the length from the distribution
   */
  //uniform random number generator
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  // generate a uniform random number between 0 and 1
  auto u = distribution(generator);

  if (alpha == 0) {
    throw "\nLevy alpha should be non zero\n";
  }

  double speed = std::sqrt(get_velocity()->x * get_velocity()->x +
      get_velocity()->y * get_velocity()->y +
      get_velocity()->z * get_velocity()->z);

  if (speed == 0) {
    throw "\nSpeed should be non zero\n";
  }

  return levy_min * pow((1 - u), -1 / alpha); // levy walk distance


}

Stg::radians_t levyWalk_planner::generate_random_direction() {
  /// Generates a random angle between min_angle and max_angle
  //uniform random number generator
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<double> distribution(min_ang, max_ang);
  // generate a uniform random number between min and max
  return distribution(generator);
}

void levyWalk_planner::generate_path(double start_time, occupancy_grid::occupancyGrid2D<double, int>*) {
  /**
   * The function generates a path for the robot to perform levy walk in an unbounded domain
   */

  via_points point;

  set_planStartTime(start_time);


  // computing first point in the path
  point.modes = MOTION_MODES::TRANSLATION_X;
  auto speed = std::fabs(get_velocity()->x);
  if (speed == 0) {
    throw "The speed can not be zero";
  }
  point.motion_end_time = generate_levy_dist() / speed + start_time;
  point.vel_control = Stg::Velocity(get_velocity()->x, get_velocity()->y, get_velocity()->z, 0);
  point.computed_desPose = false;

  path.push(point); // pushing the first point to the path


  // computing second point in the path
  point.modes = MOTION_MODES::ROTATION_Z;
  auto w = std::fabs(get_velocity()->a); // omega in magnitude
  double rotate_time = 0;
  const Stg::radians_t &des_dir = point.des_pose.a; // ref to the desired direction
  const Stg::radians_t &cur_dir = get_startPose()->a; // ref to the current direction
  point.des_pose = Stg::Pose(get_startPose()->x, get_startPose()->y, get_startPose()->z, generate_random_direction());

  // find the smallest angle to rotate, to find the right \omega
  // case 1 : when both of them are ++ or --
  if (des_dir * cur_dir >= 0) {
    // time to complete the motion
    rotate_time = std::fabs(des_dir - cur_dir) / std::fabs(w);
    if (des_dir > cur_dir) {
      // rotate in anticlockwise direction(left turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, std::fabs(w));
    } else {
      // rotate in clockwise direction(right turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, -std::fabs(w));
    }
  }
  // case 2 : when cur_dir is positive and des_dir is negative
  if (cur_dir > 0 && des_dir < 0) {
    // find the smallest angle between the current and the desired directions
    if (std::fabs(cur_dir - des_dir) < std::fabs(M_PI - cur_dir) + std::fabs(-M_PI - des_dir)) {
      // rotate in clockwise direction(right turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, -std::fabs(w));
      // time to complete the motion
      rotate_time = std::fabs(cur_dir - des_dir) / std::fabs(w);
    } else {
      // rotate in anticlockwise direction(left turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, std::fabs(w));
      // time to complete the motion
      rotate_time = (std::fabs(M_PI - cur_dir) + std::fabs(-M_PI - des_dir)) / std::fabs(w);
    }
  }
  // case 3 : when cur_dir is negative and des_dir is positive
  if (cur_dir < 0 && des_dir > 0) {
    // find the smallest angle between the current and desired directions
    if (std::fabs(cur_dir - des_dir) < std::fabs(-M_PI - cur_dir) + std::fabs(M_PI - des_dir)) {
      // rotate in anticlockwise direction(left turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, std::fabs(w));
      // time to complete the motion
      rotate_time = std::fabs(cur_dir - des_dir) / std::fabs(w);
    } else {
      // rotate in clockwise direction(right turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, -std::fabs(w));
      // time to complete the motion
      rotate_time = (std::fabs(-M_PI - cur_dir) + std::fabs(M_PI - des_dir)) / std::fabs(w);
    }
  }
  point.motion_end_time+=rotate_time; //TODO : uncomment this after debugging
  point.computed_desPose = true;


  path.push(point); // pushing the second point to the path //TODO : uncomment this after debugging

  // computing the third point in the path
  point.modes = MOTION_MODES::TRANSLATION_X;
  point.motion_end_time += generate_levy_dist() / speed;
  point.vel_control = Stg::Velocity(get_velocity()->x, get_velocity()->y, get_velocity()->z, 0);
  point.computed_desPose = false;

  path.push(point); // pushing the third point to the path


}

/***************************************************************************************************/

meters MI_levyWalk_planner::generate_levy_dist()
{
  /**
   * Generate the length from the distribution
   */
  //uniform random number generator
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  // generate a uniform random number between 0 and 1
  auto u = distribution(generator);

  if (alpha == 0) {
    throw "\nLevy alpha should be non zero\n";
  }

  double speed = std::sqrt(get_velocity()->x * get_velocity()->x +
      get_velocity()->y * get_velocity()->y +
      get_velocity()->z * get_velocity()->z);

  if (speed == 0) {
    throw "\nSpeed should be non zero\n";
  }

  return levy_min * pow((1 - u), -1 / alpha); // levy walk distance
}


void MI_levyWalk_planner::generate_dir_via_point(const Stg::Pose &start_pos,
                                                 const myPlanner::meters& plan_len,
                                                 std::queue<Stg::Pose> &dir_via_point)
/**
 *
 * @param start_pos : The start pose of the path whose via points need to be created
 * @param plan_dis : The length of the path
 * @param dir_via_point : The set of via points as a queue
 */
{
  meters dist_covered{0};
  // check if the queue is empty
  assert(dir_via_point.empty()); // abort if the queue is non empty
  // push the first point into the queue
  dir_via_point.emplace(start_pos);

  while(dist_covered + dist_btw_path_via < plan_len){
    // computing the next via point and pushing it to the queue
    dir_via_point.emplace(Stg::Pose{dir_via_point.back().x + dist_btw_path_via*cos(dir_via_point.back().a),
                                    dir_via_point.back().y + dist_btw_path_via*sin(dir_via_point.back().a),
                                    dir_via_point.back().z,
                                    dir_via_point.back().a});
    dist_covered += dist_btw_path_via;
  }
}


double MI_levyWalk_planner::compute_beam_CSQMI(occupancy_grid::occupancyGrid2D<double, int>* map,
                                            double px,
                                            double py,
                                            double p_theta)
/**
 *
 * @param map : a pointer to the map object for ray tracing operations
 * @param px : the x coordinate of the beam base in global frame
 * @param py : the y coordinate of the beam base in global frame
 * @param p_theta : the orientation of the beam in global frame
 * @return : the Cauchy Schwarz Mutual Information for a single beam
 */
{
  // finding the grids and its occupancy probability range traced by the beam
  std::map<std::vector<int>, std::pair<double , double>, occupancy_grid::vec_path_comp_class<int>> traced_grids;
  map->ray_trace_path(px, py, p_theta, fsm.z_max, traced_grids);

  int delta = 3; // the number of significant cells to compute the double summation

  // compute MI for the ray traced grid cells by the beam

  // pre-compute certain terms to speed up the computation

  // number of beam traced grid cells
  auto no_traced_grid = traced_grids.size();
  // store the probability that the i^{cell} is occupied,
  // by convection prob_0_cell_occ means non of the cells are occupied
  std::vector<double> prob_i_cell_occ(no_traced_grid+1,1);
  std::vector<double> occ_unocc(no_traced_grid+1);
  std::vector<double> w(no_traced_grid+1);

  // computing the probability that i^{th} cell is the first free cell
  for(int i=0; i<prob_i_cell_occ.size(); i++){
    if (i == 0){ // probability that all cells are unoccupied
      for(const auto& it : traced_grids){
        prob_i_cell_occ[i] *= (1-it.second.first);
      }
    } else{
      // compute the probability that i^{th} cell is the first free cell
      int j = 1; // cell count
      for(const auto& it : traced_grids){
        if(j<i){ // all i-1 cells are free
          prob_i_cell_occ[i] *= it.second.first;
        }else{
          // the i^{th} cell is occupied
          prob_i_cell_occ[i] *= (1-it.second.first);
          break;
        }
        j++;
      }
    }
  }

  // pre-computing the vector occu_unocc
  int cell_count = 1;
  for(const auto& it : traced_grids){
    occ_unocc[cell_count] =  (it.second.first*it.second.first + (1-it.second.first)*(1-it.second.first));
    cell_count++;
  }

  // pre-computing the vector w
  for (int i=0; i<w.size(); i++){
    w[i] = std::accumulate(occ_unocc.begin() + i +1, occ_unocc.end(),
                           prob_i_cell_occ[i]*prob_i_cell_occ[i], std::multiplies<>());
  }


  // compute the first term in the MI  computation

  // compute the first term
  double MI_term1;





  return 0;

}

double MI_levyWalk_planner::compute_beam_KLDMI(occupancy_grid::occupancyGrid2D<double, int>* map,
                                               double px,
                                               double py,
                                               double p_theta)
/**
 *
 * @param map : a pointer to the map object for ray tracing operations
 * @param px : the x coordinate of the beam base in global frame
 * @param py : the y coordinate of the beam base in global frame
 * @param p_theta : the orientation of the beam in global frame
 * @return : the KL Diverence Mutual Information for a single beam
 */
{
  // finding the grids and its occupancy probability range traced by the beam
  std::map<std::vector<int>, std::pair<double , double>, occupancy_grid::vec_path_comp_class<int>> traced_grids;
  map->ray_trace_path(px, py, p_theta, fsm.z_max, traced_grids);

  // compute MI for the ray traced grid cells by the beam

  // pre-compute certain terms to speed up the computation

  // number of beam traced grid cells
  auto no_traced_grid = traced_grids.size();
  // store the probability that the i^{cell} is occupied,
  // by convection prob_0_cell_occ means non of the cells are occupied
  std::vector<double> prob_i_cell_occ(no_traced_grid+1,1);


  // computing the probability that i^{th} cell is the first free cell
  for(int i=0; i<prob_i_cell_occ.size(); i++){
    if (i == 0){ // probability that all cells are unoccupied
      for(const auto& it : traced_grids){
        prob_i_cell_occ[i] *= (1-it.second.first);
      }
    } else{
      // compute the probability that i^{th} cell is the first free cell
      int j = 1; // cell count
      for(const auto& it : traced_grids){
        if(j<i){ // all i-1 cells are free
          prob_i_cell_occ[i] *= it.second.first;
        }else{
          // the i^{th} cell is occupied
          prob_i_cell_occ[i] *= (1-it.second.first);
          break;
        }
        j++;
      }
    }
  }

  // pre-compute probability of a measurement range given the sequence of cell P(z_i| e_k) as a map
  std::vector<std::vector<int>> prob_range_seq(no_traced_grid+1);

  // TODO compute KLDMI
  return 0;



}


double MI_levyWalk_planner::compute_beam_Entropy(occupancy_grid::occupancyGrid2D<double, int>* map,
                                               double px,
                                               double py,
                                               double p_theta)
/**
 *
 * @param map : a pointer to the map object for ray tracing operations
 * @param px : the x coordinate of the beam base in global frame
 * @param py : the y coordinate of the beam base in global frame
 * @param p_theta : the orientation of the beam in global frame
 * @return : entropy of the cells traced by a single beam
 */
{
  // finding the grids and its occupancy probability range traced by the beam
  std::map<std::vector<int>, std::pair<double , double>, occupancy_grid::vec_path_comp_class<int>> traced_grids;
  map->ray_trace_path(px, py, p_theta, fsm.z_max, traced_grids);

  double entropy=0;

  for (const auto& it : traced_grids){
    entropy += (-it.second.first*std::log(it.second.first) - (1-it.second.first)*std::log(1-it.second.first));
  }
  return entropy;
}


void MI_levyWalk_planner::generate_path(double start_time, occupancy_grid::occupancyGrid2D<double, int>* map)
/**
 * The virtual method of generating the direction the robot should move to increase information gain
 * @param start_time : the start time when the robot start to plan
 * @param map : the pointer to the map object for ray tracing
 */
{
  // generate the levy distance that the robot should move
  meters levy_dis = generate_levy_dist();

  // make sure the velocity is non zero
  assert(get_velocity()->x);

  double levy_travel_time = levy_dis/std::fabs(get_velocity()->x);

  const Stg::Pose* curPose = get_startPose();


  std::map<double, std::vector<radians>> dir_MI; // store the compute MI and associated direction

  // path set generation for the robots initialize with current orientation of the robot(global coordinates)
  std::vector<radians> path_directions{curPose->a};
  // adding directions to the left(anti-clockwise) of the robot(global coordinates)
  for(int i = 0; i < (no_path_each_side-1); i++){
    radians new_dir = (i+1)*max_ang/no_path_each_side;
    // adjust the angles to [-M_PI M_PI] when it goes above 180 degree
    if (curPose->a + new_dir > M_PI){
      path_directions.push_back(curPose->a + new_dir -  (2*M_PI));
    } else {
      path_directions.push_back(curPose->a+new_dir);
    }
  }

  // adding directions to the right(clockwise) of robot (global coordinates)
  for(int i=0; i < (no_path_each_side-1); i++){
    radians new_dir = (i+1)*min_ang/no_path_each_side;
    // adjust the angles the [-M_PI M_PI] when it below -180 degree
    if (curPose->a - new_dir < -M_PI){
      path_directions.push_back(curPose->a + new_dir + 2*M_PI);
    }else{
      path_directions.push_back(curPose->a - new_dir);
    }
  }

  // generate via points for each path dir in path_directions vector
  // and compute mutual information for each path and store in the map dir_MI
  for(const auto& path_dir : path_directions){
    double curr_MI=0.0; // store the mutual information value of the current path
    std::queue<Stg::Pose> dir_via_point; // variable to store via point
    Stg::Pose startPose{curPose->x, curPose->y, curPose->z, curPose->a + path_dir}; // start pose of the path

    // generate the via points for the path with this start pose
    generate_dir_via_point(startPose, levy_dis, dir_via_point);

    // iterate through various via points in the path
    while(!dir_via_point.empty()){
      Stg::Pose& via_point = dir_via_point.front(); // reference to a via point

      // iterate through various beams for that via point
      // compute the mutual information for each beam and add it to the MI of the path
      for(const auto& beam : beam_dir){
        switch (reward){
          case KLDMI: curr_MI += compute_beam_KLDMI(map, via_point.x, via_point.y, via_point.a + beam);
            break;
          case CSQMI: curr_MI += compute_beam_CSQMI(map, via_point.x, via_point.y, via_point.a + beam);
            break;
          case ENTROPY: curr_MI += compute_beam_Entropy(map, via_point.x, via_point.y, via_point.a + beam);
        }
      }

      dir_via_point.pop();

    }

    // the mutual information value and the directions with this value in to the map data structure

    // check if this mutual information value already exist in the map
    auto loc = dir_MI.find(curr_MI); // location of the key curr_MI
    if( loc != dir_MI.end()){
      loc->second.push_back(path_dir); // update vector if the key is already present
    } else { // insert a key value pair in to the map

      dir_MI.emplace(curr_MI, std::vector<radians>{path_dir});

      }

  }

  // find the path directions with maximum mutual information

  std::vector<radians>& max_MI_dirs = dir_MI.rbegin()->second;

  radians des_dir{2*M_PI};

  // pick the direction with minimum turning, if there multiple ones are present
  if(max_MI_dirs.size() > 1){
    for (const auto& dir : max_MI_dirs){
      if (des_dir < std::fabs(dir - curPose->a)){
        des_dir = dir;
      }
    }
  } else {
    des_dir = max_MI_dirs[0];
  }

  // Pushing the translation motion and rotation motion in to the path

  via_points point;

  // first point in the path
  point.modes = MOTION_MODES::ROTATION_Z;
  point.des_pose = Stg::Pose(0.0, 0.0, 0.0, des_dir);
  point.computed_desPose = false;

  const Stg::radians_t &cur_dir = get_startPose()->a; // ref to the current direction
  const auto& w = std::fabs(get_velocity()->a); // omega in magnitude
  double rotate_time = 0;


  // find the smallest angle to rotate, to find the right \omega
  // case 1 : when both of them are ++ or --
  if (des_dir * cur_dir >= 0) {
    // time to complete the motion
    rotate_time = std::fabs(des_dir - cur_dir) / std::fabs(w);
    if (des_dir > cur_dir) {
      // rotate in anticlockwise direction(left turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, std::fabs(w));
    } else {
      // rotate in clockwise direction(right turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, -std::fabs(w));
    }
  }
  // case 2 : when cur_dir is positive and des_dir is negative
  if (cur_dir > 0 && des_dir < 0) {
    // find the smallest angle between the current and the desired directions
    if (std::fabs(cur_dir - des_dir) < std::fabs(M_PI - cur_dir) + std::fabs(-M_PI - des_dir)) {
      // rotate in clockwise direction(right turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, -std::fabs(w));
      // time to complete the motion
      rotate_time = std::fabs(cur_dir - des_dir) / std::fabs(w);
    } else {
      // rotate in anticlockwise direction(left turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, std::fabs(w));
      // time to complete the motion
      rotate_time = (std::fabs(M_PI - cur_dir) + std::fabs(-M_PI - des_dir)) / std::fabs(w);
    }
  }
  // case 3 : when cur_dir is negative and des_dir is positive
  if (cur_dir < 0 && des_dir > 0) {
    // find the smallest angle between the current and desired directions
    if (std::fabs(cur_dir - des_dir) < std::fabs(-M_PI - cur_dir) + std::fabs(M_PI - des_dir)) {
      // rotate in anticlockwise direction(left turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, std::fabs(w));
      // time to complete the motion
      rotate_time = std::fabs(cur_dir - des_dir) / std::fabs(w);
    } else {
      // rotate in clockwise direction(right turn)
      point.vel_control = Stg::Velocity(0.0, 0.0, 0.0, -std::fabs(w));
      // time to complete the motion
      rotate_time = (std::fabs(-M_PI - cur_dir) + std::fabs(M_PI - des_dir)) / std::fabs(w);
    }
  }

  point.motion_end_time = rotate_time + start_time;

  path.push(point); //  pushed the rotation point

  // second point in the path
  point.modes = MOTION_MODES::TRANSLATION_X;
  point.motion_end_time += levy_travel_time;
  point.vel_control.x = get_velocity()->x;
  point.des_pose = Stg::Pose(0.0, 0.0, 0.0, 0.0);
  point.computed_desPose = false;

  path.push(point); // pushed the translation point


}

