//
// Created by Ragesh on 3/20/18.
/**
 This is the header file for robot.cpp
 The file has the basic functionality for the controlling
 a robot in the stage simulator.

  */
// TODO : Cleaning up of the code is required. Remove unnecessary attributes and methods


#ifndef STAGE_CTRL_PLUGIN_ROBOT_H
#define STAGE_CTRL_PLUGIN_ROBOT_H

// C library header files
#include <ctime>

// C++ library header files


// local header files
#include "occupancy_grid/occupancyGrid.h"
#include "planner/base_planner.h"
//#include "occupancy_grid/forward_sensor_model.h"
#include "occupancy_grid/inverse_sensor_model.h"


// stage header file added for some functions
#include <Stage-4.3/stage.hh>

namespace myRobot {

/// success flag
const bool SUCCESS = true;
/// failed flag
const bool FAILED = false;

/**
 * The robot class handles and operation of the robot to be simulated in Stage Simulator. The class can be also
 * be used building codes for experiments
 */
class robot {

  // Constant Private attributes
  /// Path to save the robot map
  std::string img_path{"./robot"};
  /// The extension indicating the map image format
  const std::string img_type{".png"};

  /// the robot should not exchange information if the
  /// time between their consecutive encounter less than
  /// this value(in seconds)
  const double comm_delay = 10;
  /// counter for counting the number of images
  unsigned long image_count = 0;
  /// robot id
  uint robot_id;
  /// name of the robot
  std::string robot_name{"Robot"};
  /// current pose of the robot
  Stg::Pose current_pose{0, 0, 0, 0};
  /// current velocity of the robot
  Stg::Velocity current_velocity{0, 0, 0, 0};
  /// current linear speed
  double current_speed = 0.0;
  /// turn speed
  double turn_speed = 0.0;
  /// pose at previous time step of the robot
  Stg::Pose past_pose{0, 0, 0, 0};
  /// velocity at previous time step of the robot
  Stg::Velocity past_velocity{0, 0, 0, 0};
  /// time when past pose was recorded
  Stg::msec_t past_pose_time{0};
  /// the robot store their time of encounter with others(seconds)
  std::vector<double> last_communication;
  /// The pointer to the planner
  myPlanner::base_planner *planner{NULL};


  // variables to store entropy and coverage
  /// a list to store the entropy at various times
  std::list<std::pair<double, double>> map_entropy;
  /// a list to store the coverage at various times
  std::list<std::pair<double, double>> map_coverage;

 public:

  //Static variables
  /// Static variable to generate robot id
  static int gen_id;
  ///Static variable to store the pointers to all robots
  /// in the swarm
  static std::vector<robot *> swarm;

  // Stage simulator variables
  /// Access the position model of the Stage library
  Stg::ModelPosition *position{NULL};
  /// Access the laser sensor model of the Stage library
  Stg::ModelRanger *laser{NULL};
  /// Access the fiducial sensor model using the Stage library
  Stg::ModelFiducial *fiducial_sensor{NULL};
  /// Obstacle avoidance variable
  long int avoidCount{0};
  /// Obstacle avoidance variable
  long int randCount{0};
  /// a public attribute to store previous pose mostly for testing
  Stg::Pose previous_pose;
  /// The pointer to the world object
  Stg::World *world{NULL};
  /// To display output
  bool verbose = false;
  /// The pointer to occupancy grid map
  occupancy_grid::occupancyGrid2D<double, int> *occ_grid_map{NULL};
  //occupancy_grid::Prob_occupancyGrid2D<double, int>* occ_grid_map{NULL};


  // Static member function
  /// Static function to update the static variable swarm
  static void swarm_update(myRobot::robot* member);



  // constructor

  robot(std::string name, Stg::Pose i_pose, Stg::Velocity i_vel, myPlanner::base_planner *plan_gen = NULL,
        occupancy_grid::occupancyGrid2D<double, int> *map = NULL) :
  // invoking attribute constructors
      robot_name{name},
      current_pose{i_pose.x, i_pose.y, i_pose.z, i_pose.a},
      current_velocity{i_vel.x, i_vel.y, i_vel.x, i_vel.a},
      planner{plan_gen},
      occ_grid_map{map} {

    /// The constructor of the class

    /**
     * \param name : Name of the robot as a C++-string
     * \param i_pose : Initial pose of the robot of as Stg::Pose object
     * \param i_vel : Initial velocity of the robot of as Stg::Velocity object
     * \param plan_gen : The pointer to the planner object
     * \param map : The pointer to the occupancy map object
     */

    robot_id = ++gen_id; // generate id for each robot
    current_speed = std::sqrt(current_velocity.x * current_velocity.x + \
                             current_velocity.y * current_velocity.y + \
                             current_velocity.z * current_velocity.z);
    turn_speed = current_velocity.a;
    robot_name = robot_name + std::to_string(robot_id);
    img_path = img_path + std::to_string(robot_id) + "/";

  }

  robot() {
    /// Default constructor with no arguments
    robot_id = ++gen_id; // generate id for each robot
    robot_name = robot_name + std::to_string(robot_id);
    img_path = img_path + std::to_string(robot_id) + "/";
  }

  robot(Stg::Model *mod) {
    /// constructor with no arguments with the model from Stg
    robot_id =  mod->GetFiducialReturn();// generate id for each robot
    ++gen_id;
    robot_name = robot_name + std::to_string(robot_id);
    img_path = img_path + std::to_string(robot_id) + "/";
  }

  ~robot() {
    /// Destructor of robot class
    //write_map();
    //printf("\n Robot %d destructor called \n", robot_id);
    // Destroying the planner object
    delete (planner);
    delete (occ_grid_map);
  }

  // get functions
  int get_robot_id() const {
    /// Returns the id of the robot
    return robot_id;
  }

  std::string get_robot_name() const {
    /// Returns the name of the robot
    return robot_name;
  }

  Stg::Pose get_current_pose() const {
    /// Returns the current pose as Stg::Pose object
    return current_pose;
  }

  Stg::Pose get_past_pose() const {
    /// Returns the past pose as Stg::Pose object
    return past_pose;
  }

  Stg::Velocity get_current_velocity() const {
    /// Returns the current velocity as Stg::Velocity object
    return current_velocity;
  }

  Stg::Velocity get_past_velocity() const {
    /// Returns past velocity as Stg::Velocity object
    return past_velocity;
  }

  Stg::msec_t get_past_pose_time() const {
    /// Returns the past pose time as Stg::msec_t
    return past_pose_time;
  }

  double get_current_linear_speed() const {
    /// Return the 2 - norm of the linear velocity
    return std::sqrt(current_velocity.x * current_velocity.x + \
                             current_velocity.y * current_velocity.y + \
                             current_velocity.z * current_velocity.z);
  }

  myPlanner::base_planner *get_planner() const {
    /// Returns the pointer to the planner object
    return planner;
  }

  // set functions

  void set_robot_name(std::string name) {
    /// Set the name of the robot
    /**
     * \param name : the robot name
     */
    robot_name = name;
  }


  void set_current_pose(Stg::meters_t x, Stg::meters_t y, Stg::meters_t z, Stg::radians_t a) {
    /// Set the current pose of the robot using all the parameters
    /**
      * \param x : x coordinate in meters
      * \param y : y coordinate in meters
      * \param z : z coordinate in meters
      * \param a : a angle in radians
      */
    current_pose = Stg::Pose(x, y, z, a);
  }

  void set_current_pose(Stg::meters_t x, Stg::meters_t y, Stg::radians_t a) {
    /// Set the current pose of the robot using x y and a
    /**
     * Here z is set to zero
      * \param x : x coordinate in meters
      * \param y : y coordinate in meters
      * \param a : a angle in radians
      */
    current_pose = Stg::Pose(x, y, 0, a);
  }

  void set_past_pose(Stg::meters_t x, Stg::meters_t y, Stg::meters_t z, Stg::radians_t a) {
    /// Set the past pose using all the parameters
    /**
     * \param x : x coordinate in meters
     * \param y : y coordinate in meters
     * \param z : z coordinate in meters
     * \param a : a angle in radians
     */
    past_pose = Stg::Pose(x, y, z, a);
  }

  void set_past_pose(Stg::meters_t x, Stg::meters_t y, Stg::radians_t a) {
    /// Set the past pose using x y and a
    /**
     * Here z is set to zero
     * \param x : x coordinate in meters
     * \param y : y coordinate in meters
     * \param a : a angle in radians
     */
    past_pose = Stg::Pose(x, y, 0, a);
  }

  void set_current_velocity(double x, double y, double z, double a) {
    /// Set the current velocity using all the parameters
    /**
     * \param x : velocity vector component along X axis (forward speed), in meters per second.
     * \param y : velocity vector component along Y axis (sideways speed), in meters per second.
     * \param z : velocity vector component along Z axis (vertical speed), in meters per second.
     * \param a : rotational velocity around Z axis (yaw), in radians per second.
     */
    current_velocity = Stg::Velocity(x, y, z, a);
    current_speed = std::sqrt(current_velocity.x * current_velocity.x + \
                             current_velocity.y * current_velocity.y + \
                             current_velocity.z * current_velocity.z);
    turn_speed = current_velocity.a;
  }

  void set_current_velocity(double x, double y, double a) {
    /// Set the current velocity using x, y and a parameters
    /**
     * Here z is set to zero
     * \param x : velocity vector component along X axis (forward speed), in meters per second.
     * \param y : velocity vector component along Y axis (sideways speed), in meters per second.
     * \param a : rotational velocity around Z axis (yaw), in radians per second.
     */
    current_velocity = Stg::Velocity(x, y, 0, a);
    current_speed = std::sqrt(current_velocity.x * current_velocity.x + \
                             current_velocity.y * current_velocity.y + \
                             current_velocity.z * current_velocity.z);
    turn_speed = current_velocity.a;
  }

  void set_past_velocity(double x, double y, double z, double a) {
    /// Set past velocity as using all the parameters
    /**
    * \param x : velocity vector component along X axis (forward speed), in meters per second.
    * \param y : velocity vector component along Y axis (sideways speed), in meters per second.
    * \param z : velocity vector component along Z axis (vertical speed), in meters per second.
    * \param a : rotational velocity around Z axis (yaw), in radians per second.
     */
    past_velocity = Stg::Velocity(x, y, z, a);
  }

  void set_past_velocity(double x, double y, double a) {
    /// Set past velocity as using  x, y and a parameters
    /**
     * Here z is set to zero
     * \param x : velocity vector component along X axis (forward speed), in meters per second.
     * \param y : velocity vector component along Y axis (sideways speed), in meters per second.
     * \param a : rotational velocity around Z axis (yaw), in radians per second.
     */
    past_velocity = Stg::Velocity(x, y, 0, a);
  }

  void set_past_pose_time(Stg::msec_t time) {
    /// Set the past pose time as Stg::msec_t
    /**
     * \param time : an object of Stg::msec_t
     */
    past_pose_time = time;
  }

  void set_planner(myPlanner::base_planner *plan_gen) {
    /// Set planner for the robot for trajectory planning
    /// as a pointer of myPlanner::base_planner.
    /// Pointer to any object derived from base_planner can be used
    /**
     * \param plan_gen : the pointer to the planner
     */
    planner = plan_gen;
  }



  // Other functions

  void move();

  void build_map();

  void write_map(std::string path="", std::string prefix="");

  void write_map_txt(std::string path="", std::string prefix="");

  void merge_map(const std::vector<myRobot::robot *> &swarm);

  void merge_map();

  void add_map_entropy();

  void add_map_coverage();

  void write_map_entropy(std::string path, std::string prefix="");

  void write_map_coverage(std::string path, std::string prefix="");

};

}



#endif //STAGE_CTRL_PLUGIN_ROBOT_H
