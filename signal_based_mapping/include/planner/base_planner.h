//
// Created by Ragesh on 4/2/18.
//


#ifndef STAGE_CTRL_PLUGIN_BASE_PLANNER_H
#define STAGE_CTRL_PLUGIN_BASE_PLANNER_H

// C++ header files
#include <functional>
#include <limits>
#include <numeric>

#include "occupancy_grid/occupancyGrid.h"

// stage header file added for some functions
#include <Stage-4.3/stage.hh>

/**
 * The planner is an abstract class for all planner for the robot.
 * Any planner written for the robot should inherit from base planner
 *
 */


namespace myPlanner {

/**
  * \brief The enum structure MOTION_MODES enumerates the various modes
  * of motion encoded in a via point of a motion path.
 */
enum MOTION_MODES {

  /// the via point with no velocity
  START,
  /// the via point with only rotational velocity along z direction
  ROTATION_Z,
  /// the via point with translational velocity
  TRANSLATION,
  /// the via point with only translational velocity along the x direction wrt to robot
  TRANSLATION_X,
  ///  the via point with only translational velocity along the y direction wrt to robot
  TRANSLATION_Y,
  /// the via point with all the velocities
  ALL
};

 /**
   * \brief The via points structure is used to store each via point in the planned
   *
   */
struct via_points {

  /// the motion mode corresponding to the via point
  MOTION_MODES modes = START;
  /// the time till the current control action to be done
  double motion_end_time = 0;
  /// the control action for the current via point
  Stg::Velocity vel_control{0.0, 0.0, 0.0, 0.0};
  /// the desired pose for the via point
  Stg::Pose des_pose{0.0, 0.0, 0.0, 0.0};
  /// flag to check whether the desired pose is computed
  bool computed_desPose = false;
};

/// a type defined type path for storing path
typedef std::queue<via_points> Path;

/**
 * The class base_planner is a class with basic attributes and methods for path generation.
 * Any class that has to perform planning should be derive from this class and implement its
 * virtual method generate_path
 */
class base_planner {

  /// the time for which motion has to be planned
  double planTime = 0;
  /// the start time of the planner
  double planStartTime = 0;
  /// the pose of the robot at the start of planning
  Stg::Pose startPose = Stg::Pose(0.0, 0.0, 0.0, 0.0);
  /// the velocity of the robot for planning
  Stg::Velocity robotTwist = Stg::Velocity(0.0, 0.0, 0.0, 0.0);

 protected:
  /// planned path
  Path path;
  /// check if the planner is using map
  bool USING_MAP;

 public:

  // constructors

  base_planner() {
    /**
     * Default constructor with no arguments
     */

  }

  base_planner(double pTime, double pSTime, Stg::Pose P, Stg::Velocity V) :
      planTime{pTime},
      planStartTime{pSTime},
      startPose(P.x, P.y, P.z, P.a),
      robotTwist(V.x, V.y, V.z, V.a),
      USING_MAP{false}{
    /**
     * constructor with parameters
     * \param pTime : the time for which motion has to be planned
     * \param pSTime : the start time of the planner
     * \param P : the pose of the robot at the start of planning
     * \param V : the velocity of the robot
     */

  }

  // get functions

  double get_planTime() const
  /**
   *
   * @return : planning time
   */
  {
    return planTime;
  }

  double get_planStartTime() const
  /**
   *
   * @return : planning start time
   */
  {
    return planStartTime;
  }

  const Stg::Pose *get_startPose() const
  /**
   *
   * @return : const pointer to the start Pose
   */
  {
    return &startPose;
  }

  const Stg::Velocity *get_velocity() const
  /**
   *
   * @return : const pointer to the velocity object
   */
  {
    return &robotTwist;
  }

  Path *get_path()
  /**
   *
   * @return : pointer to the generated path
   */
  {
    return &path;
  }

  bool is_using_map()
  /**
   * check if the planner is using a map for planning
   * @return : whether the planner is using a map or not
   */
  {
    return USING_MAP;
  }

  // set functions

  void set_planTime(double time)
  /**
   * setting the planning time
   * @param time : duration of the path to be planned
   */
  {
    planTime = time;
  }

  void set_planStartTime(double time)
  /**
   * setting the start time for planning
   * @param time : the start time for planning
   */
  {
    planStartTime = time;
  }

  void set_startPose(const Stg::Pose &P)
  /**
   * setting the start pose
   * @param P : start pose
   */
  {
    startPose.x = P.x;
    startPose.y = P.y;
    startPose.z = P.z;
    startPose.a = P.a;
  }

  void set_robotTwist(const Stg::Velocity &V)
  /**
   * setting the velocity vector
   * @param V : velocity vector object
   */
  {
    robotTwist.x = V.x;
    robotTwist.y = V.y;
    robotTwist.z = V.z;
    robotTwist.a = V.a;
  }

  // other functions
  ///The function generates path for a base planner
  virtual void generate_path(double start_time);

  /// The function generates path for a derive planner class
  /// the non trivial implementation can be found in MI_levyWalk_planner class
  virtual void generate_path(double start_time, occupancy_grid::occupancyGrid2D<double, int>* map){}

  void delete_path() {
    /// The function deletes the path stored in the variable
    while (!path.empty()) path.pop();

  }

  virtual ~base_planner() {}

};

//////////////////////////////////////////////////////////////////////////////////////////////
/**
   * \brief The class the implements the path generation based on
   * a Levy Walk motion. The class inherits from the base planner class
   *
   */
class levyWalk_planner : public base_planner {


  /// the minimum angle for the Levy walk
  Stg::radians_t min_ang = -M_PI;
  /// the maximum angle for the Levy walk
  Stg::radians_t max_ang = M_PI;
  /// alpha value for the levy walk
  double alpha = 1.5;
  /// minimum distance for the levy walk in meters
  Stg::meters_t levy_min = Stg::meters_t{3.0};

 public:

  // constructors
  levyWalk_planner() : base_planner() {
    /// Default constructor with no arguments

  }

  levyWalk_planner(double pSTime, Stg::Pose P, Stg::Velocity V, Stg::radians_t min = -M_PI, Stg::radians_t max = M_PI,
                   double a = 1.5, Stg::meters_t l_min = 3.0) :
      base_planner(0, pSTime, P, V),
      min_ang{min},
      max_ang{max},
      alpha{a},
      levy_min{l_min}
  /**
    * \brief The constructor with parameters
    * In Levy walk the time for the walk is generated from
    * a probability distribution.
    * @param pSTime : start time
    * @param P : start pose of the robot
    * @param V : velocity of the robot
    * @param min : min angle
    * @param max : max angle
    * @param a : exponent of the Levy walk
    * @param l_min : minimum levy distance
    */
 {

 }

  // other functions

  Stg::meters_t generate_levy_dist();

  Stg::radians_t generate_random_direction();

  virtual void generate_path(double start_time);

  virtual ~levyWalk_planner() {}
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// declaration type for angle variables
typedef double radians;
/// declaration type for distance variables
typedef double meters;

 /**
   * The structure stores the parameters for the forward sensor
   * for mutual information computation
   */
struct F_S_M_parameters{

   /// The maximum distance that a sensor can detect
   meters z_max{2};
   /// The variance of the sensor along the radial direction when modelled as Gaussian
   meters sigma{0.01};
   /// The minimum angle of the laser range sensor wrt to the robot base
   radians min_angle{-M_PI/2};
   /// The maximum angle of the laser range sensor wrt to the robot base
   radians max_angle{M_PI/2};
};

 /**
   * enum data type to specify different reward objective to be maximized
   */
enum REWARD{
   /// KL divergence Mutual information based reward computation
   KLDMI,
   /// Cauchy Schwarz mutual information based reward computation
   CSQMI,
   /// Entropy based reward computation
   ENTROPY
};

 /**
   * @brief The class implements the path generation for a robot by combining levy walk with
   * the Mutual Information between the sensor model and the map. The basic idea behind the planning
   * is instead of choosing a random direction like in the levy walk setting choose a direction
   * that maximizes the information gain based the mutual information.
   *
   */
class MI_levyWalk_planner : public base_planner {


  /// to store the forward sensor model parameters of the sensor
  F_S_M_parameters fsm;
  /// the reward function to be maximized
  REWARD reward;
  /// the minimum angle for the Levy walk
  radians min_ang;
  /// the maximum angle for the Levy walk
  radians max_ang;
  /// alpha value for the levy walk
  double alpha;
  /// minimum distance for the levy walk in meters
  meters levy_min;
  /// the distance adjacent via points in a path
  meters dist_btw_path_via;
  /// number of path consider on each side of the robot for computing the mutual information
  int no_path_each_side{3}; // better to keep it odd
  /// number of beams consider for computing the mutual information
  int no_beams_beam_set{8};
  /// store the directions of various beams wrt robot(local coordinates)
  std::vector<radians> beam_dir;

  // private methods
  /// the method to generate the distance to move according to the levy distribution
  meters generate_levy_dist();
  /// the method to generate the intermediate via points as poses for a given direction in the starting pos
  void generate_dir_via_point(const Stg::Pose& start_pos, const meters& plan_len, std::queue<Stg::Pose>& dir_via_point);
  /// the method to compute the Cauchy Schwarz mutual information of a beam based on
  /// radial Gaussian noise forward range sensor model
  double compute_beam_CSQMI(occupancy_grid::occupancyGrid2D<double, int>* map, double px, double py, double p_theta);
  /// the method to compute the KL divergence mutual information of a beam based on
  /// radial Gaussian noise forward range sensor model
  double compute_beam_KLDMI(occupancy_grid::occupancyGrid2D<double, int>* map, double px, double py, double p_theta);
  /// the method to compute the entropy of the cells traced by a beam
  double compute_beam_Entropy(occupancy_grid::occupancyGrid2D<double, int>* map, double px, double py, double p_theta);

 public:

  // constructor

  MI_levyWalk_planner(double pSTime, Stg::Pose P, Stg::Velocity V, F_S_M_parameters fsm_, REWARD reward_=KLDMI,
                      meters l_min = 3.0, radians min = -M_PI, radians max = M_PI, double a = 1.5,
                       meters dis_btw_path_via_=2):
                      base_planner{0, pSTime, P, V},
                      fsm{fsm_},
                      reward(reward_),
                      min_ang{min},
                      max_ang{max},
                      alpha{a},
                      levy_min{l_min},
                      dist_btw_path_via{dis_btw_path_via_}

  /**
   * The constructor with parameters for the class
   * @param pSTime : start time for planning
   * @param P : the start pose for planning
   * @param V : the start velocity for planning
   * @param min : the min angle to rotate
   * @param max : the max angle to rotate
   * @param a : the alpha value of the levy distribution
   * @param l_min : the min distance that robot should move
   * @param dis_btw_path_via_ : the distance adjacent via points in a path
   * @param fsm_ : forward sensor model of parameters
   * @param reward_ : the kind of reward function to be evaluated
   */
  {


    beam_dir.push_back(fsm.min_angle);
    // generate the directions of the beams to compute MI from [fsm.min_angle fsm.max_angle]
    radians incre_angle = (fsm.max_angle-fsm.min_angle)/(no_beams_beam_set - 1);
    while(beam_dir.back() <= fsm.max_angle){
      beam_dir.push_back(beam_dir.back() + incre_angle);
    }

    // setting the flag to indicate that the planner uses map
    USING_MAP = true;


  }

  // public methods

  virtual void generate_path(double start_time, occupancy_grid::occupancyGrid2D<double, int>* map);

  virtual ~MI_levyWalk_planner() {}
};

// Defining new namespaces for brevity
namespace b_const = boost::math::constants;

inline double gaussian1D(double x, double mu, double sigma) {
  /// Value of the one dimensional Gaussian function
  return exp(-0.5 * (x - mu) * (x - mu) / (sigma * sigma)) / (sigma * 2*b_const::root_two_pi<double>());
}

}

#endif //STAGE_CTRL_PLUGIN_BASE_PLANNER_H
