//
// Created by Ragesh on 4/8/18.
//

// boost libraries
#include <boost/math/constants/constants.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include "occupancy_grid/occupancyGrid.h"
#include "occupancy_grid/forward_sensor_model.h"

using namespace occupancy_grid;

double log_odds_observations_given_map_pose(const Observation2D &observation,
                                            occupancyGrid2D<double, int> &map,
                                            double &expected_range)
/**
 * The function returns log odds of the observation given the map and pose of
 * the robot.
 * @param observation
 * @param map
 * @param expected_range
 * @return
 */
{  //TODO : complete the parameter descriptions above

  double total_angle = observation.p_theta;
  cv::Vec2d direction{std::cos(total_angle), sin(total_angle)};
  cv::Vec2d position{observation.px, observation.py};

  assert(!std::isnan(direction(1)));
  assert(!std::isnan(direction(0)));

  cv::Vec2d final_pos;
  bool reflectance;
  expected_range = map.ray_trace(observation.px, observation.py, observation.p_theta,
                                 LASER_MAX_RANGE, final_pos, reflectance);

  // noise variance increases with distance
  double sigma = NOISE_VARIANCE * expected_range;

  // uncomment the lines below for debugging
  // printf("Sigma:%f\n", sigma);
  // printf("Expected range:%f\n", expected_range);
  // printf("Observed range:%f\n", observation.range);

  return log_gaussian1D(observation.range, expected_range, sigma);

}

double probability_observations_given_map_pose(const Observation2D &observation,
                                               occupancyGrid2D<double, int> &map,
                                               double &expected_range)
/**
 * The function returns log odds of the observation given the map and pose of
 * the robot.
 * @param observation
 * @param map
 * @param expected_range
 * @return
 */
{  //TODO : complete the parameter descriptions above

  double total_angle = observation.p_theta;
  cv::Vec2d direction{std::cos(total_angle), sin(total_angle)};
  cv::Vec2d position{observation.px, observation.py};

  assert(!std::isnan(direction(1)));
  assert(!std::isnan(direction(0)));

  cv::Vec2d final_pos;
  bool reflectance;
  expected_range = map.ray_trace(observation.px, observation.py, observation.p_theta,
                                 LASER_MAX_RANGE, final_pos, reflectance);

  // noise variance increases with distance
  double sigma = NOISE_VARIANCE * expected_range;

  // uncomment the lines below for debugging
  // printf("Sigma:%f\n", sigma);
  // printf("Expected range:%f\n", expected_range);
  // printf("Observed range:%f\n", observation.range);

  return gaussian1D(observation.range, expected_range, sigma);

}
