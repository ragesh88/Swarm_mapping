//
// Created by ragesh on 4/10/18.
//

/**
 * This file contains the definitions for the functions declared in
 * the header file "inverse_sensor_model.cpp"
 */

#include "occupancy_grid/inverse_sensor_model.h"

double occupancy_grid::reflectance_model(double grid_range, Stg::meters_t range, double max_range, double noise_sd)
/**
 * The function computes probability of occupancy of the grid cell corresponding to grid_range when the
 * ray was reflected.
 * @param grid_range : distance to starting grid
 * @param range : laser range
 * @param max_range : max range of the laser
 * @param noise_sd : standard deviation of the laser noise
 * @return : probability of occupancy of the grid
 */
{
  const double start_prob = 0.05;
  const double end_prob = 0.5;
  const double max_prob = 0.8;
  const double init_slope = (end_prob - start_prob) / max_range;
  if (grid_range <= range - 2 * noise_sd - 0.02) {
    return (init_slope * grid_range + start_prob);
  } else {
    return (max_prob);
  }
}

double occupancy_grid::non_reflectance_model(double grid_range, double max_range, double noise_sd)
/**
 * The function computes probability of occupancy of the grid cell corresponding to the grid_range when the
 * ray wasn't reflected
 * @param grid_range : distance to the starting grid
 * @param max_range : max range of the laser
 * @param noise_sd : standard deviation of the laser noise
 * @return : probability of occupancy of the grid
 */
{
  const double start_prob = 0.05;
  const double end_prob = 0.5;
  const double init_slope = (end_prob - start_prob) / (max_range - 2 * noise_sd);
  if (grid_range <= max_range - 2 * noise_sd) {
    return (init_slope * grid_range + start_prob);
  } else {
    return (end_prob);
  }

}

void occupancy_grid::log_odds_map_given_measurement_pose(const occupancy_grid::LaserSensor &sensor,
                                                         const int &ray_index,
                                                         std::map<double, cv::Vec<int, 2>> &passed_grids_ranges,
                                                         std::list<std::pair<cv::Vec<int, 2>, double>> &log_odds)
/**
 * The function computes the log odds of the map cells whose coordinates are given as values in the map
 * object passed_grids_ranges.
 * @param sensor : sensor object for the laser range sensor properties
 * @param ray_index : index of the ray in the laser range sensor object
 * @param passed_grids_ranges : map object containing distance and coordinates of ray traced grids
 * @param log_odds : list of ray traced grids and log odds
 */
{
  // find if reflectance occurred with the ray.
  // reflectance occurred if the range of the ray less than
  // max range - 2*sqrt(range_noise_const)
  bool reflectance = false;
  const double noise_variance = sensor.range_noise_const;
  if (sensor.ranges[ray_index] < (sensor.range.max - 2 * std::sqrt(noise_variance))) {
    reflectance = true;
  }

  if (reflectance) {
    for (const auto &it : passed_grids_ranges) {
      double prob = reflectance_model(it.first, sensor.ranges[ray_index], sensor.range.max, noise_variance);
      log_odds.emplace_back(std::pair<cv::Vec<int, 2>, double>(it.second, std::log(prob / (1 - prob))));

    }
  } else {
    for (const auto &it : passed_grids_ranges) {
      double prob = non_reflectance_model(it.first, sensor.range.max, noise_variance);
      log_odds.emplace_back(std::pair<cv::Vec<int, 2>, double>(it.second, std::log(prob / (1 - prob))));
    }

  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void occupancy_grid::probability_map_given_measurement_pose(const occupancy_grid::LaserSensor &sensor,
                                                            const int &ray_index,
                                                            std::map<double, cv::Vec<int, 2>> &passed_grids_ranges,
                                                            std::list<std::pair<cv::Vec<int, 2>, double>> &probability)
/**
 * The function computes the probability of the map cells whose coordinates are given as values in the map
 * object passed_grids_ranges.
 * @param sensor : sensor object for the laser range sensor properties
 * @param ray_index : index of the ray in the laser range sensor object
 * @param passed_grids_ranges : map object containing distance and coordinates of ray traced grids
 * @param probability : list of ray traced grids and probabilities
 */
{

  // find if reflectance occurred with the ray.
  // reflectance occurred if the range of the ray less than
  // max range - 2*sqrt(range_noise_const)
  bool reflectance = false;
  const double noise_variance = sensor.range_noise_const;
  if (sensor.ranges[ray_index] < (sensor.range.max - 2 * std::sqrt(noise_variance))) {
    reflectance = true;
  }

  if (reflectance) {
    for (const auto &it : passed_grids_ranges) {
      double prob = reflectance_model(it.first, sensor.ranges[ray_index], sensor.range.max, noise_variance);
      probability.emplace_back(std::pair<cv::Vec<int, 2>, double>(it.second, prob));

    }
  } else {
    for (const auto &it : passed_grids_ranges) {
      double prob = non_reflectance_model(it.first, sensor.range.max, noise_variance);
      probability.emplace_back(std::pair<cv::Vec<int, 2>, double>(it.second, prob));
    }

  }
}



