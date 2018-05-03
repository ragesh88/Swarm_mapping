//
// Created by Ragesh on 4/8/18.
//

/**
 * This header file defines prototype for the necessary classes and functions used to
 * model a laser range sensor using a forward sensor model.
 * By forward sensor model we mean \f$ P(z|x,m) \f$.
 * \f$ P(z|x,m)\f$ means the probability of a occurrence of range measurement \f$ z \f$ given the pose \f$ x \f$
 * and map \f$ m \f$ of the environment
 */

#ifndef STAGE_CTRL_PLUGIN_FORWARD_SENSOR_MODEL_H
#define STAGE_CTRL_PLUGIN_FORWARD_SENSOR_MODEL_H

#include "occupancyGrid.h"

// Defining new namespaces for brevity
namespace b_const = boost::math::constants;

namespace occupancy_grid {

// Defining the constants required for the sensor model
/// The maximum Laser sensor range. This value should match the one defined
/// for range sensor in world file used for stage simulation
const double LASER_MAX_RANGE = 2.0;
/// The Noise variance of the laser along the radial axis
const double NOISE_VARIANCE = 0.05;

template<typename real_t>
class Gen_observation2D {
  /**
   * Gen_observation2D is a template class for storing range measurements
   *
   */
 public:
  real_t px, py, p_theta, range;
  // Constructors
  Gen_observation2D() {}

  Gen_observation2D(real_t px, real_t py, real_t p_theta, real_t range) : px{px}, py{py}, p_theta{p_theta},
                                                                          range{range} {}
};

/// Defined the new class with template specification
typedef Gen_observation2D<double> Observation2D;

double log_odds_observations_given_map_pose(const Observation2D &observation,
                                            occupancyGrid2D<double, int> &map,
                                            double &expected_range);

double probability_observations_given_map_pose(const Observation2D &observation,
                                               occupancyGrid2D<double, int> &map,
                                               double &expected_range);

inline double log_gaussian1D(double x, double mu, double sigma) {
  /// Log of the one dimensional Gaussian function
  return (-0.5 * (x - mu) * (x - mu) / (sigma * sigma)) + log(sigma * b_const::root_two_pi<double>());
}

inline double gaussian1D(double x, double mu, double sigma) {
  /// Value of the one dimensional Gaussian function
  return exp(-0.5 * (x - mu) * (x - mu) / (sigma * sigma)) / (sigma * b_const::root_two_pi<double>());
}

}

#endif //STAGE_CTRL_PLUGIN_FORWARD_SENSOR_MODEL_H
