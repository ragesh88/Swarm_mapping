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

double log_odds_observations_given_map_pose(const Observation2D& observation,
                                            occupancyGrid2D<double, int>& map,
                                            double& expected_range)
{

    double total_angle = observation.p_theta;
  


}
