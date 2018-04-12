//
// Created by Ragesh on 4/11/18.
//

/**
 * This header file defines the classes and function prototype to model the
 * laser range sensor using an inverse sensor model.
 * By inverse sensor model we mean \f$ P(m_i|x,z) \f$.
 * \f$ P(m_i|x,z)\f$ means the probability that a particular grid cell \f$ m_i \f$
 * is occupied given the measurements and position of the robot.
 */

#ifndef STAGE_CTRL_PLUGIN_INVERSE_SENSOR_MODEL_H
#define STAGE_CTRL_PLUGIN_INVERSE_SENSOR_MODEL_H

#include<Stage-4.3/stage.hh>
#include<opencv2/opencv.hpp>
#include<list>

namespace occupancy_grid{

typedef Stg::ModelRanger::Sensor LaserSensor;

void log_odds_map_given_measurement_pose(const LaserSensor& sensor,
                                         const int& ray_index,
                                         std::map<double,cv::Vec<int,2>>& passed_grids_ranges,
                                         std::list<std::pair<cv::Vec<int,2>,double>>& log_odds);

void probability_map_given_measurement_pose(const LaserSensor& sensor,
                                            const int& ray_index,
                                            std::map<double,cv::Vec<int,2>>& passed_grids_ranges,
                                            std::list<std::pair<cv::Vec<int,2>,double>>& probability);

//double reflectance_model( double grid_range,  Stg::meters_t range, double max_range,  double noise_sd);

//double non_reflectance_model( double grid_range,  double max_range,  double noise_sd);

}



#endif //STAGE_CTRL_PLUGIN_INVERSE_SENSOR_MODEL_H
