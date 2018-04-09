//
// Created by Ragesh on 4/8/18.
//


#include "occupancy_grid/occupancyGrid.h"

using namespace occupancy_grid;

// renaming a namespace for brevity
namespace b_math_const = boost::math::constants;

template <typename real_t, typename int_t>
real_t occupancyGrid2D<real_t, int_t>::nearest_neighbor_distance(cv::Vec<real_t, 2> position,
                                                                 real_t max_range,
                                                                 cv::Vec<int_t, 2>& nearest_neighbor) {


  // Converting the position in to cell coordinates
  int_t i = static_cast<int_t>(std::floor((position(0) - min_pt(0))/cell_size(0)));
  int_t j = static_cast<int_t>(std::floor((position(1) - min_pt(1))/cell_size(1)));

  int_t max_range_x = static_cast<int_t>(std::floor(max_range / cell_size(0)));

  // Store the neighbor according to manhattan distance
  std::vector<cv::Vec<int_t, 2>> manhattan_neighbors;

  // Finding the neighbors
  for( int_t r=0; r < max_range_x; r++) {
    for (int_t xt = std::max(i-r, 0); xt <= std::min(i+r, og_.size[0]-1); xt++) {

      int_t ry = static_cast<int_t>(std::floor(((r - fabs(xt - i))* cell_size_(0)) / cell_size_(1)));

      // y has only two possible values
      for(int_t yt = j - ry; yt <= j + ryl; yt += ((2*ry <= 0)? 1 : 2*ry)) {
        // uncomment the line below for debugging
        // printf("(%d, %d), r: %d\n", xt, yt, r);
        if(yt >= og_.size[1] || yt < 0)
          continue;

        if(is_occupied(xt,yt)){
          manhattan_neighbors.push_back(cv::Vec<int_t, 2>(xt,yt));
          // uncomment the line below for debugging
          //printf("\n added an occupied cell to the list of neighbors");
        }

      }


    }

    if (manhattan_neighbors.size() > 0){
      break;
    }
  }

  // Finding minimum distance to the occupied neighbor cell
  real_t min_distance = std::numeric_limits<double>::infinity();

  // iterate over the neighbor cells
  for(auto it = manhattan_neighbors.begin(); it != manhattan_neighbors.end(); it++) {
    cv::Vec<real_t,2> cell_mid_pt{*it};
    // compute cell mid point
    cell_mid_pt += cv::Vec<real_t,2>(0.5, 0.5);
    cell_mid_pt = cell_mid_pt.mul(cell_size);
    cell_mid_pt += min_pt;
    real_t dist = cv::norm(cell_mid_pt - position);
    if (min_distance < dist) {
      min_distance = dist;
      nearest_neighbor = *it;
    }

    // uncomment the line below for debugging
    // printf("\n Neighbors: (%d, %d), %f \n", (*it)(0), (*it)(1), dist);
  }

  return min_distance;

}

template <typename real_t, typename int_t>
real_t occupancyGrid2D<real_t, int_t>::ray_trace(real_t px,
                                                 real_t py,
                                                 real_t p_theta,
                                                 real_t max_range,
                                                 cv::Vec<real_t, 2> &final_pos,
                                                 const bool& reflectance) {

  real_t dx = std::cos(p_theta);
  real_t dy = std::sin(p_theta);

  ray_trace_iterator<real_t, int_t> ray_trace_it(px, py, dx, dy, min_pt(0), min_pt(1), cell_size(0), cell_size(1));

  real_t dir_mag = std::sqrt(dx*dx + dy*dy);
  real_t n = std::floor(max_range * std::fabs(dx) / (dir_mag * cell_size(0))) +
             std::floor(max_range * std::fabs(dy) / (dir_mag * cell_size(1)));

  int max_size_x = og_.size[0];
  int max_size_y = og_.size[1];

  // iterate using the ray trace iterate object
  for (; n > 0 ; n--, ray_trace_it++) {

    // grid the coordinates (i,j) as std::pair
    int i = ray_trace_it->first;
    int j = ray_trace_it->second;

    // uncomment the line below to debug
    // printf("\n (%d, %d), (%f, %f) \n", i, j, tx, ty);

    // check if the coordinates are in bounds and is occupied
    if(i < 0 || j < 0 || i >= max_size_x || j >= max_size_y || is_occupied(i,j)){

      std::pair<real_t, real_t> final_pos_pair = ray_trace_it.real_position();
      final_pos(0) = final_pos_pair.first;
      final_pos(1) = final_pos_pair.second;

      real_t disp_x = final_pos(0);
      real_t disp_y = final_pos(1);
      reflectance = true;
      return std::sqrt(disp_x*disp_x + disp_y*disp_y);
    }
  }

  reflectance=false;
  return max_range;

}
