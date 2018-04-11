//
// Created by Ragesh on 4/6/18.
//

/**
 * This is the header file defines the prototypes for the necessary classes and
 * functions used to store and operate over a 2D occupancy grid map.
 * Every object of the robot class maintains an object of the occupancy grid class
 * in order to perform 2D occupancy grid mapping.
 */

#ifndef STAGE_CTRL_PLUGIN_OCCUPANCYGRID_H
#define STAGE_CTRL_PLUGIN_OCCUPANCYGRID_H

#include <Stage-4.3/stage.hh>
#include <opencv2/opencv.hpp>

// C libraries
#include <cassert>

// C++ libraries
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <map>

// Third party libraries
#include <boost/math/constants/constants.hpp>

#include "ray_trace_iterator.h" // for ray trace operations

// TODO : Need to clean up the code. I think it is better to make Prob_occupancyGrid2D inherit from
// TODO : the class occupancyGrid2D. By this manner the robot class can have a pointer to occupancyGrid2D object
// TODO : and use virtual function to invoke the appropriate method of the class.

namespace occupancy_grid{

      template <typename int_t>
      class vec_comp_class;

    template <typename real_t, typename int_t>
    class occupancyGrid2D {
      /**
       * The occupancyGrid class stores the 2D occupancy map and
       * has the methods to perform various operation on the grid map.
       *
       */
     public:

      /// the minimum coordinate of the map
      cv::Vec<real_t, 2> min_pt{static_cast<real_t>(0), static_cast<real_t>(0)};
      /// the cell size of the grid in meters
      cv::Vec<real_t, 2> cell_size{static_cast<real_t>(0.2), static_cast<real_t>(0.2)};
      /// the matrix to store the occupancy values
      cv::Mat og_{100, 100, CV_8U, cv::Scalar(FREE)};
      /// a counter variable
      int counter{0};
      /// Value when the map cell is occupied
      static const uint8_t OCCUPIED{255};
      /// Value when the map cell is free
      static const uint8_t FREE{0};
      /// Value when the map cell status is unknown
      static const uint8_t UNKNOWN{127};

      // Constructors

      occupancyGrid2D() {
        ///Default constructor
      }

      occupancyGrid2D(real_t min_x, real_t min_y, real_t cell_size_x, real_t cell_size_y, uint n_cells_x, uint n_cell_y)
          :
          min_pt{min_x, min_y}, cell_size{cell_size_x, cell_size_y}, og_{n_cells_x, n_cell_y, CV_8U, cv::Scalar(FREE)},
          counter(0) {
        /**
         * The constructor for the occupancyGrid2D class.
         *
         * \param min_x : the minimum x coordinate of the actual map
         * \param min_y : the minimum y coordinate of the actual map
         * \param cell_size_x : the size of the cell in the grid map along x in meters
         * \param cell_size_y : the size of the cell in the grid map along y in meters
         * \param n_cell_x : the number of cell in the grid map along x
         * \param n_cell_y : the number of cell in the grid map along y
         *
         */

      }


      // get functions

      uint8_t get(int k) const {
        /// get the value at the \f$k^{the}\f$ location
        int row = k / og_.size[1];
        int col = k % og_.size[1];
        return og_.at<uint8_t>(row, col);
      }

      uint8_t get(int row, int col) const {
        /// get the value at \f$(row,col)\f$ location
        return og_.at<uint8_t>(row, col);
      }



      // set functions

      void set(int k, uint8_t value) {
        /// set the value at the \f $k^{th}\f$ location
        int row = k / og_.size[1];
        int col = k % og_.size[1];
        og_.at<uint8_t>(row, col) = value;

      }

      void set(int row, int col, uint8_t value) {
        /// set the value at \f$(row,col)\f$ location
        og_.at<uint8_t>(row, col) = value;
      }


      // other functions

      bool out_of_bounds(int i, int j) {
        /// the function checks whether the index pair \f$(i,j)\f$ is strictly inside the grid
        return (i >= og_.size[0] || j >= og_.size[1] || i < 0 || j < 0);
      }

      virtual bool is_occupied(int i, int j) {
        /// check if the grid cell in \f$(i,j)\f$ is occupied
        return (og_.at<uint8_t>(i, j) != FREE);
      }

      real_t nearest_neighbor_distance(cv::Vec<real_t, 2> position, real_t max_range,
                                       cv::Vec<int_t, 2> &nearest_neighbor);

      real_t ray_trace(real_t px, real_t py, real_t p_theta, real_t max_range, cv::Vec<real_t, 2> &final_pos,
                        bool &reflectance);

      void ray_trace_all(real_t px, real_t py,real_t p_theta, real_t max_range,
                         std::map<cv::Vec<int_t, 2>, real_t,vec_comp_class<int_t>>& all_pos_range);



      cv::Point2i xy2rc(const cv::Vec<real_t, 2> &xy) const {
        /// the function converts the \f$(x,y)\f$ coordinates to corresponding
        /// row and col coordinates of the grid map
        int row = static_cast<int>((xy(0) - min_pt(0)) / cell_size(0));
        int col = static_cast<int>((xy(1) - min_pt(1)) / cell_size(1));
        return cv::Point2i(col, row);
      }

    };

/////////////////////////////////////////////////////////////////////////////////////////////////////////

// Definitions for the members of the class occupancyGrid2D

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

      int_t ry = static_cast<int_t>(std::floor(((r - fabs(xt - i))* cell_size(0)) / cell_size(1)));

      // y has only two possible values
      for(int_t yt = j - ry; yt <= j + ry; yt += ((2*ry <= 0)? 1 : 2*ry)) {
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename real_t, typename int_t>
real_t occupancyGrid2D<real_t, int_t>::ray_trace(real_t px,
                                                 real_t py,
                                                 real_t p_theta,
                                                 real_t max_range,
                                                 cv::Vec<real_t, 2> &final_pos,
                                                 bool& reflectance) {

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

/////////////////////////////////////////////////////////////////////////////////////////////////////////


template <typename real_t, typename int_t>
void occupancyGrid2D<real_t, int_t>::ray_trace_all(real_t px,real_t py, real_t p_theta, real_t max_range,
                                                   std::map<cv::Vec<int_t, 2>, real_t,vec_comp_class<int_t>>& all_pos_range)

  /// The function return the grid coordinates of all the grids that the ray pass through.
{
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

    const std::pair<real_t, real_t>& pos_pair = ray_trace_it.real_position();
    auto range = std::sqrt((pos_pair.first-px)*(pos_pair.first-px) +
                 (pos_pair.second-py)*(pos_pair.second-py));

    // uncomment the line below to debug
    printf("\n (%d, %d), (%f, %f) \n", i, j, pos_pair.first, pos_pair.second);


    // check if the coordinates are in bounds and is occupied
    if(i < 0 || j < 0 || i >= max_size_x || j >= max_size_y || range >= max_range){

      break;

    }
    // inserting the elements
    all_pos_range.insert(std::pair<cv::Vec<int_t, 2>, real_t>(cv::Vec<int_t,2>{i,j},range));


    

  }


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename int_t>
class vec_comp_class{
 public:
  bool operator()(const cv::Vec<int_t, 2>& t1, const cv::Vec<int_t, 2>& t2){
    // An operator to compare the cv::Vec<int_t, 2> objects
    if(t1[0] < t2[0]){
      return true;
    } else {
      if (t1[0] == t1[0]){
        return (t1[1] < t2[1]);
      }
      return false;
    }

  }
};



//////////////////////////////////////////////////////////////////////////////////////////////////////////
      ///// The prototype for class Prob_occupancyGrid2D ///////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////


      template <typename real_t, typename int_t>
      class Prob_occupancyGrid2D {
        /**
         * The Prob_occupancyGrid class stores the 2D probabilistic occupancy map and
         * has the methods to perform various operation on the grid map.
         *
         */
       public:

        /// the minimum coordinate of the map
        cv::Vec<real_t, 2> min_pt{static_cast<real_t>(0), static_cast<real_t>(0)};
        /// the cell size of the grid in meters
        cv::Vec<real_t, 2> cell_size{static_cast<real_t>(0.2), static_cast<real_t>(0.2)};
        /// the matrix to store the occupancy values
        cv::Mat og_{100,100, CV_16S, cv::Scalar(UNKNOWN)};
        /// a counter variable
        int counter{0};
        /// Value when the map cell is occupied
        static const int OCCUPIED{INT16_MAX};
        /// Value when the map cell is free
        static const int FREE{INT16_MIN};
        /// Value when the map cell status is unknown
        static const int UNKNOWN{0};

        // Constructors

        Prob_occupancyGrid2D(){
          ///Default constructor
        }

        Prob_occupancyGrid2D(real_t min_x, real_t min_y, real_t cell_size_x, real_t cell_size_y, uint n_cells_x, uint n_cell_y):
            min_pt{min_x, min_y}, cell_size{cell_size_x, cell_size_y}, og_{n_cells_x, n_cell_y, CV_16S, cv::Scalar(UNKNOWN)},
            counter(0){
          /**
           * The constructor for the occupancyGrid2D class.
           *
           * \param min_x : the minimum x coordinate of the actual map
           * \param min_y : the minimum y coordinate of the actual map
           * \param cell_size_x : the size of the cell in the grid map along x in meters
           * \param cell_size_y : the size of the cell in the grid map along y in meters
           * \param n_cell_x : the number of cell in the grid map along x
           * \param n_cell_y : the number of cell in the grid map along y
           *
           */

        }


        // get functions

        uint8_t get(int k) const{
          /// get the value at the \f$k^{the}\f$ location
          int row = k/og_.size[1];
          int col = k%og_.size[1];
          return og_.at<int>(row, col);
        }

        uint8_t get(int row, int col) const{
          /// get the value at \f$(row,col)\f$ location
          return og_.at<int>(row, col);
        }




        // set functions

        void set(int k, int value){
          /// set the value at the \f $k^{th}\f$ location
          int row = k/og_.size[1];
          int col = k%og_.size[1];
          og_.at<int>(row, col) = value;

        }

        void set(int row, int col, int value){
          /// set the value at \f$(row,col)\f$ location
          og_.at<int>(row, col) = value;
        }


        // other functions

        bool out_of_bounds(int i, int j) {
          /// the function checks whether the index pair \f$(i,j)\f$ is strictly inside the grid
          return (i >= og_.size[0] || j >= og_.size[1] || i < 0 || j < 0);
        }

        virtual bool is_occupied(int i, int j){
          /// check if the grid cell in \f$(i,j)\f$ is occupied
          return (og_.at<int>(i,j) != FREE);
        }

        real_t nearest_neighbor_distance(cv::Vec<real_t, 2> position, real_t max_range,
                                         cv::Vec<int_t, 2>& nearest_neighbor);

        real_t ray_trace(real_t px, real_t py, real_t p_theta, real_t max_range, cv::Vec<real_t, 2>& final_pos,
                          bool& reflectance);

        cv::Point2i xy2rc(const cv::Vec<real_t, 2>& xy) const{
          /// the function converts the \f$(x,y)\f$ coordinates to corresponding
          /// row and col coordinates of the grid map
          int row = static_cast<int>((xy(0)-min_pt(0))/cell_size(0));
          int col = static_cast<int>((xy(1)-min_pt(1))/cell_size(1));
          return cv::Point2i(col, row);
        }





      };


// Definitions for the members of class Prob_occupancyGrid2D///////////////////////////////////////////////////////////

template <typename real_t, typename int_t>
real_t Prob_occupancyGrid2D<real_t, int_t>::nearest_neighbor_distance(cv::Vec<real_t, 2> position,
                                                                      real_t max_range,
                                                                      cv::Vec<int_t, 2>& nearest_neighbor) {

  /**
   * The function computes and the nearest neighbor distance in the map. The function also the return
   * nearest neighbor using the reference to the parameter nearest_neighbor
   */


  // Converting the position in to cell coordinates
  int_t i = static_cast<int_t>(std::floor((position(0) - min_pt(0))/cell_size(0)));
  int_t j = static_cast<int_t>(std::floor((position(1) - min_pt(1))/cell_size(1)));

  int_t max_range_x = static_cast<int_t>(std::floor(max_range / cell_size(0)));

  // Store the neighbor according to manhattan distance
  std::vector<cv::Vec<int_t, 2>> manhattan_neighbors;

  // Finding the neighbors
  for( int_t r=0; r < max_range_x; r++) {
    for (int_t xt = std::max(i-r, 0); xt <= std::min(i+r, og_.size[0]-1); xt++) {

      int_t ry = static_cast<int_t>(std::floor(((r - fabs(xt - i))* cell_size(0)) / cell_size(1)));

      // y has only two possible values
      for(int_t yt = j - ry; yt <= j + ry; yt += ((2*ry <= 0)? 1 : 2*ry)) {
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
real_t Prob_occupancyGrid2D<real_t, int_t>::ray_trace(real_t px,
                                                      real_t py,
                                                      real_t p_theta,
                                                      real_t max_range,
                                                      cv::Vec<real_t, 2> &final_pos,
                                                      bool& reflectance) {
  /**
   * The function returns the final position of the ray if it is hit by an obstacle.
   */

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


}




#endif //STAGE_CTRL_PLUGIN_OCCUPANCYGRID_H
