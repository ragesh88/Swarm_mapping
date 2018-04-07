//
// Created by Ragesh on 4/6/18.
//

/**
 * This is the header file which defines the prototypes for the necessary classes and
 * functions used to store and operate over a 2D occupancy grid map.
 * Every object of the robot class maintains an object of the occupancy grid class
 * in order to perform 2D occupancy grid mapping.
 */

#ifndef STAGE_CTRL_PLUGIN_OCCUPANCYGRID_H
#define STAGE_CTRL_PLUGIN_OCCUPANCYGRID_H

#include <Stage-4.3/stage.hh>
#include <opencv2/opencv.hpp>


namespace myRobot{

    template <typename real_t, typename int_t>
    class occupancyGrid {
        /**
         * The occupancyGrid class stores the 2D occupancy map and
         * has the methods to perform various operation on the grid map.
         */
    public:

    };

}




#endif //STAGE_CTRL_PLUGIN_OCCUPANCYGRID_H
