//
// Created by Ragesh on 4/2/18.
//

#ifndef STAGE_CTRL_PLUGIN_BASE_PLANNER_H
#define STAGE_CTRL_PLUGIN_BASE_PLANNER_H

/**
 * The planner is an abstract class for all planner for the robot.
 * Any planner written for the robot should inherit from base planner
 *
 */


namespace myPlanner{

    class base_planner{
        /**
         * \param planTime : the time for which motion has to be planned
         * \param planStartTime : the start time of the planner
         */
        double planTime=0;
        double planStartTime=0;

    public:

        // constructors

        base_planner(){
            /**
             * Default constructor with no arguments
             */

        }

        base_planner(double pTime, double pSTime): planTime{pTime}, planStartTime{pSTime}{
            /**
             * constructor with parameters
             * \param pTime : the time for which motion has to be planned
             * \param pSTime : the start time of the planner
             */

        }

        // get functions

        void get_planTime() const{
            return &planTime;
        }

        void get_planStartTime() const{
            return &planStartTime;
        }

        // set functions

        // other functions

        virtual get_planned_direction()=0;


    };
}

#endif //STAGE_CTRL_PLUGIN_BASE_PLANNER_H
