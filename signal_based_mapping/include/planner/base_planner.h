//
// Created by Ragesh on 4/2/18.
//


#ifndef STAGE_CTRL_PLUGIN_BASE_PLANNER_H
#define STAGE_CTRL_PLUGIN_BASE_PLANNER_H

// stage header file added for some functions
#include <Stage-4.3/stage.hh>

/**
 * The planner is an abstract class for all planner for the robot.
 * Any planner written for the robot should inherit from base planner
 *
 */


namespace myPlanner{

    enum MOTION_MODES {

        /**
         * \brief The enum structure MOTION_MODES enumerates the various modes
         * of motion encoded in a via point of a motion path.
         *
         * \param START : the via point with no velocity
         * \param ROTATION_Z : the via point with only rotational velocity along z direction
         * \param TRANSLATION_X : the via point with only translational velocity along the x direction wrt to robot
         * \param TRANSLATION_Y : the via point with only translational velocity along the y direction wrt to robot
         * \param ALL : the via point with all the velocities
         */

        START,
        ROTATION_Z,
        TRANSLATION_X,
        TRANSLATION_Y,
        ALL
    };

    struct via_points{
        /**
         * \brief The via points structure is used to store each via point in the planned
         *
         * \param modes : the motion mode corresponding to the via point
         * \param motion_end_time : the time till the current control action to be done
         * \param vel_control : the control action for the current via point
         * \param des_pose : the desired pose for the via point
         * \param computed_desPose : flag to check whether the desired pose is computed
         */
        MOTION_MODES modes=START;
        double motion_end_time=0;
        Stg::Velocity vel_control{0.0, 0.0, 0.0, 0.0};
        Stg::Pose des_pose{0.0, 0.0, 0.0, 0.0};
        bool computed_desPose = true;
    };

    /// a type defined type path for storing path
    typedef std::queue<via_points> Path;


    class base_planner{
        /**
         * \param planTime : the time for which motion has to be planned
         * \param planStartTime : the start time of the planner
         * \param startPose : the pose of the robot at the start of planning
         * \param startVelocity : the velocity of the robot at the start of the planning
         * \param path : the planned path
         */
        double planTime=0;
        double planStartTime=0;
        Stg::Pose startPose=Stg::Pose(0.0, 0.0, 0.0, 0.0);
        Stg::Velocity startVelocity = Stg::Velocity(0.0, 0.0, 0.0, 0.0);
        Path path;

    public:

        // constructors

        base_planner(){
            /**
             * Default constructor with no arguments
             */

        }

        base_planner(double pTime, double pSTime, Stg::Pose P, Stg::Velocity V):
                planTime{pTime},
                planStartTime{pSTime},
                startPose(P.x, P.y, P.z, P.a),
                startVelocity(V.x, V.y, V.z, V.a){
            /**
             * constructor with parameters
             * \param pTime : the time for which motion has to be planned
             * \param pSTime : the start time of the planner
             * \param P : the pose of the robot at the start of planning
             */

        }

        // get functions

        double get_planTime() const{
            return planTime;
        }

        double get_planStartTime() const{
            return planStartTime;
        }

         const Stg::Pose* get_startPose() const{
            return &startPose;
        }

         const Stg::Velocity* get_velocity() const{
            return &startVelocity;
        }

        // set functions

        void set_planTime(double time){
            planTime = time;
        }

        void set_planStartTime(double time){
            planStartTime = time;
        }

        void set_startPose(const Stg::Pose &P){
            startPose.x=P.x;
            startPose.y=P.y;
            startPose.z=P.z;
            startPose.a=P.a;
        }

        void set_startVelocity(const Stg::Velocity &V){
            startVelocity.x=V.x;
            startVelocity.y=V.y;
            startVelocity.z=V.z;
            startVelocity.a=V.a;
        }

        // other functions
        ///The function generates path for a base planner
        Path* generate_path();


    };

    class levyWalk_planner : public base_planner{

        /**
         * \brief The class the implements the path generation based on
         * a Levy Walk motion. The class inherits from the base planner class
         *
         */

        Stg::radians_t min=-M_PI;
        Stg::radians_t max=M_PI;

    public:

        // constructors
        levyWalk_planner(): base_planner(){
            /// Default constructor with no arguments

        }

        levyWalk_planner(double pSTime, Stg::Pose P, Stg::Velocity V):
                base_planner(0, pSTime, P, v){
            /**
             * \brief The constructor with parameters
             * In Levy walk the time for the walk is generated from
             * a probability distribution
             */

        }

        // other functions

        Path* generate_path();
    };
}

#endif //STAGE_CTRL_PLUGIN_BASE_PLANNER_H