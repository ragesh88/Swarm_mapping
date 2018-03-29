//
// Created by Ragesh on 3/20/18.
/**
 This is the header file for robot.cpp
 The file has the basic functionality for the controlling
 a robot in the stage simulator.

  */
//


#ifndef STAGE_CTRL_PLUGIN_ROBOT_H
#define STAGE_CTRL_PLUGIN_ROBOT_H

// stage header file added for some functions
#include <Stage-4.3/stage.hh>
#include <random>

namespace myRobot{

    /// success flag
    const bool SUCCESS = true;
    /// failed flag
    const bool FAILED = false;

    enum LEVY_MODE{
        /**
         * The various modes for levy motion
         */
        /// Cruise mode
        CRUISE,
        /// Rotation mode
        ROTATION,
        /// Start mode
        START
    };


    class robot {
        /// name of the robot
        std::string robot_name = "Robot";
        /// current pose of the robot
        Stg::Pose current_pose = Stg::Pose(0, 0, 0, 0);
        /// current velocity of the robot
        Stg::Velocity current_velocity = Stg::Velocity(0, 0, 0, 0);
        /// current linear speed
        double current_speed = 0.0;
        /// turn speed
        double turn_speed = 0.0;
        /// pose at previous time step of the robot
        Stg::Pose past_pose = Stg::Pose(0, 0, 0, 0);
        /// velocity at previous time step of the robot
        Stg::Velocity past_velocity = Stg::Velocity(0, 0, 0, 0);
        /// time when past pose was recorded
        Stg::msec_t past_pose_time = Stg::msec_t(0);
        /// minimum distance for the levy flight in meters
        Stg::meters_t levy_min = Stg::meters_t{1};
        /// alpha value for the levy flight
        double levy_alpha = 1.5;
        /// the distance generated by the levy distribution
        double levy_dis = 0.0;
        /// the total time required to cover the levy distance
        double levy_total_time = 0.0;
        /// the total time required to reach the levy direction
        double levy_dir_time = 0.0;
        /// the start time for the levy flight
        double levy_start_time = 0.0;
        /// The time for the robot should perform a levy cruise
        Stg::usec_t levy_cruise_time=0;

    public:

        // Stage simulator variables
        /// Access the position model of the Stage library
        Stg::ModelPosition *position;
        /// Access the laser sensor model of the Stage library
        Stg::ModelRanger *laser;
        /// Obstacle avoidance variable
        int avoidCount, randCount;
        Stg::Pose previous_pose;
        /// The pointer to the world object
        Stg::World *world;
        /// The modes for levy flight
        LEVY_MODE mode=START;
        /// The levy direction for the flight
        Stg::radians_t desired_levy_direction=0;



        // constructor

        robot(std::string name, Stg::Pose i_pose, Stg::Velocity i_vel, Stg::meters_t l_min, double l_alpha, double l_start_time)
             : robot_name{name},
               current_pose{i_pose.x, i_pose.y, i_pose.z, i_pose.a},
               current_velocity{i_vel.x, i_vel.y, i_vel.x, i_vel.a},
               levy_min{l_min},
               levy_alpha{l_alpha},
               levy_start_time{l_start_time}    {

            /// The constructor of the class

            /**
             * \param name : Name of the robot as a C-str
             * \param i_pose : Initial pose of the robot of as Stg::Pose object
             * \param i_vel : Initial velocity of the robot of as Stg::Velocity object
             * \param l_min : Minimum distance for the levy flight in meters
             * \param l_alpha : The distance generated by the levy distribution
             * \param l_start_time : The start time for the levy flight
             */

            current_speed = std::sqrt(current_velocity.x * current_velocity.x + \
                             current_velocity.y * current_velocity.y + \
                             current_velocity.z * current_velocity.z);
            turn_speed = current_velocity.a;

        }


        robot(){
            /// Default constructor with no arguments
        }

        // get functions

        std::string get_robot_name() const {
            /// Returns the name of the robot
            return robot_name;
        }

        Stg::Pose get_current_pose() const {
            /// Returns the current pose as Stg::Pose object
            return current_pose;
        }

        Stg::Pose get_past_pose() const {
            /// Returns the past pose as Stg::Pose object
            return past_pose;
        }

        Stg::Velocity get_current_velocity() const {
            /// Returns the current velocity as Stg::Velocity object
            return current_velocity;
        }

        Stg::Velocity get_past_velocity() const {
            /// Returns past velocity as Stg::Velocity object
            return past_velocity;
        }

        Stg::msec_t get_past_pose_time() const {
            /// Returns the past pose time as Stg::msec_t
            return past_pose_time;
        }

        double get_levy_alpha() const {
            /// Returns the exponent of the levy flight
            return levy_alpha;
        }

        Stg::meters_t get_levy_min() const {
            /// Return the minimum levy distance
            return levy_min;
        }

        Stg::meters_t get_levy_dis() const {
            /// Returns the levy distance
            return levy_dis;
        }

        double get_levy_total_time() const {
            /// Return total levy translation time
            return levy_total_time;
        }

        double get_levy_dir_time() const {
            /// Return total levy rotation time
            return levy_dir_time;
        }

        double get_levy_start_time() const {
            /// Return start time of the levy flight
            return levy_start_time;
        }

        double get_current_linear_speed() const {
            /// Return the 2 - norm of the linear velocity
            return std::sqrt(current_velocity.x * current_velocity.x + \
                             current_velocity.y * current_velocity.y + \
                             current_velocity.z * current_velocity.z);
        }

        double get_levy_cruise_time() const {
            /// Returns the time for levy translation
            return levy_cruise_time;
        }

        // set functions

        void set_robot_name(std::string name){
            /// Set the name of the robot
            /**
             * \param name : the robot name
             */
            robot_name = name;
        }

        void set_current_pose(Stg::meters_t x, Stg::meters_t y, Stg::meters_t z, Stg::radians_t a){
            /// Set the current pose of the robot using all the parameters
            /**
              * \param x : x coordinate in meters
              * \param y : y coordinate in meters
              * \param z : z coordinate in meters
              * \param a : a angle in radians
              */
            current_pose = Stg::Pose(x, y, z, a);
        }

        void set_current_pose(Stg::meters_t x, Stg::meters_t y, Stg::radians_t a){
            /// Set the current pose of the robot using x y and a
            /**
             * Here z is set to zero
              * \param x : x coordinate in meters
              * \param y : y coordinate in meters
              * \param a : a angle in radians
              */
            current_pose = Stg::Pose(x, y, 0, a);
        }

        void set_past_pose(Stg::meters_t x, Stg::meters_t y, Stg::meters_t z, Stg::radians_t a){
            /// Set the past pose using all the parameters
            /**
             * \param x : x coordinate in meters
             * \param y : y coordinate in meters
             * \param z : z coordinate in meters
             * \param a : a angle in radians
             */
            past_pose = Stg::Pose(x, y, z, a);
        }

        void set_past_pose(Stg::meters_t x, Stg::meters_t y, Stg::radians_t a){
            /// Set the past pose using x y and a
            /**
             * Here z is set to zero
             * \param x : x coordinate in meters
             * \param y : y coordinate in meters
             * \param a : a angle in radians
             */
            past_pose = Stg::Pose(x, y, 0, a);
        }

        void set_current_velocity(double x, double y, double z, double a){
            /// Set the current velocity using all the parameters
            /**
             * \param x : velocity vector component along X axis (forward speed), in meters per second.
             * \param y : velocity vector component along Y axis (sideways speed), in meters per second.
             * \param z : velocity vector component along Z axis (vertical speed), in meters per second.
             * \param a : rotational velocity around Z axis (yaw), in radians per second.
             */
            current_velocity = Stg::Velocity(x, y, z, a);
            current_speed  = std::sqrt(current_velocity.x * current_velocity.x + \
                             current_velocity.y * current_velocity.y + \
                             current_velocity.z * current_velocity.z);
            turn_speed = current_velocity.a;
        }

        void set_current_velocity(double x, double y, double a){
            /// Set the current velocity using x, y and a parameters
            /**
             * Here z is set to zero
             * \param x : velocity vector component along X axis (forward speed), in meters per second.
             * \param y : velocity vector component along Y axis (sideways speed), in meters per second.
             * \param a : rotational velocity around Z axis (yaw), in radians per second.
             */
            current_velocity = Stg::Velocity(x, y, 0, a);
            current_speed  = std::sqrt(current_velocity.x * current_velocity.x + \
                             current_velocity.y * current_velocity.y + \
                             current_velocity.z * current_velocity.z);
            turn_speed = current_velocity.a;
        }

        void set_past_velocity(double x, double y, double z, double a){
            /// Set past velocity as using all the parameters
            /**
            * \param x : velocity vector component along X axis (forward speed), in meters per second.
            * \param y : velocity vector component along Y axis (sideways speed), in meters per second.
            * \param z : velocity vector component along Z axis (vertical speed), in meters per second.
            * \param a : rotational velocity around Z axis (yaw), in radians per second.
             */
            past_velocity = Stg::Velocity(x, y, z, a);
        }

        void set_past_velocity(double x, double y, double a){
            /// Set past velocity as using  x, y and a parameters
            /**
             * Here z is set to zero
             * \param x : velocity vector component along X axis (forward speed), in meters per second.
             * \param y : velocity vector component along Y axis (sideways speed), in meters per second.
             * \param a : rotational velocity around Z axis (yaw), in radians per second.
             */
            past_velocity = Stg::Velocity(x, y, 0, a);
        }

        void set_past_pose_time(Stg::msec_t time){
            /// Set the past pose time as Stg::msec_t
            /**
             * \param time : an object of Stg::msec_t
             */
            past_pose_time = time;
        }

        void set_levy_alpha(double alpha){
            /// Set the exponent of the levy flight
            /**
             * \param alpha : levy distribution parameter
             */
            levy_alpha = alpha;
        }

        void set_levy_min(Stg::meters_t x_min){
            /// Set the minimum levy distance
            /**
             * \param x_min : the minimum levy distance that can be taken in meters
             */
            levy_min = x_min;
        }

        void set_levy_total_time(double time){
            /// Set total levy time
            /**
             * \param time : current simulation time
             */
            levy_total_time = time + levy_cruise_time;
        }

        void set_levy_start_time(double time){
            /// Set start time of the levy flight
            /**
             * \param time : the start time of levy flight
             */
            levy_start_time = time;
        }

        bool set_levy_cruise_time(double speed) {
            /// Compute the levy flight time based on the given speed
            /**
             * \param speed : the translation speed of the robot
             */

            try {
                if(speed == 0){
                    throw "\nspeed should be non zero\n";
                }
            }
            catch (const char* a){
                std::cout<<a;
                return FAILED;
            }
            levy_cruise_time = levy_dis/speed;
            return SUCCESS;
        }


        // Other functions

        bool generate_levy_dist();

        Stg::radians_t generate_random_direction(Stg::radians_t min = -M_PI, Stg::radians_t max = M_PI);



    };

}


#endif //STAGE_CTRL_PLUGIN_ROBOT_H
