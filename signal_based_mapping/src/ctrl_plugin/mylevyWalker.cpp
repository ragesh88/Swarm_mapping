/**
 mylevywalker.cpp

 This is a source code for a Levy walk plugin for the stage simulator.
 The plugin enables the robot to perform Levy walk in a bounded domain.
 The robot can have only one laser sensor.
**/

#include <Stage-4.3/stage.hh>
#include "robot/robot.h"
#include "planner/base_planner.h"

using namespace Stg;



static const double cruisesSpeed = 0.4;
static const double turnSpeed = 0.2;
static const double avoidSpeed = 0.05;
static const double avoidTurn = 0.5;
static const double minFrontDistance = 1.0;
static const bool verbose = true;
static const bool verbose_new = true;
static const bool debug = false;


int8_t newLaserUpdate(Model *mod, myRobot::robot *robot);

int PositionUpdate(Model *mod, myRobot::robot *robot);

double generateGaussianNoise(double variance);


// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *) {


    // local arguments
    /*  printf( "\nWander controller initialised with:\n"
        "\tworldfile string \"%s\"\n"
        "\tcmdline string \"%s\"",
        args->worldfile.c_str(),
        args->cmdline.c_str() );
  */

    myRobot::robot *robot = new myRobot::robot();
    printf("\n*******Ragesh Levy walk controller******");
    robot->set_current_velocity(cruisesSpeed, 0, turnSpeed);
    robot->world = mod->GetWorld();
    robot->avoidCount = 0;
    robot->randCount = 0;
    robot->position = dynamic_cast<ModelPosition *>(mod);
    if (!robot->position) {
        PRINT_ERR("No position model given in wander controller.");
        exit(1);
    }
    if (debug)
        robot->position->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);
    Pose pose = robot->position->GetPose();
    robot->set_current_pose(pose.x, pose.y, pose.z, pose.a);
    robot->position->Subscribe(); // starts the position updates

    // Setting up the planner
    myPlanner::levyWalk_planner* levyWalkPlanner = new myPlanner::levyWalk_planner(0, pose, Stg::Velocity(cruisesSpeed, 0, 0, turnSpeed));

    if(verbose)
        if (levyWalkPlanner == NULL)
            printf("NO Planner generated");

    robot->set_planner(levyWalkPlanner);

    // find a range finder

    ModelRanger *laser = NULL;

    printf("\nWander ctrl for robot %s:\n", robot->position->Token());
    for (int i = 0; i < 16; i++) {

        char name[32];
        snprintf(name, 32, "ranger:%d", i); // generate sequence of model names

        printf("  looking for a suitable ranger at \"%s:%s\" ... ", robot->position->Token(), name);
        laser = dynamic_cast<ModelRanger *>(robot->position->GetChild(name));


        if (laser && laser->GetSensors()[0].sample_count > 8) {

            printf("yes.");
            break;
        }

        printf("no.");
    }


    if (!laser) {
        PRINT_ERR("  Failed to find a ranger with more than 8 samples. Exit.");
        exit(2);
    }

    robot->laser = laser;
    robot->laser->AddCallback(Model::CB_UPDATE, model_callback_t(newLaserUpdate), robot);
    robot->laser->Subscribe(); // starts the ranger updates
    return 0;
}



// inspect the ranger data and decide what to do
int8_t newLaserUpdate(Model *, myRobot::robot *robot) {
    robot->move();
    return 0;
}


double generateGaussianNoise(double variance) {
    static bool haveSpare = false;
    static double rand1, rand2;

    if (haveSpare) {
        haveSpare = false;
        return sqrt(variance * rand1) * sin(rand2);
    }

    haveSpare = true;

    rand1 = rand() / ((double) RAND_MAX);
    if (rand1 < 1e-100)
        rand1 = 1e-100;
    rand1 = -2 * log(rand1);
    rand2 = (rand() / ((double) RAND_MAX)) * M_PI * 2;

    return sqrt(variance * rand1) * cos(rand2);
}

int PositionUpdate(Model *, myRobot::robot *robot) {
    Pose pre_pose = robot->previous_pose;
    printf("Pre Pose: [%.2f %.2f %.2f %.2f]\n", \
  pre_pose.x, pre_pose.y, pre_pose.z, pre_pose.a);
    Pose pose = robot->position->GetPose();

    printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);
    robot->previous_pose.x = pose.x;
    robot->previous_pose.y = pose.y;
    robot->previous_pose.a = pose.a;
    return 0; // run again
}
