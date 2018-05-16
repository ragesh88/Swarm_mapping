//
// Created by Ragesh on 5/15/18.
//

/**
 myMILevywalker.cpp

 This is a source code for a plugin for the stage simulator.
 The plugin enables the robot to perform Levy walk in a bounded domain,
 which is directed based on a reward function
 that pick the direction which as maximum information gain.
 The robot can have only one laser sensor and fiducial sensor to identify
 its neighbors. The robots also follow a consensus based map sharing
 protocol.
**/

#include "robot/robot.h"

using namespace Stg;
// Some global variables as parameters
static const double cruisesSpeed = 0.4;
static const double turnSpeed = 0.2;
static const bool verbose = true;
static const bool debug = false;
static const bool record_maps = false;


int8_t newLaserUpdate(Model *mod, myRobot::robot *robot);

int PositionUpdate(Model *mod, myRobot::robot *robot);

int8_t newFiducialUpdate(Model *, myRobot::robot *robot);


// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *) {


  auto*robot = new myRobot::robot();

  // Storing the pointer of the dynamically allocated object in a vector
  // This is done that other robots can access the robot data to mimic communication.
  myRobot::robot::swarm_update(robot);


  printf("\n\n**************Ragesh Levy walk controller assignment*************");
  robot->set_current_velocity(cruisesSpeed, 0, turnSpeed);
  robot->world = mod->GetWorld();
  robot->avoidCount = 0;
  robot->randCount = 0;
  robot->position = dynamic_cast<ModelPosition *>(mod);

  if (!robot->position) {
    PRINT_ERR("No position model given for the controller.");
    exit(1);
  }


  Pose pose = robot->position->GetPose();
  robot->set_current_pose(pose.x, pose.y, pose.z, pose.a);
  robot->position->Subscribe(); // starts the position updates

  // forward sensor model parameter
  myPlanner::F_S_M_parameters fsm{};

  // Setting up the Mutual information based planner
  auto* MIlevyWalkPlanner = new myPlanner::MI_levyWalk_planner(0, pose, Stg::Velocity(cruisesSpeed, 0, 0, turnSpeed));

  if (levyWalkPlanner == NULL)
    printf("NO Planner generated");

  robot->set_planner(levyWalkPlanner);

  // The parameters for map object
  const double min_x = -8; // in meters
  const double min_y = -8; // in meters
  const double cell_size_x = 0.02; // in meters
  const double cell_size_y = 0.02; // in meters
  const int n_cell_x = 800; // no of cells along x
  const int n_cell_y = 800; // no of cells along y

  // Setting up the map object
  auto *occ_grid = new occupancy_grid::occupancyGrid2D<double, int>(min_x, min_y,
                                                                    cell_size_x, cell_size_y,
                                                                    n_cell_x, n_cell_y);
  if (occ_grid == nullptr)
    printf("No map object created");

  // Assigning the map object to robot
  robot->occ_grid_map = occ_grid;

  // find a range finder

  ModelRanger *laser = nullptr;

  printf("\n  Ragesh Levy walk controller assignment for robot %s initiated \n", robot->position->Token());
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

  // Looking for fiducial sensors in the model.
  // This required for sharing map among robots using consensus
  printf(" \n  looking for a suitable fiducial sensor for \"%s\" ... ", robot->position->Token());
  ModelFiducial *fiducial_sensor = nullptr;
  fiducial_sensor = dynamic_cast<ModelFiducial *>(robot->position->GetChild("fiducial:0"));
  if (fiducial_sensor == nullptr) {
    printf("\n Failed to find a fiducial sensor. Exit");
    exit(2);

  }
  printf("found one");
  robot->fiducial_sensor = fiducial_sensor;
  robot->fiducial_sensor->AddCallback(Model::CB_UPDATE, model_callback_t(newFiducialUpdate), robot);
  robot->fiducial_sensor->Subscribe(); // starts the fiducial sensor update

  printf("\n************************Process completed************************");
  return 0;
}