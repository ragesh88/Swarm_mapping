/**
 mylevywalker.cpp

 This is a source code for a Levy walk plugin for the stage simulator.
 The plugin enables the robot to perform Levy walk in a bounded domain.
 The robot can have only one laser sensor and fiducial sensor to identify
 its neighbors. The robots also follow a consensus based map sharing
 protocol.
**/


#include "robot/robot.h"

using namespace Stg;

static const double cruisesSpeed = 0.4;
static const double turnSpeed = 0.2;
static const bool verbose = true;
static const bool debug = false;
static const bool record_maps = false;


int8_t newLaserUpdate(Model *mod, myRobot::robot *robot);

int PositionUpdate(Model *mod, myRobot::robot *robot);

int8_t newFiducialUpdate(Model *, myRobot::robot *robot);

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


  auto*robot = new myRobot::robot();

  // Storing the pointer of the dynamically allocated object in a vector
  // This is done that other robots can access the robot data to mimic communication.
  myRobot::robot::swarm_update(robot);


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

  // display the position of the robot for debugging
  if (debug)
    robot->position->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);

  Pose pose = robot->position->GetPose();
  robot->set_current_pose(pose.x, pose.y, pose.z, pose.a);
  robot->position->Subscribe(); // starts the position updates

  // Setting up the planner
  auto* levyWalkPlanner = new myPlanner::levyWalk_planner(0, pose, Stg::Velocity(cruisesSpeed, 0, 0, turnSpeed));

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

  // Looking for fiducial sensors in the model.
  // This required for sharing map among robots using consensus
  printf(" \n looking for a suitable fiducial sensor for \"%s ... \"", robot->position->Token());
  ModelFiducial *fiducial_sensor = nullptr;
  fiducial_sensor = dynamic_cast<ModelFiducial *>(robot->position->GetChild("fiducial:0"));
  if (fiducial_sensor == nullptr) {
    printf("\n Failed to find a fiducial sensor. Exit");
    exit(2);

  }

  robot->fiducial_sensor = fiducial_sensor;
  robot->fiducial_sensor->AddCallback(Model::CB_UPDATE, model_callback_t(newFiducialUpdate), robot);
  robot->fiducial_sensor->Subscribe(); // starts the fiducial sensor update


  return 0;
}

// inspect the ranger data and decide what to do
int8_t newLaserUpdate(Model *, myRobot::robot *robot) {
  robot->build_map();
  try {
    robot->move();
  } catch (const char *a) {
    std::cerr << a << std::endl;
  }

  if (robot->world->Paused() || record_maps) {
    //printf("\n Paused");
    //printf("\n Writing the map");
    if (robot->get_robot_id() == 1 || robot->get_robot_id() == 3)
      robot->write_map();
  }

  return 0;
}

// inspect the fiducial id of the observed robot and decide what to do
int8_t newFiducialUpdate(Model *, myRobot::robot *robot) {

  if (robot->verbose) { // displaying the output of fiducial sensor for debugging
    const auto &fiducials = robot->fiducial_sensor->GetFiducials();
    std::cout << "\n The number of robots detected is : " << fiducials.size() << std::endl;
    if (fiducials.size()) {
      std::cout << "\nThe data of the fiducial sensor of " << robot->get_robot_name();
      std::cout << "\nThe field of view of the fiducial sensor is : " << robot->fiducial_sensor->fov << std::endl;
      std::cout << "\nThe heading of the fiducial sensor is : " << robot->fiducial_sensor->heading << std::endl;
      printf("\n The %s sees the robot with fiducial id %d\n", robot->get_robot_name().c_str(), fiducials[0].id);
    }
  }

  // merge the map
  //robot->merge_map(robots_pointer);
  robot->merge_map();

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

  //Pose pre_pose = robot->previous_pose;
  //printf("Pre Pose: [%.2f %.2f %.2f %.2f]\n", \
    //pre_pose.x, pre_pose.y, pre_pose.z, pre_pose.a);
//    Pose pose = robot->position->GetPose();
//
//    printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);
//    robot->previous_pose.x = pose.x;
//    robot->previous_pose.y = pose.y;
//    robot->previous_pose.a = pose.a;
  return 0; // run again
}
