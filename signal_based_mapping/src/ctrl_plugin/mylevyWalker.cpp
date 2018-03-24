/**
 mylevywalker.cpp

 This is a source code for a Levy walk plugin for the stage simulator.
 The plugin enables the robot to perform Levy walk in a bounded domain.
 The robot can have only one laser sensor.
**/

#include <Stage-4.3/stage.hh>
#include "robot/robot.h"

using namespace Stg;

// random walk step size
static const double step_size = 1.0;
// random walk threshold
static const double thresh = 0.2;
static const double cruisesSpeed = 0.4;
static const double avoidSpeed = 0.05;
static const double avoidTurn = 0.5;
static const double minFrontDistance = 1.0;
static const bool verbose = false;
static const bool debug = true;
static const double stopDist = 0.3;
static const int avoidDuration = 10;



int LaserUpdate(Model *mod, myRobot::robot *robot);
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
  printf("\n*******Ragesh Random walk controller******");
  robot->avoidCount = 0;
  robot->randCount = 0;
  robot->position = dynamic_cast<ModelPosition *>(mod);
  if (!robot->position) {
    PRINT_ERR("No position model given in wander controller.");
    exit(1);
  }
  if (debug)
    robot->position->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);
  robot->previous_pose = robot->position->GetPose();
  robot->position->Subscribe(); // starts the position updates

  // find a range finder

  ModelRanger *laser = NULL;

  printf( "\nWander ctrl for robot %s:\n",  robot->position->Token() );
  for( int i=0; i<16; i++ ) {

      char name[32];
      snprintf( name, 32, "ranger:%d", i ); // generate sequence of model names

      printf( "  looking for a suitable ranger at \"%s:%s\" ... ", robot->position->Token(), name );
      laser = dynamic_cast<ModelRanger *>(robot->position->GetChild( name ));


      if( laser && laser->GetSensors()[0].sample_count > 8 ) {

        printf( "yes." );
	       break;
	      }

      printf( "no." );
    }


    if( !laser ) {
      PRINT_ERR("  Failed to find a ranger with more than 8 samples. Exit.");
      exit(2);
    }

    robot->laser = laser;
    robot->laser->AddCallback(Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
    robot->laser->Subscribe(); // starts the ranger updates
    return 0;
}


// inspect the ranger data and decide what to do
int LaserUpdate(Model *, myRobot::robot *robot)
{
  // get the data
  const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
  uint32_t sample_count = scan.size();
  if(verbose)
      printf("\n sample count laser :%i \n", sample_count);
  if (sample_count < 1)
    return 0;

  bool obstruction = false;
  bool stop = false;
  // find the closest distance to the left and right and check if
  // there's anything in front
  double minleft = 1e6;
  double minright = 1e6;

  for (uint32_t i = 0; i < sample_count; i++) {
    if (verbose)
      printf("%.3f ", scan[i]);

    if ((i > (sample_count / 3)) && (i < (sample_count - (sample_count / 3)))
        && scan[i] < minFrontDistance) {
      if (verbose)
        puts("  obstruction!");
      obstruction = true;
    }

    if (scan[i] < stopDist) {
      if (verbose)
        puts("  stopping!");
      stop = true;
    }

    if (i > sample_count / 2)
      minleft = std::min(minleft, scan[i]);
    else
      minright = std::min(minright, scan[i]);
  }

  if (verbose) {
    puts("");
    printf("minleft %.3f \n", minleft);
    printf("minright %.3f\n ", minright);
  }

  if (obstruction || stop || (robot->avoidCount > 0)) {
    if (verbose)
      printf("Avoid %d\n", robot->avoidCount);

    robot->position->SetXSpeed(stop ? 0.0 : avoidSpeed);

    /* once we start avoiding, select a turn direction and stick
 with it for a few iterations */
    if (robot->avoidCount < 1) {
      if (verbose)
        puts("Avoid START");
      robot->avoidCount = random() % avoidDuration + avoidDuration;

      if (minleft < minright) {
        robot->position->SetTurnSpeed(-avoidTurn);
        if (verbose)
          printf("turning right %.2f\n", -avoidTurn);
      } else {
        robot->position->SetTurnSpeed(+avoidTurn);
        if (verbose)
          printf("turning left %2f\n", +avoidTurn);
      }
    }

    robot->avoidCount--;
  } else {
    if (verbose)
      puts("Cruise");

    robot->avoidCount = 0;
    //robot->position->SetXSpeed(cruisesSpeed);
    //robot->position->SetTurnSpeed(0);

    // Random walk behaviour code
    // motion in x direction
    double noise_x = generateGaussianNoise(1.0);
    double noise_y = generateGaussianNoise(1.0);
    robot->position->SetXSpeed(2*cruisesSpeed*noise_x);
    robot->position->SetYSpeed(2*cruisesSpeed*noise_y);
    robot->position->SetTurnSpeed(0);
    // motion in x direction
    // double randx = double(rand())/RAND_MAX;
    // if(debug){
    //   printf("\n randx : %f", randx);
    // }
    // if (randx > thresh){
    //   robot->position->GoTo(pose.x + step_size, pose.y, pose.a);
    //   if(debug){
    //     puts("\n move + x ");
    //   }
    // }else {
    //   robot->position->GoTo(pose.x - step_size, pose.y, pose.a);
    //   if(debug){
    //     puts("\n move - x ");
    //   }
    // }
    //
    // // motion in y direction
    // double randy = double(rand())/RAND_MAX;
    // if(debug){
    //   printf("\n randy : %f", randy);
    // }
    // if (randy > thresh){
    //   robot->position->GoTo(pose.x, pose.y + step_size, pose.a);
    //   if(debug){
    //     puts("\n move + y ");
    //   }
    // }else {
    //   robot->position->GoTo(pose.x, pose.y - step_size, pose.a);
    //   if(debug){
    //     puts("\n move - y ");
    //   }
    // }


  }

  //  if( robot->pos->Stalled() )
  // 	 {
  // 		robot->pos->SetSpeed( 0,0,0 );
  // 		robot->pos->SetTurnSpeed( 0 );
  // }

  return 0; // run again
}


double generateGaussianNoise(double variance)
{
  static bool haveSpare = false;
  static double rand1, rand2;

  if (haveSpare) {
    haveSpare = false;
    return sqrt(variance * rand1) * sin(rand2);
  }

  haveSpare = true;

  rand1 = rand() / ((double)RAND_MAX);
  if (rand1 < 1e-100)
    rand1 = 1e-100;
  rand1 = -2 * log(rand1);
  rand2 = (rand() / ((double)RAND_MAX)) * M_PI * 2;

  return sqrt(variance * rand1) * cos(rand2);
}

int PositionUpdate(Model *, myRobot::robot *robot)
{
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
