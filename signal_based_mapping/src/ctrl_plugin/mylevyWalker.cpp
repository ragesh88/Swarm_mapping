/**
 mylevywalker.cpp

 This is a source code for a Levy walk plugin for the stage simulator.
 The plugin enables the robot to perform Levy walk in a bounded domain.
 The robot can have only one laser sensor.
**/

#include <Stage-4.3/stage.hh>
#include "robot/robot.h"

using namespace Stg;


static const double cruisesSpeed = 0.4;
static const double turnSpeed = 0.2;
static const double avoidSpeed = 0.05;
static const double avoidTurn = 0.5;
static const double minFrontDistance = 1.0;
static const bool verbose = false;
static const bool verbose_new = true;
static const bool debug = false;
static const double stopDist = 0.3;
static const int avoidDuration = 10;
static const double tol = 3;


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
    robot->laser->AddCallback(Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
    robot->laser->Subscribe(); // starts the ranger updates
    return 0;
}


// inspect the ranger data and decide what to do
int LaserUpdate(Model *, myRobot::robot *robot) {
    // get the data
    const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
    uint32_t sample_count = scan.size();
    if (verbose)
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
            robot->set_levy_total_time(0);
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
        robot->set_levy_total_time(0);
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

        // Levy walk behaviour code
        // motion generation
        static radians_t desired_direction=0;
        if(robot->get_levy_total_time() == 0) {
            if(verbose_new)
                puts("\nrotation mode\n");
            desired_direction = robot->generate_random_direction();
            assert(robot->generate_levy_dist());
            Stg::usec_t trans_time = robot->get_levy_dis()/cruisesSpeed;
            Pose current = robot->position->GetPose();
            Stg::usec_t simTime = robot->world->SimTimeNow();
            if(verbose_new){
                printf("\n Desired direction %f",(desired_direction/(M_PI))*180);
                printf("\n Current direction %f",(current.a/(M_PI))*180);
                printf("\n Current pose %f, %f, %f",current.x, current.y, (current.a/(M_PI))*180);
                printf("\n Current sim time %" PRId64 ,simTime);
            }
            if(verbose_new){
                printf("\n angle difference 1 %f \n", (desired_direction-current.a)/(M_PI)*180);
                printf("\n angle difference 2 %f \n", (std::fabs(2*M_PI - desired_direction) + current.a)/(M_PI)*180);
                printf("\n angle difference 3 %f \n", (std::fabs(2*M_PI - current.a) + desired_direction)/(M_PI)*180);
            }
            // find the smallest angle to rotate
            if(std::fabs(desired_direction-current.a) < std::fabs(2*M_PI - desired_direction) + current.a || \
            std::fabs(desired_direction-current.a) < std::fabs(2*M_PI - current.a) + desired_direction) {
                robot->position->SetXSpeed(0);
                if (desired_direction > current.a){
                    robot->position->SetTurnSpeed(+turnSpeed);
                    if(verbose_new){
                        printf("\n angle difference 1  turn + direction \n");
                    }
                } else{
                    robot->position->SetTurnSpeed(-turnSpeed);
                    if(verbose_new){
                        printf("\n angle difference 1  turn - direction \n");
                    }
                }
                Stg::usec_t rot_time = std::fabs(desired_direction-current.a)/turnSpeed;
                robot->set_levy_total_time(simTime+trans_time+rot_time);
            } else{
                robot->position->SetXSpeed(0);
                if (std::fabs(2*M_PI - desired_direction) + current.a < std::fabs(2*M_PI - current.a) + desired_direction){
                    robot->position->SetTurnSpeed(-turnSpeed);
                    if(verbose_new){
                        printf("\n angle difference 2  turn - direction \n");
                    }
                } else{
                    robot->position->SetTurnSpeed(+turnSpeed);
                    if(verbose_new){
                        printf("\n angle difference 3  turn + direction \n");
                    }
                }
                robot->set_levy_total_time(simTime+trans_time);
            }
        } else{
            if(verbose_new){
                Pose p = robot->position->GetPose();
                printf("\n Current pose before cruise %f, %f, %f\n",p.x, p.y, (p.a/(M_PI))*180);
            }
            if(std::fabs(desired_direction-robot->position->GetPose().a)/(M_PI)*180 < tol) {
                if(verbose_new)
                    puts("\nCruise mode\n");
                robot->position->SetTurnSpeed(0);
                robot->position->SetXSpeed(cruisesSpeed);
            }
            if(std::fabs(robot->world->SimTimeNow() - robot->get_levy_total_time()) < tol){
                robot->set_levy_total_time(0);
            }
        }


    }

    //  if( robot->pos->Stalled() )
    // 	 {
    // 		robot->pos->SetSpeed( 0,0,0 );
    // 		robot->pos->SetTurnSpeed( 0 );
    // }

    return 0; // run again
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
