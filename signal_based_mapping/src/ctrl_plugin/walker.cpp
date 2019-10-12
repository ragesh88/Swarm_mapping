//
// Created by ragesh on 10/11/19.
//

/**
 walker.cpp

 This is a source code for a plugin for the stage simulator.
 The plugin enables the robot to perform normal walker

 In particular the program generate the true map of the domain
**/

#include "robot/robot.h"
#include <boost/algorithm/string.hpp>
#include "json/json.hpp"

using namespace Stg;
using json = nlohmann::json;

// Some global variables as parameters
static const double cruisesSpeed = 0.4;
static const double turnSpeed = 0.2;
double quit_time = 1000; // denoting 7200 seconds
uint no_of_robots=0;
std::string trail;
std::string map_name;
static const bool verbose = true;
static const bool debug = false;
static bool record_maps = false;

// some flags for computation
bool control_verbose=true;
bool compute_entropy=false;
bool compute_coverage=false;
bool display_final_entropy=false;
bool display_final_coverage=false;
bool write_txt_map=false;
bool write_img_map=true;

// paths for storing data
std::string data_path_entropy;
std::string data_path_coverage;
std::string data_path_img_map;
std::string data_path_txt_map;


int8_t newLaserUpdate(Model *mod, myRobot::robot *robot);

int PositionUpdate(Model *mod, myRobot::robot *robot);

int8_t newFiducialUpdate(Model *, myRobot::robot *robot);


// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs * args) {


    // local arguments
//    printf( "\nMI Levy walk controller controller initialised with:\n"
//      "\tworldfile string \"%s\"\n"
//      "\tcmdline string \"%s\"",
//      args->worldfile.c_str(),
//      args->cmdline.c_str() );


    // The parameters for map object (cave map)
    double min_x = -8; // in meters
    double min_y = -8; // in meters
    double cell_size_x = 0.02; // in meters
    double cell_size_y = 0.02; // in meters
    int n_cell_x = 800; // no of cells along x
    int n_cell_y = 800; // no of cells along y

    // The parameters for map object (frieburg map)
//  const double min_x = -20; // in meters
//  const double min_y = -20; // in meters
//  const double cell_size_x = 0.02; // in meters
//  const double cell_size_y = 0.02; // in meters
//  const int n_cell_x = 2000; // no of cells along x
//  const int n_cell_y = 2000; // no of cells along y

    // create a robot object as a dynamic one
    auto*robot = new myRobot::robot(mod);

    // Storing the pointer of the dynamically allocated object in a vector
    // This is done that other robots can access the robot data to mimic communication.
    myRobot::robot::swarm_update(robot);



    // parse the data from the json file if it exist
    ///////////////////////////////////////////////////////////
    if (args->cmdline.length()){
        std::vector<std::string> results;
        boost::split(results, args->cmdline, [](char c){ return c==' ';});
        std::string json_file; // json file
        auto j = find(results.begin(), results.end(), "-j");
        auto t = find(results.begin(), results.end(), "-t");
        // found a trial option
        if(t != results.end()){
            trail = *(t+1);
        }
        // found a json file parse the data
        if(j != results.end()){
            json_file = *(j+1);
            // parsing the data
            std::ifstream jread(json_file);
            json j_obj;
            jread >> j_obj;
            // setting up the parameter based on the parameter json file
            map_name = j_obj["Map_name"];
            quit_time = j_obj["quit_time"];
            no_of_robots = j_obj["no_of_robots"];
            control_verbose = static_cast<bool>(j_obj["control_verbose"]);
            robot->verbose=control_verbose;
            record_maps = static_cast<bool>(j_obj["record_maps"]);
            compute_entropy = static_cast<bool>(j_obj["compute_entropy"]);
            compute_coverage = static_cast<bool>(j_obj["compute_coverage"]);
            display_final_coverage = static_cast<bool>(j_obj["display_final_coverage"]);
            display_final_entropy = static_cast<bool>(j_obj["display_final_entropy"]);
            write_img_map = static_cast<bool>(j_obj["write_img_map"]);
            write_txt_map = static_cast<bool>(j_obj["write_txt_map"]);
            if(compute_entropy){
                data_path_entropy = j_obj["data_path_entropy"];
            }
            if(compute_coverage){
                data_path_coverage = j_obj["data_path_coverage"];
            }
            if(write_img_map){
                data_path_img_map = j_obj["data_path_img_map"];
            }
            if(write_txt_map){
                data_path_txt_map = j_obj["data_path_txt_map"];
            }
            // setting up the map settings from the file
            min_x = j_obj["floorplan"]["min_x"];
            min_y = j_obj["floorplan"]["min_y"];
            cell_size_x = j_obj["floorplan"]["cell_size_x"];
            cell_size_y = j_obj["floorplan"]["cell_size_y"];
            n_cell_x = j_obj["floorplan"]["n_cell_x"];
            n_cell_y = j_obj["floorplan"]["n_cell_y"];
        }

    }
    ///////////////////////////////////////////////////////////




    robot->set_current_velocity(cruisesSpeed, 0, turnSpeed);
    robot->world = mod->GetWorld();
    if(control_verbose){
        printf("\n\n**************Ragesh MI Levy walk controller assignment*************");
        std::cout<<"\n  The fiducial return is : "<<mod->GetFiducialReturn();
        std::cout<<"\n  The robot id is :"<<robot->get_robot_id()<<std::endl;
    }

    // check if fiducial return is same and robot id
    if(mod->GetFiducialReturn() != robot->get_robot_id()){
        std::cout<<"robot id and fiducial return not same"<<std::endl;
        exit(0);
    }
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

    if(control_verbose){
        printf("\n  Ragesh MI Levy walk controller assignment for robot %s initiated \n", robot->position->Token());
    }

    for (int i = 0; i < 16; i++) {

        char name[32];
        snprintf(name, 32, "ranger:%d", i); // generate sequence of model names
        if (control_verbose){
            printf("  looking for a suitable ranger at \"%s:%s\" ... ", robot->position->Token(), name);
        }
        laser = dynamic_cast<ModelRanger *>(robot->position->GetChild(name));

        if (laser && laser->GetSensors()[0].sample_count > 8) {
            if(control_verbose){
                printf("yes.");
            }
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

    // forward sensor model parameter
    myPlanner::F_S_M_parameters fsm;
    fsm.sigma = std::sqrt(robot->laser->GetSensors()[0].range_noise_const);

    // Setting up the Mutual information based planner
//    auto* MIlevyWalkPlanner = new myPlanner::MI_levyWalk_planner(0, pose, Stg::Velocity(cruisesSpeed, 0, 0, turnSpeed),
//                                                                 fsm, myPlanner::KLDMI, 5);
//
//    if (MIlevyWalkPlanner == nullptr)
//        printf("NO Planner generated");
//
//    robot->set_planner(MIlevyWalkPlanner);

    // Setting a planner
    auto* planner = new myPlanner::base_planner(50, 0, pose, Stg::Velocity(cruisesSpeed, 0, 0, turnSpeed));

    if (planner == nullptr)
        printf("Base planner not generated");
    robot->set_planner(planner);


    // Looking for fiducial sensors in the model.
    // This required for sharing map among robots using consensus
    if(control_verbose){
        printf(" \n  looking for a suitable fiducial sensor for \"%s\" ... ", robot->position->Token());
    }
    ModelFiducial *fiducial_sensor = nullptr;
    fiducial_sensor = dynamic_cast<ModelFiducial *>(robot->position->GetChild("fiducial:0"));
    if (fiducial_sensor == nullptr) {
        printf("\n Failed to find a fiducial sensor. Exit");
        exit(2);

    }
    if(control_verbose){
        printf("found one");
    }

    robot->fiducial_sensor = fiducial_sensor;
    robot->fiducial_sensor->AddCallback(Model::CB_UPDATE, model_callback_t(newFiducialUpdate), robot);
    robot->fiducial_sensor->Subscribe(); // starts the fiducial sensor update

    if (control_verbose){
        printf("\n*************************Process completed**************************");
    }



    return 0;
}

// inspect the ranger data and decide what to do
int8_t newLaserUpdate(Model *, myRobot::robot *robot) {
    robot->build_map();
    // the compute the map entropy at that instant
    if(compute_entropy){
        robot->add_map_entropy();
    }
    // the compute the percentage of the map covered till that instant
    if(compute_coverage){
        robot->add_map_coverage();
    }
    try {
        robot->move();
    } catch (const char *a) {
        std::cerr << a << std::endl;
    }

    if ((robot->world->Paused() || record_maps)) {
        //printf("\n Paused");
        //printf("\n Writing the map");
        if (robot->get_robot_id() == 1 || robot->get_robot_id() == 3) {
            //std::cout<<" \nData from robot "<<robot->get_robot_id()<<std::endl;
            robot->write_map();
//      std::cout << "\n The map percentage coverage is : " << robot->occ_grid_map->compute_map_coverage();
//      std::cout << "\n The entropy of the map is : " << robot->occ_grid_map->compute_map_entropy();
//      std::string path{"./robot"};
//      robot->write_map_entropy(path + std::to_string(robot->get_robot_id()) + "/");
//      robot->write_map_coverage(path + std::to_string(robot->get_robot_id()) + "/");
//      std::cout << std::endl;
        }
    }

    if (std::fabs(robot->world->SimTimeNow()/ 1000000.0 - quit_time) < robot->world->sim_interval/ 1000000.0 ){
        if(control_verbose){
            std::cout<<"\n sim time :"<<robot->world->SimTimeNow()/ 1000000.0;
            std::cout<<"\nsimulation finished\n";
            std::cout<<" \n Data from robot "<<robot->get_robot_id()<<std::endl;
        }
        std::string path{"./robot"};
        std::string prefix{map_name + "_Qt_" + std::to_string(uint(quit_time)) + "_Rs_" + std::to_string(no_of_robots)};
        if(trail.length()){
            prefix +=  "_Tr_" + trail + "_";
        }
        // write the map as an image
        if(write_img_map){
            if(data_path_img_map.length()){
                if(trail.length()){
                    robot->write_map(data_path_img_map, prefix);
                } else{
                    robot->write_map(data_path_img_map);
                }
            } else{
                if(trail.length()){
                    robot->write_map(path + std::to_string(robot->get_robot_id()) + "/", prefix);
                } else{
                    robot->write_map(path + std::to_string(robot->get_robot_id()) + "/");
                }
            }
            //robot->write_map();
        }
        // write the map as a text file
        if(write_txt_map){
            if(data_path_txt_map.length()){
                if(trail.length()){
                    robot->write_map_txt(data_path_txt_map, prefix);
                } else{
                    robot->write_map_txt(data_path_txt_map);
                }
            } else{
                if(trail.length()){
                    robot->write_map_txt(path + std::to_string(robot->get_robot_id()) + "/", prefix);
                } else{
                    robot->write_map_txt(path + std::to_string(robot->get_robot_id()) + "/");
                }
            }
            //robot->write_map_txt();
        }

        if (display_final_coverage){
            printf("\n %f %f", quit_time, robot->occ_grid_map->compute_map_coverage());
        }

        if (compute_coverage){
            if(control_verbose){
                std::cout << "\n The map percentage coverage is : " << robot->occ_grid_map->compute_map_coverage();
            }
            // write the percentage of coverage at various times
            if(data_path_coverage.length()){
                if(trail.length()){
                    robot->write_map_coverage(data_path_coverage, prefix);
                }else{
                    robot->write_map_coverage(data_path_coverage);
                }
            } else{
                if(trail.length()){
                    robot->write_map_coverage(path + std::to_string(robot->get_robot_id()) + "/", prefix);
                } else{
                    robot->write_map_coverage(path + std::to_string(robot->get_robot_id()) + "/");
                }
            }
            //robot->write_map_coverage(path + std::to_string(robot->get_robot_id()) + "/");
        }

        if (display_final_entropy){
            printf("\n %f %f", quit_time, robot->occ_grid_map->compute_map_entropy());
        }

        if(compute_entropy){
            if(control_verbose){
                std::cout << "\n The entropy of the map is : " << robot->occ_grid_map->compute_map_entropy();
            }
            // write the entropy of map at various times
            if(data_path_entropy.length()){
                if(trail.length()){
                    robot->write_map_entropy(data_path_entropy, prefix);
                }else{
                    robot->write_map_entropy(data_path_entropy);
                }
            } else{
                if(trail.length()){
                    robot->write_map_entropy(path + std::to_string(robot->get_robot_id()) + "/", prefix);
                } else{
                    robot->write_map_entropy(path + std::to_string(robot->get_robot_id()) + "/");
                }
            }
            //robot->write_map_entropy(path + std::to_string(robot->get_robot_id()) + "/");
        }

        if(control_verbose){
            std::cout << std::endl;
        }

    }
    return 0;
}

// inspect the fiducial id of the observed robot and decide what to do
int8_t newFiducialUpdate(Model *, myRobot::robot *robot) {

    if (robot->verbose) { // displaying the output of fiducial sensor for debugging
        const auto &fiducials = robot->fiducial_sensor->GetFiducials();
        std::cout << "\n The number of robots detected is : " << fiducials.size() << std::endl;
        if (!fiducials.empty()) {
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