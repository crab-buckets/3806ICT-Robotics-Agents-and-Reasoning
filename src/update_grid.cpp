#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/ros.h>
#include <streambuf>
#include <fstream>
#include <map>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "assignment_3/UpdateGrid.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SetModelState.h"
#include "assignment_3/Sensor.h"
#include "communal_defines.cpp"

// width of each grid (currently set to 1m)
#define GRID_WIDTH 1.0

// ROS service client so sensors have access to bot position
ros::ServiceClient bot_location;
gazebo_msgs::GetModelState srv;

// current grid is always compared with new grid when updating gazebo to move/delete models
int currentGrid[BOARD_H][BOARD_W];

// 2D array of points used as an interface into the positioning system in gazebo
geometry_msgs::Point coordinates[BOARD_H][BOARD_W];

// global access to the spawn, delete, and set services
ros::ServiceClient spawnClient;
ros::ServiceClient deleteClient;
ros::ServiceClient setClient;

// initialise number of spawned survivors and hostiles
int numSurvivors = 0;
int numHostiles = 0;
// initialise check to see if submarine has already been spawned (its moved if already spawned)
bool submarineSpawned = false;
// home directory and model directory
std::string homeDir = getenv("HOME");
std::string modelDir = homeDir + "/catkin_ws/src/3806ICT-Robotics-Agents-and-Reasoning/models/";

// comparison function for Point
struct ComparePoints
{
	bool operator()(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) const
	{
		if (p1.x != p2.x)
			return p1.x < p2.x;
		if (p1.y != p2.y)
			return p1.y < p2.y;
		return p1.z < p2.z;
	}
};

// Dictionary with coordinates as key and survivor/hostile as value
std::map<geometry_msgs::Point, std::string, ComparePoints> objectPositions;

// -- function declarations --
// takes a model type and returns a spawn model request (gazebo_msgs::SpawnModel)
gazebo_msgs::SpawnModel createSpawnRequest(int modelType, geometry_msgs::Point position);
// updates gazebo with the new positions of objects, spawns them if non-existent
bool updateGrid(assignment_3::UpdateGrid::Request &req, assignment_3::UpdateGrid::Response &res);
// simulates something like a sonar sensor which detects objects. Can take variable sensorRange
// and returns an array of detected objects, corresponding to the distance away from the bot's
// current position in east, north, west, south. The bot's current position is taken from
// gazebo get_model_state in an attempt to model a real sensor
bool obstacleSensor(assignment_3::Sensor::Request &req, assignment_3::Sensor::Response &res);
// simulates something like a infrared sensor which detects objects. Can take variable sensorRange
// and returns an array of detected objects, corresponding to the distance away from the bot's
// current position in east, north, west, south. The bot's current position is taken from
// gazebo get_model_state in an attempt to model a real sensor
bool survivorSensor(assignment_3::Sensor::Request &req, assignment_3::Sensor::Response &res);

// main
int main(int argc, char **argv)
{
	// initialise the node
	ros::init(argc, argv, "gazebo_object_manager");
	ros::NodeHandle n;
	// creating a client to the get_model_state service, so that the sensors have access to the bot's
	// x, y coordinates at all times for emulation purposes (to try simulate actual sensors)
	bot_location = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	// pretend we're a delivery bot :)
	srv.request.model_name = "delivery_bot";

	// Initialize current grid to all empty squares
	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
			currentGrid[i][j] = EMPTY;

	// Initialise coordinates
	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
		{
			coordinates[i][j].x = i * GRID_WIDTH;
			coordinates[i][j].y = j * GRID_WIDTH;
			coordinates[i][j].z = 0;
		}

	// create client for set_model_state to update model states when moving objects
	setClient = n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
	// create client for spawning models when generating the initial board
	spawnClient = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	// create client for deleting models when updating/moving objcets
	deleteClient = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
	// advertise the hostile sensor emulator for the robot to call when required
	ros::ServiceServer HostileSenService = n.advertiseService("obstacle_sensor", obstacleSensor);
	// advertise the survivor sensor emulator for the robot to call when required
	ros::ServiceServer SurvivorSenService = n.advertiseService("survivor_sensor", survivorSensor);
	// advertise the update_grid sensor emulator. This is responsible for updating gazebo
	// with new locations of the objects
	ros::ServiceServer updateGridService = n.advertiseService("update_grid", updateGrid);
	ros::spin();
	return 0;
}

gazebo_msgs::SpawnModel createSpawnRequest(int modelType, geometry_msgs::Point position)
{
    gazebo_msgs::SpawnModel spawn;
    std::string modelPath;
    if (modelType == ORDER_PICKUP) {
        spawn.request.model_name = "bowl" + std::to_string(numSurvivors);
        numSurvivors++;
        modelPath = modelDir + "bowl/model.sdf";
    } else if (modelType == OBSTACLE) {
        spawn.request.model_name = "cardboard_box" + std::to_string(numHostiles);
        numHostiles++;
        modelPath = modelDir + "cardboard_box/model.sdf";
    } else if (modelType == DELIVERY_BOT) {
        spawn.request.model_name = "delivery_bot";
        modelPath = modelDir + "turtlebot3_burger/model.sdf";
    }
    std::ifstream t(modelPath);
    if (!t.is_open()) {
        ROS_WARN("Could not open model file: %s", modelPath.c_str());
        exit(1);
    }
    std::string modelXml((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    t.close();
    spawn.request.model_xml = modelXml;
    spawn.request.initial_pose.position = position;
    return spawn;
}

// updateGrid: update logic for delivery bot scenario
bool updateGrid(assignment_3::UpdateGrid::Request &req, assignment_3::UpdateGrid::Response &res)
{
    std_msgs::Int32MultiArray read_grid = req.grid;
    gazebo_msgs::SetModelState set;
    gazebo_msgs::DeleteModel del;
    gazebo_msgs::SpawnModel spawn;
    for (int i = 0; i < BOARD_H; ++i)
        for (int j = 0; j < BOARD_W; ++j) {
            int oldIndex = currentGrid[i][j];
            int newIndex = read_grid.data[i * BOARD_W + j];
            if (oldIndex != newIndex) {
                geometry_msgs::Point point = coordinates[i][j];
                if (oldIndex == EMPTY && newIndex == ORDER_PICKUP) {
                    spawn = createSpawnRequest(ORDER_PICKUP, point);
                    objectPositions[point] = spawn.request.model_name;
                    bool success = spawnClient.call(spawn);
                    if (!success) {
                        ROS_ERROR("Failed to spawn ORDER_PICKUP model: %s", spawn.request.model_name.c_str());
                    }
                }
                if (oldIndex == ORDER_PICKUP && newIndex == DELIVERY_BOT) {
                    del.request.model_name = objectPositions[point];
                    deleteClient.call(del);
                    objectPositions.erase(point);
                    set.request.model_state.model_name = "delivery_bot";
                    set.request.model_state.pose.position = point;
                    setClient.call(set);
                }
                if (oldIndex == EMPTY && newIndex == OBSTACLE) {
                    spawn = createSpawnRequest(OBSTACLE, point);
                    objectPositions[point] = spawn.request.model_name;
                    bool success = spawnClient.call(spawn);
                    if (!success) {
                        ROS_ERROR("Failed to spawn OBSTACLE model: %s", spawn.request.model_name.c_str());
                    }
                }
                if ((oldIndex == EMPTY || oldIndex == VISITED) && newIndex == DELIVERY_BOT) {
                    if (submarineSpawned) {
                        ROS_INFO("Moving delivery bot to pos: (%.0f, %.0f)", point.x, point.y);
                        set.request.model_state.model_name = "delivery_bot";
                        set.request.model_state.pose.position = point;
                        setClient.call(set);
                    } else {
                        spawn = createSpawnRequest(DELIVERY_BOT, point);
                        objectPositions[point] = spawn.request.model_name;
                        bool success = spawnClient.call(spawn);
                        if (!success) {
                            ROS_ERROR("Failed to spawn DELIVERY_BOT model: %s", spawn.request.model_name.c_str());
                        }
                        submarineSpawned = true;
                    }
                }
                currentGrid[i][j] = newIndex;
            }
        }
    res.altered_grid = req.grid;
    return true;
}

bool obstacleSensor(assignment_3::Sensor::Request &req, assignment_3::Sensor::Response &res)
{
    res.objectNorth = false;
    res.objectSouth = false;
    res.objectWest = false;
    res.objectEast = false;
    res.objectDetected = false;
    if (!bot_location.call(srv)) {
        ROS_WARN("Failed to call ROS GetModelState service to get bot location");
        return false;
    }
    int x = std::round(srv.response.pose.position.x);
    int y = std::round(srv.response.pose.position.y);
    int range = req.sensorRange;
    res.northRadar = std::vector<int32_t>(range, 0);
    res.southRadar = std::vector<int32_t>(range, 0);
    res.eastRadar = std::vector<int32_t>(range, 0);
    res.westRadar = std::vector<int32_t>(range, 0);
    for (int i = 1; i <= range; ++i) {
        if (x - i >= 0 && currentGrid[x - i][y] == OBSTACLE) {
            res.objectNorth = true;
            res.northRadar[i - 1] = 1;
        }
        if (x + i < BOARD_H && currentGrid[x + i][y] == OBSTACLE) {
            res.objectSouth = true;
            res.southRadar[i - 1] = 1;
        }
        if (y - i >= 0 && currentGrid[x][y - i] == OBSTACLE) {
            res.objectWest = true;
            res.westRadar[i - 1] = 1;
        }
        if (y + i < BOARD_W && currentGrid[x][y + i] == OBSTACLE) {
            res.objectEast = true;
            res.eastRadar[i - 1] = 1;
        }
    }
    if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
        res.objectDetected = true;
    return true;
}

bool survivorSensor(assignment_3::Sensor::Request &req, assignment_3::Sensor::Response &res)
{
    res.objectNorth = false;
    res.objectSouth = false;
    res.objectWest = false;
    res.objectEast = false;
    res.objectDetected = false;
    if (!bot_location.call(srv)) {
        ROS_WARN("Failed to call ROS GetModelState service to get bot location");
        return false;
    }
    int x = std::round(srv.response.pose.position.x);
    int y = std::round(srv.response.pose.position.y);
    int range = req.sensorRange;
    res.northRadar = std::vector<int32_t>(range, 0);
    res.southRadar = std::vector<int32_t>(range, 0);
    res.eastRadar = std::vector<int32_t>(range, 0);
    res.westRadar = std::vector<int32_t>(range, 0);
    for (int i = 1; i <= range; ++i) {
        if (x - i >= 0 && currentGrid[x - i][y] == ORDER_PICKUP) {
            res.objectNorth = true;
            res.northRadar[i - 1] = 1;
        }
        if (x + i < BOARD_H && currentGrid[x + i][y] == ORDER_PICKUP) {
            res.objectSouth = true;
            res.southRadar[i - 1] = 1;
        }
        if (y - i >= 0 && currentGrid[x][y - i] == ORDER_PICKUP) {
            res.objectWest = true;
            res.westRadar[i - 1] = 1;
        }
        if (y + i < BOARD_W && currentGrid[x][y + i] == ORDER_PICKUP) {
            res.objectEast = true;
            res.eastRadar[i - 1] = 1;
        }
    }
    if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
        res.objectDetected = true;
    return true;
}