/*
 * path_planner.h
 *
 *  Created on: 09.11.2018
 *      Author: academy
 */

#ifndef SRC_PATH_PLANNER_H_
#define SRC_PATH_PLANNER_H_


#include <math.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "Spline/spline.h"
#include "util.h"
#include "vehicle.h"

using std::vector;
using namespace std;


class PathPlanner
{
public:
  Vehicle ego;
  vector<Vehicle> vehicles;
  int lane = 1;
  double ref_velocity = 0;

  json previous_path_x ;
  json previous_path_y ;
  // Previous path's end s and d values
  double end_path_s = 0.0;
  double end_path_d = 0.0;

  vector<double> map_waypoints_x_;
    vector<double> map_waypoints_y_;
    vector<double> map_waypoints_s_;
    vector<double> map_waypoints_dx_;
    vector<double> map_waypoints_dy_;


    PathPlanner(vector<double> map_waypoints_x,
          vector<double> map_waypoints_y,
          vector<double> map_waypoints_s,
          vector<double> map_waypoints_dx,
          vector<double> map_waypoints_dy);
    void updateSensorFusion(json data);
    double laneSpeed(int lane);
    int fastestLane();
    double safetyDistance(double speed_mps);
    json path();
    double centerLaneD(int lane);
    double safetyCosts(int lane);
    double wrappedDistance(double s1, double s2);
    vector<Vehicle> vehiclesInFrontInLane(int lane);
};


#endif /* SRC_PATH_PLANNER_H_ */
