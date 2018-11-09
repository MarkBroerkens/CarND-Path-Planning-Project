/*
 * path_planner.c
 *
 *  Created on: 09.11.2018
 *      Author: academy
 */

#include "path_planner.h"
#include "config.h"
#include "algorithm"
#include "util.h"

PathPlanner::PathPlanner(vector<double> map_waypoints_x,
                         vector<double> map_waypoints_y,
                         vector<double> map_waypoints_s,
                         vector<double> map_waypoints_dx,
                         vector<double> map_waypoints_dy) {
   map_waypoints_x_ = map_waypoints_x;
   map_waypoints_y_ = map_waypoints_y;
   map_waypoints_s_ = map_waypoints_s;
   map_waypoints_dx_ = map_waypoints_dx;
   map_waypoints_dy_ = map_waypoints_dy;

}

void PathPlanner::updateSensorFusion(json data) {
  // initialize ego vehicle
  ego = Vehicle(data, true);

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  vehicles.clear();
  json sensor_fusion = data[1]["sensor_fusion"];
  for (json sensor_fusion_vehicle : sensor_fusion) {
    vehicles.push_back(Vehicle(sensor_fusion_vehicle, false));
  }

  // Previous path data given to the Planner
  previous_path_x = data[1]["previous_path_x"];
  previous_path_y = data[1]["previous_path_y"];
  // Previous path's end s and d values
  end_path_s = data[1]["end_path_s"];
  end_path_d = data[1]["end_path_d"];
}

double PathPlanner::laneSpeed(int lane) {
  double lane_speed = milesPerHourToMetersPerSecond(MAX_SPEED_MPH);
  // TODO: handle wrap after TRACK_LENGTH_M
  for (Vehicle vehicle : vehicles) {
    if (vehicle.lane == lane) {
      double distance = vehicle.s - ego.s;
      if ((distance > 0) && distance < FRONT_SENSOR_RANGE_M
          && vehicle.speed < lane_speed) {
        lane_speed = vehicle.speed;
      }
    }
  }
  return lane_speed;

}

// figure out the fastes lane of the current and the direct neighbours
int PathPlanner::fastestLane() {
  int fastest_lane = ego.lane;
  double fastest_speed = laneSpeed(fastest_lane);

//  int lane_up = ego.lane + 1;
//  if (lane_up < NUMBER_OF_LANES) {
//    double lane_speed = laneSpeed(lane_up);
//    if (lane_speed > fastest_speed) {
//      fastest_speed = lane_speed;
//      fastest_lane = lane_up;
//    }
//  }
//  int lane_down = ego.lane - 1;
//  if (lane_down >= 0) {
//    double lane_speed = laneSpeed(lane_down);
//    if (lane_speed > fastest_speed) {
//      fastest_speed = lane_speed;
//      fastest_lane = lane_down;
//    }
//  }

  for (int lane = 0; lane<NUMBER_OF_LANES ; lane++) {
    double lane_speed = laneSpeed(lane);
    if (lane_speed > fastest_speed) {
      fastest_speed = lane_speed;
      fastest_lane = lane;
    }
  }
  return fastest_lane;
}

double PathPlanner::centerLaneD(int lane) {
  return LANE_WIDTH*(0.5 + lane);
}

double PathPlanner::safetyDistance(double speed_mps) {
  // see http://www.softschools.com/formulas/physics/stopping_distance_formula/89/
  double reaction_distance = speed_mps * REACTION_TIME_S;
  double brake_distance = (speed_mps * speed_mps)
      / (2 * CAR_COEFFICIENT_OF_FRICION * CAR_ACCELERATION_DUE_TO_GRAVITY_MPS2);
  return reaction_distance + brake_distance;
}

json PathPlanner::path() {

  // define the actual (x,y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  int prev_size = previous_path_x.size();

  double car_s = ego.s;

  if (prev_size>0) {
    car_s = end_path_s;
  }

  bool too_close = false;

  // find ref_v to use
  for (Vehicle vehicle : vehicles) {
    if (vehicle.lane == ego.lane) {
      double check_speed = vehicle.speed;
      double check_car_s = vehicle.s;

      std::cout << "ego s " << car_s << std::endl;
      std::cout << "prev_size " << prev_size << std::endl;
      std::cout << "vehicle s " << check_car_s << std::endl;
      check_car_s+=((double)prev_size*0.02*check_speed);
      std::cout << "vehicle s2 " << check_car_s << std::endl;


      if ((check_car_s > car_s) && ((check_car_s-car_s) < safetyDistance(ego.speed))) {
        int fastest_lane = fastestLane();
        std::cout << "Fastest Lane: " << fastest_lane << std::endl;
        if (fastest_lane > lane) {
          lane +=1;
        } else if (fastest_lane < lane) {
          lane -=1;
        }

        too_close = true;
      }

    }
  }

  if (too_close) {
        ref_velocity -= 0.224;
      } else if (ref_velocity < MAX_SPEED_MPH) {
        ref_velocity += 0.224;
      }

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = ego.yaw_rad;

  if (prev_size < 2) {
    double prev_car_x = ego.x - cos(ego.yaw);
    double prev_car_y = ego.y - sin(ego.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ego.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ego.y);
  } else {
    ref_x = previous_path_x[prev_size -1];
    ref_y = previous_path_y[prev_size -1];
    double ref_x_prev = previous_path_x[prev_size -2];
    double ref_y_prev = previous_path_y[prev_size -2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x - ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

  }

  double destinationD = centerLaneD(lane);
  vector<double> next_wp0 = getXY(car_s + 40, destinationD, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp1 = getXY(car_s + 80, destinationD, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp2 = getXY(car_s + 120, destinationD, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i<ptsx.size(); i++) {
    // shift car reference angle to 0 degree
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
  }

  // create a spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx,ptsy);



  // start with the previous path points from last time
  for (int i=0; i<previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 40.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

  double x_add_on = 0;

  // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points

  for (int i = 1; i<= NUMBER_OF_PATH_POINTS - previous_path_x.size(); i++) {


    double N = (target_dist/(TICK_S*milesPerHourToMetersPerSecond(ref_velocity)));  // each 0.02 seconds a new point is reached, transform miles per hour to m/s
    double x_point = x_add_on+(target_x)/N;
    double y_point = s(x_point);
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotating back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));;

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);

  }

  json msgJson;
  msgJson["next_x"] = next_x_vals;
  msgJson["next_y"] = next_y_vals;

  return msgJson;

}

