#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "spline.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// function to return if left lane if free and lane change is possible
bool laneChangeLeft(vector<vector<double>> sensor_fusion,int lane,double car_s,int prev_size)
{

    std::vector<double> cars_in_lane{100.0, 100.0};

    // set focal lane as one to the right of current lane
    int new_lane = lane-1;

    // if lane is right or centre lane
    if(new_lane >= 0)
    {

      // loop through sensor detections
      for(int i=0; i <sensor_fusion.size();i++)
      {

        float d = sensor_fusion[i][6];

        // if check car is in potential lane
        if(d < (2+4*new_lane+2) && d > (2+4*new_lane-2))
        {
            int id = sensor_fusion[i][0];
            double  vx = sensor_fusion[i][3];
            double vy= sensor_fusion[i][4];
            double check_speed_front = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s+= ((double)prev_size *.02*check_speed_front);

            // if is in front and smaller distance value than already stored
            if((check_car_s >= car_s) && ((check_car_s-car_s) < cars_in_lane[0]))
            {
              cars_in_lane[0] = check_car_s-car_s;
            }
            // else car is rearward and smaller distance value than already stored
            else if((check_car_s < car_s) && ((car_s-check_car_s) < cars_in_lane[1]))
            {
              cars_in_lane[1] = car_s-check_car_s;
            }
        }

      }

      // if cars are greater than 40m forward and greater than 20m rear of ego
      if ((cars_in_lane[0] >= 35) && (cars_in_lane[1] >= 15))
      {
        // allow a lane change
        return true;
      }
    }

    // not safe for lane change
    return false;
}

// function to return if right lane if free and lane change is possible
bool laneChangeRight(vector<vector<double>> sensor_fusion,int lane,double car_s,int prev_size)
{

    std::vector<double> cars_in_lane{100.0, 100.0};

    // set focal lane as one to the right of current lane
    int new_lane = lane+1;

    // if lane is right or centre lane
    if(new_lane <= 2)
    {

      // loop through sensor detections
      for(int i=0; i <sensor_fusion.size();i++)
      {

        float d = sensor_fusion[i][6];

        // if check car is in potential lane
        if(d < (2+4*new_lane+2) && d > (2+4*new_lane-2))
        {
            int id = sensor_fusion[i][0];
            double  vx = sensor_fusion[i][3];
            double vy= sensor_fusion[i][4];
            double check_speed_front = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s+= ((double)prev_size *.02*check_speed_front);

            // if is in front and smaller distance value than already stored
            if((check_car_s >= car_s) && ((check_car_s-car_s) < cars_in_lane[0]))
            {
              cars_in_lane[0] = check_car_s-car_s;
            }
            // else car is rearward and smaller distance value than already stored
            else if((check_car_s < car_s) && ((car_s-check_car_s) < cars_in_lane[1]))
            {
              cars_in_lane[1] = car_s-check_car_s;
            }
        }

      }

      // if cars are greater than 40m forward and greater than 20m rear of ego
      if ((cars_in_lane[0] >= 35) && (cars_in_lane[1] >= 15))
      {
        // allow a lane change
        return true;
      }
    }

    // not safe for lane change
    return false;
}

// function to get lane
int getLane(double d){

    // initialise variable to hold lane set to invalid value
    int lane = -1;

    if (d >= 0 and d < 4)
    {
      // set lane as left
      lane = 0;
    }
    else if (d >= 4 and d < 8)
    {
      // set lane as centre
      lane  = 1;
    }
    else {
      // set lane as right
      lane = 2;
    }
    // return lane position
    return lane;
}

// fucntion to check if a vehicle is ahead of ego vehicle in same lane
int carAhead(vector<vector<double>> sensor_fusion,int ego_lane,double car_s,int prev_size)
{
  // loop through sensor detecions
  for(int i=0; i <sensor_fusion.size();i++)
    {
        // get the lateral position of the detected vehicle
        float d = sensor_fusion[i][6]; // other car d position in frenet

        // check if detection is in same lane as ego
        if(d < (2 + 4 * ego_lane + 2) && d > (2 + 4 * ego_lane - 2))
        {
          float car_ID = sensor_fusion[i][0];
          // confirm speed of other vehicle
          double vx = sensor_fusion[i][3]; // other car x vel in m/s
          double vy = sensor_fusion[i][4]; // other car y vel  in m/s
          // calculate other vehicle relative speed - get magnitude
          double check_speed = sqrt(vx*vx+vy*vy);
          // get the distance other car is from our car
          double check_car_s = sensor_fusion[i][5]; // other car s position in frenet

          // check speed can help predict where car will be in future
          // project s value out over time
          check_car_s+=((double)prev_size*0.02*check_speed); // if using previous points car is not there yet

          // if deteced car is in front of ego AND is too close; set flag
          if ((check_car_s > car_s) && (check_car_s - car_s) < 30)
            {
              // return car ID of vehicle in front of ego - to enable track and follow
              return car_ID;
            }
        }
      }

      // return a large number to indicate no car in front of ego
      return 1000;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);


  }

    // lane initialisation - start in middle lane - 0:Left, 1:Centre, 2:Right
    int lane = 1;

    // specify reference velocity - speed limit is 50mph
    double ref_vel = 0.0; // start at 0 mph and increment to avoid jerk

    // flag to allow lane change
    bool lane_change = false;

    // flag to steady a lane change before performing next
    bool lane_steady = false;




  h.onMessage([&ref_vel, &lane, &lane_change, &lane_steady, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // previous path size to support performing a transition
          int prev_size = previous_path_x.size();

          // using sensor fusion to avoid collisions

          // if path points exist
          if (prev_size > 0)
          {
            // change car to frenet coordinates
            // set car t previous list's last point's frenet s value
            car_s = end_path_s;
          }

          // variables to hold other car positions
          int car_in_front = -1;
          bool car_right = false;
          bool car_left = false;
          std::vector<bool> car_left_or_right = {false, false};

          // get ego lane position
          int ego_lane = getLane(car_d);

          // steady lane position
          if (ego_lane == lane)
          {
            // make sure car is more centralised in lane before allowing another manouvre
            if (car_d < (1 + 4 * ego_lane + 1) && car_d > (1 + 4 * ego_lane - 1))
            {
              // confirm ego car is steady in lane position
              lane_steady = true;

            }
          }

          // check if car is in front of ego - set flag if too close
          car_in_front = carAhead(sensor_fusion, ego_lane, car_s, prev_size);

          // take action if car in front
          if (car_in_front < 1000)
          {

            // set a flag to check for lane change
            lane_change = true;

            // decrease speed
            // get tracked ID speed and set target speed
            double follow_vx = sensor_fusion[car_in_front][3]; // other car x vel in m/s
            double follow_vy = sensor_fusion[car_in_front][4]; // other car y vel  in m/s
            // calculate other vehicle relative speed - get magnitude
            double target_speed = sqrt(follow_vx*follow_vx+follow_vy*follow_vy);

            double target_car_s = sensor_fusion[car_in_front][5]; // other car s position in frenet

            // check speed can help predict where car will be in future
            // project s value out over time
            target_car_s+=((double)prev_size*0.02*target_speed);

            // slow ego car if vehicle is ahead
            if ((target_car_s - car_s) < 20)
            {
              // slow car down
              ref_vel -= 0.05;

            }
            // start to slow car if vehicle is far ahead
            else if ((target_car_s - car_s) < 40)
            {
              // slow car down to avoid future heavy braking
              ref_vel -= 0.08;
            }
            // simple car follower - try to track speed of car in front if stuck in smae lane
            else if (ref_vel < (target_speed * 2.237))
            {
              // speed up
              ref_vel += 0.3; // 0.224 = 5m/s
            }

            // lane change check
            if ((lane_change) && (lane_steady))
            {
              // set flag if clear in left lane
              car_left = laneChangeLeft(sensor_fusion,lane,car_s,prev_size);
              // set flag if clear in right lane
              car_right = laneChangeRight(sensor_fusion,lane,car_s,prev_size);

              // check to see if lane change is possible and implement move
              if (lane_change && car_left)
              {
                // set target lane to one lane left
                lane = lane - 1;
                // set flag of lane change back to false
                lane_change = false;
                // flag to indicate if lane manouevre is in progress
                lane_steady = false;

              } else if (lane_change && car_right)
              {
                // set target lane to one lane right
                lane = lane + 1;
                // set flag of lane change back to false
                lane_change = false;
                // flag to indicate if lane manouvre is in progress
                lane_steady = false;

              }
            }

          }

          // if no car in front and speed is below max speed
          else if (ref_vel < 49.7)
          {
            // accelerate up to max speed
            ref_vel += 0.224;
          }

          // create list of evenly spaced waypoints evenly spaced at 30m
          // this sparse list will later be interpolated with a spline to get a higher density of points
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y, yaw
          // either use referenc or previous point to establish start position
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is near empty then use the car as starting reference
          if (prev_size < 2) {
              //use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          }
          else
          {
              //use previous paths end point as starting reference

              // redefine reference state as previous path and point
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

              //use two points that make the path tangent to the car
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }


          // In frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, 2.0 + 4.0 * lane ,map_waypoints_s,map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,2.0 + 4.0 * lane,map_waypoints_s,map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,2.0 + 4.0 * lane,map_waypoints_s,map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // convert from global to local coordinates
          for (int i = 0; i < ptsx.size(); i++) {
              // shift car reference angle to 0 degrees
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;
              // transform
              ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;
          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // define x,y points that will be used for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all the points from previous path
          for (int i = 0; i < prev_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so car travels at desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);

          double x_add_on = 0;

          // fill in rest of path planner
          for (int i = 0; i < 50 - prev_size; i++) {

              // calculate spacing of points along vector distance .. mph > m/s
              double N = (target_dist/(0.02*ref_vel/2.24));
              // generate x points from vecotr spaing
              double x_point = x_add_on+(target_x)/N;
              // interpolate spline to obtain y point from x
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back to global
              x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }

          json msgJson;
          // planned path x,y
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
