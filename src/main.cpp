#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "FSM.h"

// #define LOG

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	// debug
	// if (seg_s < 0) {
	// 	printf("%s: seg_s: %lf\n", __FUNCTION__, seg_s);
	// }

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	// if (x < 0) {
	// 	printf("%s: x: %lf, seg_x: %lf, d: %lf, perp_heading: %lf\n", __FUNCTION__, x, seg_x, d, perp_heading);
	// }

	return {x,y};

}


vector<vector<double> > getNextWayPoints(double x, double y, double s, double d, double yaw, double ref_vel,
																						const vector<double> &prev_x, const vector<double> &prev_y, 
																						const vector<double> &maps_s, const vector<double> &maps_x, 
																						const vector<double> &maps_y, Lanes lane, bool change) {
	int prev_size = prev_x.size();
	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x = x;
	double ref_y = y;
	double ref_yaw = yaw;

	if (prev_size < 2) {
		double prev_x = x - cos(yaw);
		double prev_car_y = y - sin(yaw);

		ptsx.push_back(prev_x);
		ptsx.push_back(x);
		ptsy.push_back(prev_car_y);
		ptsy.push_back(y);
	}
	else {
		ref_x = prev_x[prev_size-1];
		ref_y = prev_y[prev_size-1];

		double ref_prev_x = prev_x[prev_size-2];
		double ref_prev_y = prev_y[prev_size-2];

		ref_yaw = atan2(ref_y-ref_prev_y, ref_x-ref_prev_x);

		ptsx.push_back(ref_prev_x);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_prev_y);
		ptsy.push_back(ref_y);
	}
  vector<double> next_wp0;
  vector<double> next_wp1;
//  vector<double> next_wp2;
  if (change) {
    next_wp0 = getXY(s + 30, (LANE_WIDTH*lane), maps_s, maps_x, maps_y);
    next_wp1 = getXY(s + 60, (LANE_WIDTH*lane), maps_s, maps_x, maps_y);
//    next_wp2 = getXY(s + 90, (LANE_WIDTH*lane), maps_s, maps_x, maps_y);
  }
  else {
	  next_wp0 = getXY(s + 30, ((LANE_WIDTH/2)+LANE_WIDTH*lane), maps_s, maps_x, maps_y);
    next_wp1 = getXY(s + 60, ((LANE_WIDTH/2)+LANE_WIDTH*lane), maps_s, maps_x, maps_y);
//    next_wp2 = getXY(s + 90, ((LANE_WIDTH/2)+LANE_WIDTH*lane), maps_s, maps_x, maps_y);
  }
	
	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
//	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
//	ptsy.push_back(next_wp2[1]);

	for (int i = 0; i < ptsx.size(); ++i) {
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;
		ptsx[i] = (shift_x * cos(-ref_yaw)) - (shift_y * sin(-ref_yaw));
		ptsy[i] = (shift_x * sin(-ref_yaw)) + (shift_y * cos(-ref_yaw));
	}

	tk::spline sp;
	sp.set_points(ptsx, ptsy);

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	for (int i = 0; i < prev_x.size(); ++i) {
		next_x_vals.push_back(prev_x[i]);
		next_y_vals.push_back(prev_y[i]);
	}

	double target_x = 50.0;
	double target_y = sp(target_x);
	double target_distance = distance(0.0, 0.0, target_x, target_y);

	double x_add_on = 0;
	for (int i = prev_x.size(); i < 50; ++i) {
		double N = target_distance / (0.02 * ref_vel/2.24);
		double x_pt = target_x / N + x_add_on;
		double y_pt = sp(x_pt);
		x_add_on = x_pt;

		double x_ref = x_pt;
		double y_ref = y_pt;

		x_pt = (x_ref * cos(ref_yaw)) - (y_ref * sin(ref_yaw));
		y_pt = (x_ref * sin(ref_yaw)) + (y_ref * cos(ref_yaw));

		x_pt += ref_x;
		y_pt += ref_y;

		next_x_vals.push_back(x_pt);
		next_y_vals.push_back(y_pt);
	}

	return {next_x_vals, next_y_vals};
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
  string map_file_ = "data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

	Lanes lane = MIDDLE;
	States state = KEEP_LANE;
  States prev_state = state;
	double ref_vel = 2.5;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &state, &prev_state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;
						FSM fsm(state, lane);
           	bool change = false;
						state = fsm.GetNextState(sensor_fusion, car_s, car_d, previous_path_x.size());
#ifdef LOG
              std::ofstream file("data_dump.log", std::ios_base::app);
              file << "state: " << state << endl;
							file.close();
#endif
						if (state == TOO_CLOSE) {
							ref_vel -= 0.224;
						}
						else if (state == SHIFT_LEFT) {
							lane = Lanes((int)lane - 1);
							// ref_vel -= 2.24*5;
              change = true;
						}
						else if (state == SHIFT_RIGHT) {
							lane = Lanes((int)lane + 1);
							// ref_vel -= 2.24*5;
              change = true;
						}
						else if (state == KEEP_LANE && ref_vel < 49.5) {
							ref_vel += 0.224;
						}

						if (previous_path_x.size() > 2) {
						 	car_s = end_path_s;
						}

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
						vector<vector<double> > next_vals = getNextWayPoints(car_x, car_y, car_s, car_d, car_yaw, ref_vel, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y, (Lanes)lane, change);

            if (prev_state != state) {
              prev_state = state;
#ifdef LOG
              std::ofstream file("data_dump.log", std::ios_base::app);
//              file << "state: " << state << endl;
              file << "lane: " << lane << endl;
              file << "ref vel: " << ref_vel << endl;
              // file << "next vals: " << endl;

              // for (int i = 0; i < next_vals.size(); ++i) {
              //   for (int j = 0; j < next_vals[i].size(); ++j) {
              //     file << next_vals[i][j] << "\t";  
              //   }
              //   file << endl;
              // }
              file.close();
#endif
            }

          	msgJson["next_x"] = next_vals[0];
          	msgJson["next_y"] = next_vals[1];

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
