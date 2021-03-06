#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Clip between range.
double clipVal(double val, double ll, double ul) {
	double result = 0.0;
	if (val < ll) {
		result = ll;
	}
	else if (val > ul) {
		result = ul;
	}
	else {
		result = val;
	}
	return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  double steer_value = j[1]["steering_angle"];
		  steer_value = -steer_value;
		  double throttle_value = j[1]["throttle"];
		  double Lf = 2.67;
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

		  // Shift car reference
		  for (int i = 0; i < ptsx.size(); i++) {
			  double shiftx = ptsx[i] - px;
			  double shifty = ptsy[i] - py;

			  ptsx[i] = (shiftx * cos(0 - psi)) - (shifty * sin(0 - psi));
			  ptsy[i] = (shiftx * sin(0 - psi)) + (shifty * cos(0 - psi));

		  }
		  // Fit a polynomial to x and y coordindates
		  double* ptrx = &ptsx[0];
		  Eigen::Map<Eigen::VectorXd> ptsxT(ptrx, ptsx.size());
		  double* ptry = &ptsy[0];
		  Eigen::Map<Eigen::VectorXd> ptsyT(ptry, ptsy.size());
		  auto coeffs = polyfit(ptsxT, ptsyT, 3);

		  // Calculate Current Cross Track and Orientation Errors
		  double cte = polyeval(coeffs, 0); // This is for no latency case
		  double epsi = -atan(coeffs[1]); // This is for no latency case
		  
		  // Calculate Future Cross Track and Orientation Errors
		  double delayT = 0.1;
		  double ctet1 = cte + (v * sin(epsi) * delayT); // This is for the latency case
		  double epsit1 = epsi + (v * steer_value * delayT) / Lf; // This is for the latency case

		  // Predict x, y, psi and v to account for latency in 100ms
		  px = 0 + (v * cos(0) * delayT);
		  py = 0 + (v * sin(0) * delayT);
		  psi = 0 + (v * steer_value * delayT) / Lf;
		  v = v + (throttle_value * delayT);

		  // Create State Vector
		  Eigen::VectorXd state(6);
		  // state << 0, 0, 0, v, cte, epsi; // This is for no latency case
		  state << px, py, psi, v, ctet1, epsit1; // This is for the latency case

		  // Solve MPC Optimization Problem 
		  auto vars = mpc.Solve(state, coeffs);

		  // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
		  // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
		  steer_value = vars[0]/(deg2rad(25) * Lf);
		  throttle_value = vars[1];

		  steer_value = clipVal(steer_value, -1.0, 1.0);
		  throttle_value = clipVal(throttle_value, -1.0, 1.0);

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
		  for (int i = 2; i < vars.size(); i++) {
			  if (i % 2 == 0) {
				  mpc_x_vals.push_back(vars[i]);
			  }
			  else {
				  mpc_y_vals.push_back(vars[i]);
			  }
		  }
		  
          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
		  double wayPinc = 2;
		  int nPoints = 25;
		  for (int i = 1; i < nPoints; i++) {
			  next_x_vals.push_back(wayPinc * i);
			  next_y_vals.push_back(polyeval(coeffs, wayPinc * i));
		  }

		  // Push data to simulator
		  json msgJson;
		  msgJson["steering_angle"] = steer_value;
		  msgJson["throttle"] = throttle_value;

		  msgJson["mpc_x"] = mpc_x_vals;
		  msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
		  
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
