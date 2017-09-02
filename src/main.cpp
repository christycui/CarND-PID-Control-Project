#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Resetting the Simulator
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws){
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(),msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(1, 1, 1);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    int n = 100;
    int param_int = -1;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          if (pid.found_best_error == false) {
            // use twiddle to find best PID gains
            if (pid.found_error == false) {
              steer_value = pid.Steer(cte);
              if (pid.t > 2*n) {
                pid.error = pid.error / n; // average the error
                pid.found_error = true;
              } else if (pid.t > n && pid.t <= 2*n) {
                pid.error = pid.error + cte*cte;
              }
            } else {
              // iterate through kp, ki, kd
              for (int i=0; i<pid.dp.size(); i++) {
                param_int = -1;
                if (pid.twiddle_mark[i] == false) {
                  param_int = i;
                  break;
                }
              }
              
              if (param_int == -1) {
                // restart iteration again
                pid.twiddle_mark = {false, false, false};
                pid.twiddle_up = {false, false, false};
                pid.twiddle_down = {false, false, false};
                param_int = 0;
              }
              
              if (pid.p[0] + pid.p[1] + pid.p[2] > pid.tolerance) {
                if (pid.twiddle_up[param_int] == false) {
                  pid.p[param_int] += pid.dp[param_int];
                  reset_simulator(ws);
                  pid.Init(pid.p[0], pid.p[1], pid.p[2]);
                  steer_value = pid.Steer(cte);
                  pid.twiddle_up[param_int] = true;
                } else if (pid.twiddle_down[param_int] == false) {
                  if (pid.error < pid.best_error) {
                    pid.best_error = pid.error;
                    pid.dp[param_int] = pid.dp[param_int] * 1.1;
                    pid.twiddle_mark[param_int] = true;
                  } else {
                    pid.p[param_int] -= 2*pid.dp[param_int];
                    pid.twiddle_down[param_int] = true;
                    reset_simulator(ws);
                    pid.Init(pid.p[0], pid.p[1], pid.p[2]);
                    steer_value = pid.Steer(cte);
                  }
                } else {
                  if (pid.error < pid.best_error) {
                    pid.best_error = pid.error;
                    pid.dp[param_int] = pid.dp[param_int] * 1.1;
                    pid.twiddle_mark[param_int] = true;
                  } else {
                    // add back if not best err
                    pid.p[param_int] = pid.p[param_int] + pid.dp[param_int];
                    pid.dp[param_int] = pid.dp[param_int] * 0.9;
                    pid.twiddle_mark[param_int] = true;
                  }
                }
              } else {
                // best params are found
                pid.found_best_error = true;
                std::cout << "best_err: " << pid.best_error << std::endl;
              }
            }
          } else {
            // drive
            steer_value = pid.Steer(cte);
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } 
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
