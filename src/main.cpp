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

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid_steering;
  PID pid_throttle;
  
  float Kp_st = 0.09;
  float Ki_st = 0.000001;
  float Kd_st = 2.5;
  float set_speed_in = 80;
    
  if(argc > 1)
  {
    Kp_st = atof(argv[1]);
    if(argc > 2)
    {
      Ki_st = atof(argv[2]);
      if(argc > 3)
      {
        Kd_st = atof(argv[3]);
        if(argc > 4)
        {
          set_speed_in = atof(argv[4]);
        }
      }
    }
  }

  // TODO: Initialize the pid variable.
  pid_steering.Init(Kp_st, Ki_st, Kd_st, -10, 10, false, 0.005);
  pid_throttle.Init(0.5, 0.002, 1, -10.0, 10.0, false, 0.001);

  h.onMessage([&pid_steering, &pid_throttle, &set_speed_in](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double steer_value;
          

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          
          steer_value = pid_steering.UpdateError(cte);
          
          if(steer_value > 1)
            steer_value = 1;
          if(steer_value < -1)
            steer_value = -1;
            
          if(fabs(speed) < 0.5)
            steer_value = 0;

          double set_speed = set_speed_in;
          double throttle = 1;
          
          set_speed = set_speed * (1 - 2*(abs(steer_value)));
          if(set_speed < 10)
            set_speed = 10;
          
          double speed_error = -1*(set_speed - speed);
          throttle = pid_throttle.UpdateError(speed_error);
          
          if(throttle > 1)
            throttle = 1;
          if(throttle < -1)
            throttle = -1;
            
          // DEBUG
          std::cout << "CTE:" << cte << " Steering Value:" << steer_value << " Current Value:" << angle << " Throttle:" << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          
          if(pid_steering.request_reset)
          {
            pid_steering.ReInit();
            std::string msg = "42[\"reset\",{}]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else
          {
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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
