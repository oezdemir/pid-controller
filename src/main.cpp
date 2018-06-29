#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <cstdlib>

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

// Restart the sim sending a command over the websocket
// (Found the command here: https://github.com/bguisard/CarND-PID-Control-Project/blob/master/src/PID.cpp)
void RestartSim(uWS::WebSocket<uWS::SERVER> & ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

  // // stop the vehicle
  // json msgJson;
  // msgJson["steering_angle"] = 0.0;
  // msgJson["brake"] = 0;
  // msgJson["throttle"] = -1.0;
  // auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  // std::cout << msg << std::endl;
  // for (auto i=0; i<5; i++){
  //   ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  // }
}

int main(int argc, char *argv[]) {
  uWS::Hub h;
  PID pid;
  unsigned num_telemetry = 0; // iterates with each telemetry response from the sim
  unsigned num_samples = 200;
  bool is_optimizing = false;
  double last_throttle = 0.3;
  double max_speed = 0.0;
  // double Kp = 0.05,
  //        Ki = 0.003,
  //        Kd = 3.9;
  // tuned on 0.4 throttle
  // double Kp = 0.7815647561804492,
  //        Ki = 0.0040939106276118775,
  //        Kd = 31.747072260867547;
  // further tuned on 0.8
  double Kp = 0.6963741977567802,
         Ki = 0.004904095540816269,
         Kd = 38.029817861293246;

  if (argc == 4 || argc == 5) {
    is_optimizing = true;
    Kp = std::atof(argv[1]);
    Ki = std::atof(argv[2]);
    Kd = std::atof(argv[3]);
    if (argc == 5) {
      num_samples = std::atoi(argv[4]);
    }
    std::cout << "Optimizing.. Using commandline supplied values." << " Kp=" << Kp << " Ki= " << Ki << " Kd=" << Kd << std::endl;
  }
  pid.Init(Kp, Ki, Kd);

  h.onMessage([&pid, &num_telemetry, &is_optimizing, &num_samples, &last_throttle, &max_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          num_telemetry++;
          if (is_optimizing) {
            if( num_telemetry == 1) {
              // start with a clean sim state
              //RestartSim(ws);
            } else if (num_telemetry == num_samples) {
              num_telemetry = 0;
              std::cerr << "Accumulated error:" << pid.GetAccumulatedError() << std::endl;
              exit(0);
            }
          }

          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());

          if (speed > max_speed) {
            max_speed = speed;
            std::cout << "Max Speed: " << speed << " mph" << std::endl;
          }
          // double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // Supply the cte to the PID controller
          pid.UpdateError(cte);

          // PID controller output for steering
          double steer_value = pid.TotalError();
          double throttle = 1.0;
          const int kSpeedDownHysteresis = 4;  // frames to wait before speed up again
          int speeding_down = 0;
          double slowdown_factor = 1.6;

          if (speeding_down) {
            throttle = last_throttle / slowdown_factor;
            speeding_down--;
          }

          //if (num_telemetry % 1000 == 0) pid.i_error = 0;

          // reduce speed, if PID tries to oversteer
          if (speed > 20.0 && (steer_value > 1.0 || steer_value < -1.0 )) {
            throttle = (speed / 100.0) * (1.0 / (std::abs(steer_value) * slowdown_factor ));
            speeding_down = kSpeedDownHysteresis;
          }
          last_throttle = throttle;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // DEBUG
          // DEBUG std::cout << "[" << num_telemetry << "]" << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // DEBUG std::cout << msg << std::endl;
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
