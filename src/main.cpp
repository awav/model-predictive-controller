#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "aux.h"
#include "json.hpp"
#include "mpc.h"

#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

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
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd &coeffs, double x) {
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

template <typename Value = double>
Value json_data_get(const json &j, const std::string &key) {
  return j[1][key];
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  mpc::MPC controller;

  h.onMessage([&controller](uWS::WebSocket<uWS::SERVER> ws, char *data,
                            size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          std::vector<double> x_pts =
              json_data_get<std::vector<double>>(j, "ptsx");
          std::vector<double> y_pts =
              json_data_get<std::vector<double>>(j, "ptsy");
          assert(x_pts.size() == y_pts.size());
          const double px = json_data_get(j, "x");
          const double py = json_data_get(j, "y");
          const double psi = json_data_get(j, "psi");
          const double vel = json_data_get(j, "speed");
          double throttle = json_data_get(j, "throttle");
          double steer = json_data_get(j, "steering_angle");

          const double sin_psi = std::sin(-psi);
          const double cos_psi = std::cos(-psi);

          const int wlen = x_pts.size();
          Eigen::VectorXd wx_pts(wlen);
          Eigen::VectorXd wy_pts(wlen);
          for (int i = 0; i < wlen; i++) {
            const double dx = x_pts[i] - px;
            const double dy = y_pts[i] - py;
            wx_pts[i] = dx * cos_psi - dy * sin_psi;
            wy_pts[i] = dy * cos_psi + dx * sin_psi;
          }

          Eigen::VectorXd coeffs = polyfit(wx_pts, wy_pts, 3);

          std::vector<double> next_x(mpc::num_pred);
          std::vector<double> next_y(mpc::num_pred);
          for (int i = 0; i < (int)mpc::num_pred; ++i) {
            const double dx = 5 * i;
            const double dy = coeffs[3] * std::pow(dx, 3) +
                              coeffs[2] * std::pow(dx, 2) + coeffs[1] * dx +
                              coeffs[0];
            next_x[i] = dx;
            next_y[i] = dy;
          }

          const double cte = coeffs[0];
          const double epsi = -std::atan(coeffs[1]);

          const double cur_x = vel * mpc::dt;
          const double cur_y = 0;
          const double cur_psi = -vel * steer / mpc::lf * mpc::dt ;
          const double cur_vel = vel + throttle * mpc::dt;
          const double cur_cte = cte + vel * std::sin(epsi) * mpc::dt;
          const double cur_epsi = epsi - vel * steer / mpc::lf * mpc::dt;

          controller.Solve(cur_x, cur_y, cur_psi, cur_vel, cur_cte, cur_epsi,
                           coeffs);

          steer = controller.Steer();
          throttle = controller.Throttle();
          // const double steer_value = - controller.Steer() / mpc::rad25;

          json msgJson;
          msgJson["steering_angle"] = steer;
          msgJson["throttle"] = throttle;

          std::vector<double> x_future(controller.XCoord());
          std::vector<double> y_future(controller.YCoord());
          msgJson["mpc_x"] = x_future;
          msgJson["mpc_y"] = y_future;

          //msgJson["mpc_x"] = controller.XCoord();
          //msgJson["mpc_y"] = controller.YCoord();

          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
