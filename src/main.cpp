#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "mpc.h"
#include "aux.h"

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
Eigen::VectorXd polyfit(const Eigen::VectorXd xvals,
                        const Eigen::VectorXd &yvals, int order) {
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

template <typename Out = double, typename Json = json::basic_json>
Out json_data_get(const Json &j, const std::string &key) {
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
          const auto &wx = json_data_get<std::vector<double>>(j, "ptsx");
          const auto &wy = json_data_get<std::vector<double>>(j, "ptsy");
          assert(wx.size() == wy.size());
          const double px = json_data_get(j, "x");
          const double py = json_data_get(j, "y");
          const double psi = json_data_get(j, "psi");
          const double steering_angle = json_data_get(j, "steering_angle");
          const double throttle = json_data_get(j, "throttle");
          // Hah, convert mph to m/sec. Stop using mph :)
          const double vel = json_data_get(j, "speed") * 0.44704;

          const double sin_psi = std::sin(-psi);
          const double cos_psi = std::cos(-psi);

          const int wlen = wx.size();
          Eigen::VectorXd x_veh(wlen);
          Eigen::VectorXd y_veh(wlen);
          std::vector<double> next_x(wlen);
          std::vector<double> next_y(wlen);
          for (int i = 0; i < wlen; i++) {
            const double dx = wx[i] - px;
            const double dy = wy[i] - py;
            x_veh[i] = dx * cos_psi - dy * sin_psi;
            y_veh[i] = dy * cos_psi + dx * sin_psi;
            next_x[i] = x_veh[i];
            next_y[i] = y_veh[i];
          }

          Eigen::VectorXd coeffs = polyfit(x_veh, y_veh, 3);

          const double px_act = vel * mpc::dt;
          const double py_act = 0;
          const double cte = coeffs[0];
          const double epsi = -std::atan(coeffs[1]);

          const double psi_act = -vel * steering_angle * mpc::dt / mpc::lf;
          const double v_act = vel + throttle * mpc::dt;
          const double cte_act = cte + vel * std::sin(epsi) * mpc::dt;
          const double epsi_act = epsi + psi_act;

          Eigen::VectorXd state(static_cast<int>(mpc::kStateSize));

          state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;

          controller.Solve(state, coeffs);

          const double throttle_value = controller.State()[mpc::kStateAcc];
          const double steer_value =
              -controller.State()[mpc::kStateDelta] / mpc::rad25;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;

          msgJson["mpc_x"] = controller.XCoord();
          msgJson["mpc_y"] = controller.YCoord();

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
