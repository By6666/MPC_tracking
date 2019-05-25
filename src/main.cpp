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
extern double dt;
extern double Lf;//前轮半径

// // For converting back and forth between radians and degrees.
// constexpr double pi() { return M_PI; }//pai
// double deg2rad(double x) { return x * pi() / 180; }//角度转弧度
// double rad2deg(double x) { return x * 180 / pi(); }//弧度转角读

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
//计算多项式  y = a0 + a1*x^1 + a2*x^2 + a3*x^3
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
  assert(xvals.size() == yvals.size());//如果为假则终止
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
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];//传过来6个导航点，为了建立形式多项式曲线
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];//获得x
          double py = j[1]["y"];//获得y
          double psi = j[1]["psi"];//获得车身角度psi
          double v = j[1]["speed"];//获得车速
          double delta = j[1]["steering_angle"];//获得车轮转角
          double a = j[1]["throttle"];//获得加速度
          
          // Need Eigen vectors for polyfit
          //ptsx_car为以车辆为圆心的导航点坐标的存储变量
          Eigen::VectorXd ptsx_car(ptsx.size());
          Eigen::VectorXd ptsy_car(ptsy.size()); 
          
          // Transform the points to the vehicle's orientation
          // 转换到车辆坐标系
          for (int i = 0; i < ptsx.size(); i++) {
            // std::cout<<ptsx[i]<<" ";
            double x = ptsx[i] - px; //平移
            double y = ptsy[i] - py;
            ptsx_car[i] = x * cos(-psi) - y * sin(-psi);//旋转
            ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
          }
          // std::cout<<std::endl;
          // for (int i = 0; i < ptsx.size(); i++) {
          //   std::cout<<ptsx_car[i]<<" ";
          // }
          // std::cout<<std::endl;

          /*
          * Calculate steering angle and throttle using MPC.
          * Both are in between [-1, 1].
          * Simulator has 100ms latency, so will predict state at that point in time.
          * This will help the car react to where it is actually at by the point of actuation.
          */
          
          // Fits a 3rd-order polynomial to the above x and y coordinates
          //将6个导航点绘制成3阶多项式曲线
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);//计算3阶多项式的系数
          
          // Calculates the cross track error
          // Because points were transformed to vehicle coordinates, x & y equal 0 below.
          // 'y' would otherwise be subtracted from the polyeval value
          //计算最初的轨迹偏差，因为坐标转换为车辆坐标，所以初始的轨迹偏差为0
          double cte = /*polyeval(coeffs, 0)*/0;
          
          // Calculate the orientation error
          // Derivative of the polyfit goes in atan() below
          // Because x = 0 in the vehicle coordinates, the higher orders are zero
          // Leaves only coeffs[1]
          //初始的车身角度偏差
          double epsi = -atan(coeffs[1]);
                    
          // Predict state after latency
          // x, y and psi are all zero after transformation above
          //预测下一时刻的状态
          double pred_px = 0.0 + v * dt; // Since psi is zero, cos(0) = 1, can leave out
          double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
          double pred_psi = 0.0 + v * -delta / Lf * dt;//车身转角与车轮转角是负相关
          double pred_v = v + a * dt;
          double pred_cte = cte + v * sin(epsi) * dt;
          double pred_epsi = epsi + v * -delta / Lf * dt;
          
          // Feed in the predicted state values
          Eigen::VectorXd pre_state(6);
          pre_state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;
          
          // Solve for new actuations (and to show predicted x and y in the future)
          auto vars = mpc.Solve(pre_state, coeffs);
          
          // Calculate steering and throttle
          // Steering must be divided by deg2rad(25) to normalize within [-1, 1].
          // Multiplying by Lf takes into account vehicle's turning ability
          double steer_value = vars[0] / (deg2rad(25) * Lf);
          double throttle_value = vars[1];
          
          // Send values to the simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          std::cout<<"steering_angle : "<<steer_value<<"   throttle : "<<throttle_value<<std::endl; 

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals = {pre_state[0]};
          vector<double> mpc_y_vals = {pre_state[1]};

          // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          
          for (int i = 2; i < vars.size(); i+=2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          double poly_inc = 2;
          int num_points = 25;
          
          for (int i = 1; i < num_points; i++) {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

           auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car doesn't actuate the commands instantly.
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