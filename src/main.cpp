#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
//#include "gnuplot_i.hpp"

//#include "MPC.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
//用于在弧度和度之间来回转换。
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
//检查SocketIO事件是否包含JSON数据。
//如果有数据，将返回字符串格式的JSON对象，
//否则将返回空字符串“”。
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
//计算一个多项式。
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
//拟合多项式。
//改编自
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

int main() {
  uWS::Hub h;
  //文件初始化
  ofstream  mpcout,nextout,psiout,xyout,steerout,throttleout,upsi;
  mpcout.open("output.txt",ios::out|ios::trunc);
  nextout.open("nextop.txt",ios::out|ios::trunc);
  psiout.open("psiop.txt",ios::out|ios::trunc);
  xyout.open("xyop.txt",ios::out|ios::trunc);
  steerout.open("steerop.txt",ios::out|ios::trunc);
  throttleout.open("throttleop.txt",ios::out|ios::trunc);
  upsi.open("upsiop.txt",ios::out|ios::trunc);

  mpcout <<"0 0" <<std::endl;
  nextout<<"0 0" <<std::endl;
  psiout<<"0 0" <<std::endl;
  xyout<<"0 0" <<std::endl;
  steerout<<"0 0" <<std::endl;
  throttleout<<"0 0" <<std::endl;
  upsi<<"0 0" <<std::endl;

  mpcout.close();
  nextout.close();
  psiout.close();
  xyout.close();
  steerout.close();
  throttleout.close();
  upsi.close();
  // MPC is initialized here!
  //这里初始化了MPC！
  MPC mpc;
//打开表格文件
//ofstream  mpcout;
//mpcout.open("output.csv",ios::out|ios::trunc);
//
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //消息开头的“42”表示有websocket消息事件。
//4表示websocket消息
//2表示websocket事件
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          //j[1]是数据JSON对象
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];
          
          // Need Eigen vectors for polyfit
          //多边形拟合需要特征向量
          Eigen::VectorXd ptsx_car(ptsx.size());
          Eigen::VectorXd ptsy_car(ptsy.size());
          
          // Transform the points to the vehicle's orientation
          //将这些点转换为车辆的方向
 
          for (int i = 0; i < ptsx.size(); i++) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
            ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
          }
          
          /*
          * Calculate steering angle and throttle using MPC.
          * Both are in between [-1, 1].
          * Simulator has 100ms latency, so will predict state at that point in time.
          * This will help the car react to where it is actually at by the point of actuation.
          * *使用MPC计算转向角和油门。
          *两者都在[-1,1]之间。
         *模拟器有100毫秒的延迟，所以会及时预测该时间点的状态。
          *这将有助于汽车在启动时对实际位置做出反应。
          */
          
          // Fits a 3rd-order polynomial to the above x and y coordinates
          //将三阶多项式拟合到上述x和y坐标
 
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
          
          // Calculates the cross track error
          // Because points were transformed to vehicle coordinates, x & y equal 0 below.
          // 'y' would otherwise be subtracted from the polyeval value
          //计算交叉轨迹误差
          //因为点被转换为车辆坐标，所以x和y等于下面的0。
          //否则，“y”将从polyeval值中减去
          double cte = polyeval(coeffs, 0);
          
          // Calculate the orientation error
          // Derivative of the polyfit goes in atan() below
          // Because x = 0 in the vehicle coordinates, the higher orders are zero
          // Leaves only coeffs[1]
          //计算定向误差
          //polyfit的导数在下面的atan（）中
          //因为在车辆坐标中x=0，所以高阶为零
          //只留下系数[1]

          double epsi = -atan(coeffs[1]);
          
          // Center of gravity needed related to psi and epsi
          //需要与psi和epsi相关的重心
          const double Lf = 2.67;
          
          // Latency for predicting time at actuation
          //用于预测启动时间的延迟
          const double dt = 0.1;
          
          // Predict state after latency
          // x, y and psi are all zero after transformation above
          //预测延迟后的状态
          //变换后x和y都大于零
          double pred_px = 0.0 + v * dt; 
          // Since psi is zero, cos(0) = 1, can leave out//由于psi为零，cos（0）=1可以省略
 
          const double pred_py = 0.0; 
          // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)//由于sin（0）=0，y保持为0（y+v*0*dt）
          double pred_psi = 0.0 + v * -delta / Lf * dt;
          double pred_v = v + a * dt;
          double pred_cte = cte + v * sin(epsi) * dt;
          double pred_epsi = epsi + v * -delta / Lf * dt;
          
          // Feed in the predicted state values
          //输入预测的状态值
          Eigen::VectorXd state(6);
          state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;
          
          // Solve for new actuations (and to show predicted x and y in the future)
          //求解新的驱动（并在未来显示预测的x和y）
          auto vars = mpc.Solve(state, coeffs);
          
          // Calculate steering and throttle
          // Steering must be divided by deg2rad(25) to normalize within [-1, 1].
          // Multiplying by Lf takes into account vehicle's turning ability
          //计算转向和油门
          //转向必须除以deg2rad（25）才能在[-1,1]范围内正常化。
          //乘以Lf会考虑车辆的转向能力
          double steer_value = vars[0] / (deg2rad(25) * Lf);
          double throttle_value = vars[1];
          
          // Send values to the simulator
          //将值发送到模拟器
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          //显示MPC预测的轨迹
          vector<double> mpc_x_vals = {state[0]};
          vector<double> mpc_y_vals = {state[1]};

          // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          //在此处添加（x，y）点，这些点参照车辆坐标系
          //模拟器中的点通过绿线连接
          
          for (int i = 2; i < vars.size(); i+=2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          //显示航路点/参考线
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          //在此处添加（x，y）点，这些点参照车辆坐标系
          //模拟器中的点由一条黄线连接
          double poly_inc = 2.5;
          int num_points = 25;
          
          for (int i = 1; i < num_points; i++) {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //文件输出
          ofstream  mpcout,nextout,psiout,xyout,steerout,throttleout,upsi;
          mpcout.open("output.txt",ios::app|ios::out);//|ios::out|ios::trunc);
          nextout.open("nextop.txt",ios::app|ios::out);
          psiout.open("psiop.txt",ios::app|ios::out);
          xyout.open("xyop.txt",ios::app|ios::out);
          steerout.open("steerop.txt",ios::app|ios::out);
          throttleout.open("throttleop.txt",ios::app|ios::out);
          upsi.open("upsiop.txt",ios::app|ios::out);

          //mpcout <<  msgJson.dump()<< ","<<j[0]<<"##"<<j[1] << std::endl;
          
          for(int i=0;i<11;++i){
          mpcout << msgJson["mpc_x"][i] <<" " 
          <<msgJson["mpc_y"][i] 
          <<std::endl;
          }
          for(int i=0;i<24;++i){
          nextout << msgJson["next_x"][i] <<" " 
          <<msgJson["next_y"][i] 
          <<std::endl;
          }
          for(int i=0;i<6;++i){
          psiout << j[1]["ptsx"][i] <<" " 
          <<j[1]["ptsy"][i] 
          <<std::endl;
          }
          xyout <<j[1]["x"]<<" "<<j[1]["y"]<<std::endl;
          steerout<<msgJson["steering_angle"]<<" "<<j[1]["steering_angle"]<<std::endl;
          throttleout<<msgJson["throttle"]<<" "<<j[1]["throttle"]<<std::endl;
          upsi<<j[1]["psi"]<<" "<<j[1]["psi"]<<std::endl;

          mpcout.close();
          nextout.close();
          psiout.close();
          xyout.close();
          steerout.close();
          throttleout.close();
          upsi.close();
          //mpcout <<  j[1] << std::endl;
         /* mpcout << "mpc_x"//<<","
          <<"mpc_y"//<<","
          
          << "next_x"<<","
          <<"next_y"<<","
          <<"steering_angle"<<","
          <<"throttle"<<","
          <<std::endl;*/
          /*
          for(int i=0;i<11;++i){
          mpcout << msgJson["mpc_x"][i] <<" "//<<","
          <<msgJson["mpc_y"][i]//<<","
          <<std::endl;
          }*/
          /*
          for(int j=0;j<25;++j){
          mpcout <<msgJson["next_x"][j]<<","
          <<msgJson["next_y"][j]<<","
          <<std::endl;
          }
          mpcout<<msgJson["steering_angle"]<<","
          <<msgJson["steering_angle"]<<std::endl;*/
          
          std::cout << msg << std::endl;
          //mpcout.close();
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car doesn't actuate the commands instantly.
          //潜伏期
          //其目的是模拟真实的驾驶条件
          //汽车不会立即启动指令。
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        //人工驾驶
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  //我们不需要这个，因为我们不使用HTTP，但是如果它被删除了
  //节目
  //不编译：-(
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
