#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }//pai
double deg2rad(double x) { return x * pi() / 180; }//角度转弧度
double rad2deg(double x) { return x * 180 / pi(); }//弧度转角读

// Set the timestep length and duration
// Currently tuned to predict 1 second worth
size_t N = 10;//预测步长
double dt = 0.1;//控制周期为0.1s

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
double Lf = 2.67;//前轮半径

// Set desired speed for the cost function (i.e. max speed)
const double ref_v = 60;//最大速递

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
//以下定义相应变量在存储vector中的开始位置
size_t x_start = 0;
size_t y_start = x_start + N;

size_t psi_start = y_start + N;//车身调度psi
size_t v_start = psi_start + N;//速度v

size_t cte_start = v_start + N;//轨迹跟踪误差cte
size_t epsi_start = cte_start + N;//车身角度误差epsi

size_t delta_start = epsi_start + N;//前轮转向角delta
size_t a_start = delta_start + N - 1;//加速度a

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;//系数
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }//构造函数

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implementing MPC below--在下面执行MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;//存放cost
    
    // Reference State Cost
    // Below defines the cost related the reference state and
    // any anything you think may be beneficial.
    
    // Weights for how "important" each cost is - can be tuned
    const int cte_cost_weight = /*2000*/1000;//轨迹跟踪误差权重
    const int epsi_cost_weight = /*2000*/500;//车身角度误差权重
    const int v_cost_weight = /*1*/2;//速度损失函数权重
    const int delta_cost_weight = /*10*/0;//转向损失函数权重
    const int a_cost_weight = /*10*/0;//加速度损失函数权重
    const int delta_change_cost_weight = /*100*/1;//转向变化率权重
    const int a_change_cost_weight = /*10*/20;//加速度变化率权重
    
    //计算cost
    // Cost for CTE, psi error and velocity
    for (int t = 0; t < N; t++) {
      fg[0] += cte_cost_weight * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += v_cost_weight * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // Costs for steering (delta) and acceleration (a)
    for (int t = 0; t < N-1; t++) {
      fg[0] += delta_cost_weight * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += a_cost_weight * CppAD::pow(vars[a_start + t], 2);
    }
    
    // Costs related to the change in steering and acceleration (makes the ride smoother)
    for (int t = 0; t < N-2; t++) {
      fg[0] += delta_change_cost_weight * pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += a_change_cost_weight * pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    
    // Setup Model Constraints
    
    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // State at time t + 1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      
      // State at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      
      // Actuator constraints at time t only
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      
      //轨迹模型，得出轨迹上的y与车身转角psi
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));
      
      // Setting up the rest of the model constraints
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
    }
  } 
};

//
// MPC class definition implementation.
//
//MPC::MPC() {}//**
//MPC::~MPC() {}//**

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  // State vector holds all current values neede for vars below
  //赋值操作（这是下一个时刻的预测值）
  double x = state[0];//坐标
  double y = state[1];
  double psi = state[2];//车身转角
  double v = state[3];//速度
  double cte = state[4];//轨迹误差
  double epsi = state[5];//车身转角误差

  // Setting the number of model variables (includes both states and inputs).
  // N * state vector size + (N - 1) * 2 actuators (For steering & acceleration)
  //变量个数定义，8个变量为 X、Y、Psi、V、cte、epsi    和   delta、a
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Setting the number of constraints
  //约束个数
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  //变量状态存储vector
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Sets lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  //规定变量的上下限
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  //转向角度限制
  for (int i = delta_start; i < a_start; i++) {
    // vars_lowerbound[i] = -0.436332;deg2rad(-25);
    // vars_upperbound[i] = 0.436332;
    vars_lowerbound[i] = deg2rad(-25);
    vars_upperbound[i] = deg2rad(25);
  }
  
  // Acceleration/decceleration upper and lower limits.
  //加速度a限制
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  // Start lower and upper limits at current values
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  // std::cout << "Cost " << cost << std::endl;
  std::cout << "Cost " << cost << "  ";

  // Return the first actuator values, along with predicted x and y values to plot in the simulator.
  vector<double> solved;
  solved.push_back(solution.x[delta_start]);
  solved.push_back(solution.x[a_start]);
  for (int i = 0; i < N; ++i) {
    solved.push_back(solution.x[x_start + i]);
    solved.push_back(solution.x[y_start + i]);
  }
  
  return solved;

}
