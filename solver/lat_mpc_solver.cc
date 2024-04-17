#include "lat_mpc_solver.h"
#define N 20
#define step_length 0.1
using namespace std;
using namespace casadi;

casadi::DM LatMpcSolver(double curr_s, double curr_l, double curr_heading_angle,
                        double curr_heading_angle_rate, double ref_s,
                        double ref_l, double ref_heading_angle,
                        double ref_heading_angle_rate, vector<double> ref_speed,
                        vector<double> lower_l, vector<double> upper_l,
                        double MAX_WHEELANGLE, double MAX_HEADINGANGLE,
                        double MAX_SPEED, double wheelbase) {
  // 初始状态，终端状态, 由纵向mpc得到的速度
  vector<double> parameters = {
      curr_s, curr_l, curr_heading_angle, curr_heading_angle_rate,
      ref_s,  ref_l,  ref_heading_angle,  ref_heading_angle_rate};
  for (double& ref : ref_speed) {
    parameters.emplace_back(ref);
  }
  SX P = SX::sym("P", parameters.size());
  // 状态向量构建，分别为纵向位置，横向位置，航向角，航向角变化率
  SX s = SX::sym("s");
  SX l = SX::sym("l");
  SX heading_angle = SX::sym("heading_angle");
  SX heading_angle_rate = SX::sym("heading_angle_rate");
  SX x = vertcat(s, l, heading_angle, heading_angle_rate);
  // 控制向量,此处为航向角变化率导数
  SX heading_angle_rate_dot = SX::sym("heading_angle_rate_dot");
  SX U = SX::sym("U", 1, N);
  SX X = SX::sym("X", 4, N + 1);  // N+1步的系统状态，长度比控制多1

  X(Slice(), 0) = P(Slice(0, 4, 1));
  for (int i = 0; i < N; i++) {
    X(Slice(), i + 1) =
        X(Slice(), i) + step_length * vertcat(P(8 + i) * cos(X(Slice(), i)(2)),
                                    P(8 + i) * sin(X(Slice(), i)(2)),
                                    X(Slice(), i)(3), U(i));
  }
  // 预测函数
  Function predict_f = Function("predict_f", {reshape(U, -1, 1), P}, {X},
                                {"input_U", "parameter"}, {"states"});
  // reshape第二个参数设为 -1，表示保持原始的行数不变，而将第三个参数设为
  // 1，表示将矩阵重塑为只有一列的列向量

  // 惩罚矩阵
  SX Q = SX::zeros(4, 4);
  SX R = SX::zeros(1, 1);
  Q(0, 0) = 0.1;
  Q(1, 1) = 5;
  Q(2, 2) = 5;
  Q(3, 3) = 1;
  R(0, 0) = 0.5;

  // 代价函数
  SX cost_f = SX::sym("cost_f");
  cost_f = 0;
  for (int i = 0; i < N; i++) {
    SX states_err =
        X(Slice(), i) -
        P(Slice(4, 8, 1));  // 从索引 3 开始（包含索引 3），到索引 6
                            // 结束（不包含索引 6），以步长 1 选择元素。
    cost_f = cost_f + SX::mtimes({states_err.T(), Q, states_err}) +
             SX::mtimes({U(i).T(), R, U(i)});  // 这里用控制量输入大小衡量
  }
  // 状态变量约束,输入变量约束
  vector<double> lbg;
  vector<double> ubg;
  vector<double> lbx, ubx;
  SX g;

  for (int i = 0; i < N; i++) {
    lbx.push_back(step_length * ref_speed[i] * tan(-MAX_WHEELANGLE) / wheelbase);
    ubx.push_back(step_length * ref_speed[i] * tan(MAX_WHEELANGLE) / wheelbase);
    g = vertcat(g, X(Slice(), i));
    lbg.push_back(curr_s);
    lbg.push_back(lower_l[i]);
    lbg.push_back(-MAX_HEADINGANGLE);
    lbg.push_back(-MAX_SPEED * tan(MAX_WHEELANGLE) / wheelbase);
    ubg.push_back(curr_s + (i + 1) * step_length * MAX_SPEED);
    ubg.push_back(upper_l[i]);
    ubg.push_back(MAX_HEADINGANGLE);
    ubg.push_back(MAX_SPEED * tan(MAX_WHEELANGLE) / wheelbase);
  }
  // 构造
  SXDict qp = {
      {"x", SX::reshape(U.T(), -1, 1)}, {"f", cost_f}, {"p", P}, {"g", g}};
  // 求解器设置
  Dict qp_opts;
  qp_opts["expand"] =
      true;  // 设置了求解器选项 expand 的值为
             // true。这个选项表示是否将所有符号变量和函数进行展开，以加速求解。
  qp_opts["ipopt.max_iter"] = 10;  // 最大迭代次数为
  qp_opts["ipopt.print_level"] = 0;  // 设置了 ipopt 求解器的打印级别为
                                     // 0。这个选项控制了求解器在求解过程中输出的详细程度，0
                                     // 表示不输出详细信息。
  qp_opts["print_time"] = 0;  // 设置了是否打印求解器运行时间的选项。设置为 0
                              // 表示不打印求解器运行时间。
  qp_opts["ipopt.acceptable_tol"] =
      1e-6;  // 设置了 ipopt
             // 求解器的可接受目标函数变化容差为。这个选项控制了求解器在停止迭代之前所允许的目标函数变化的最小值。
  qp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;

  Function solver = nlpsol("Solver", "ipopt", qp, qp_opts);

  map<std::string, DM> arg;
  arg["lbg"] = lbg;
  arg["ubg"] = ubg;
  arg["lbx"] = lbx;
  arg["ubx"] = ubx;
  arg["p"] = parameters;

  DMDict res = solver(arg);
  return res.at("x");
}

casadi::DM LatMpcSolverWheelAngle(
    double curr_l, double curr_l_rate, double curr_heading_angle,
    double curr_heading_angle_rate, double ref_l, double ref_l_rate,
    double ref_heading_angle, double ref_heading_angle_rate, double wheelangle,
    std::vector<double> ref_speed, std::vector<double> lower_l,
    std::vector<double> upper_l, double MAX_WHEELANGLE, double MAX_HEADINGANGLE,
    double MAX_SPEED, double wheelbase, double vehicle_weigth,
    double front_wheel_corner_stiffness, double rear_wheel_corner_stiffness,
    double moment_of_inertia) {
  // 初始状态，终端状态, 由纵向mpc得到的速度
  vector<double> parameters = {
      curr_l, curr_l_rate, curr_heading_angle, curr_heading_angle_rate,
      ref_l,  ref_l_rate,  ref_heading_angle,  ref_heading_angle_rate};
  for (double& ref : ref_speed) {
    if(ref < 0.01) parameters.emplace_back(0.01);
    else parameters.emplace_back(ref);
  }
  SX P = SX::sym("P", parameters.size());

  // 状态向量构建，分别为纵向位置，横向位置，航向角，航向角变化率
  SX l = SX::sym("l");
  SX l_rate = SX::sym("l_rate");
  SX heading_angle = SX::sym("heading_angle");
  SX heading_angle_rate = SX::sym("heading_angle_rate");
  SX x = vertcat(l, l_rate, heading_angle, heading_angle_rate);

  // 控制向量,此处为轮胎转角
  SX wheel_angle = SX::sym("wheel_angle");
  SX U = SX::sym("U", 1, N);
  SX X = SX::sym("X", 4, N + 1);  // N+1步的系统状态，长度比控制多1
  X(Slice(), 0) = P(Slice(0, 4, 1));

  // 构造状态转移方程，假设重心位于车轴中点
  for (int i = 0; i < N; i++) {
    X(Slice(), i + 1) =
        X(Slice(), i) +
        step_length * vertcat(X(Slice(), i)(1),
                    -2 * (front_wheel_corner_stiffness + rear_wheel_corner_stiffness) / (vehicle_weigth * P(8 + i)) * X(Slice(), i)(1)
                    - (P(8 + i) + (front_wheel_corner_stiffness - rear_wheel_corner_stiffness) * wheelbase / (vehicle_weigth * P(8 + i))) * X(Slice(), i)(3)
                    + 2 * front_wheel_corner_stiffness * U(i) / vehicle_weigth,
                    X(Slice(), i)(3), 
                    -(front_wheel_corner_stiffness - rear_wheel_corner_stiffness) * wheelbase / (moment_of_inertia * P(8 + i)) * X(Slice(), i)(1)
                    - 0.5 * pow(wheelbase, 2) * (front_wheel_corner_stiffness + rear_wheel_corner_stiffness) / (moment_of_inertia * P(8 + i)) * X(Slice(), i)(3)
                    + wheelbase * front_wheel_corner_stiffness / moment_of_inertia * U(i));
  }
  
  // 预测函数
  Function predict_f = Function("predict_f", {reshape(U, -1, 1), P}, {X},
                                {"input_U", "parameter"}, {"states"});
  // reshape第二个参数设为 -1，表示保持原始的行数不变，而将第三个参数设为
  // 1，表示将矩阵重塑为只有一列的列向量

  // 惩罚矩阵
  SX Q = SX::zeros(4, 4);
  SX R = SX::zeros(1, 1);
  Q(0, 0) = 10;
  Q(1, 1) = 1;
  Q(2, 2) = 2;
  Q(3, 3) = 1;
  R(0, 0) = 0.5;

  // 代价函数
  SX cost_f = SX::sym("cost_f");
  cost_f = 0;
  for (int i = 0; i < N; i++) {
    SX states_err = X(Slice(), i) - P(Slice(4, 8, 1));  // 从索引 4 开始（包含索引 4），到索引 8 结束（不包含索引 8），以步长 1 选择元素。
    cost_f = cost_f + SX::mtimes({states_err.T(), Q, states_err}) +
             SX::mtimes({U(i).T(), R, U(i)});  // 这里用控制量输入大小衡量
  }
  // 状态变量约束,输入变量约束
  vector<double> lbg;
  vector<double> ubg;
  vector<double> lbx, ubx; //控制输入上下界
  SX g;

  for (int i = 0; i < N; i++) {
    // lbx.push_back(max(-MAX_WHEELANGLE, wheelangle - step_length * MAX_WHEELANGLE * (i + 1)));
    // ubx.push_back(min(MAX_WHEELANGLE, wheelangle + step_length * MAX_WHEELANGLE * (i + 1)));
    lbx.push_back(-MAX_HEADINGANGLE);
    ubx.push_back(MAX_HEADINGANGLE);
    g = vertcat(g, X(Slice(), i));
    lbg.push_back(lower_l[i]);
    lbg.push_back(-5);
    lbg.push_back(-MAX_HEADINGANGLE);
    lbg.push_back(-MAX_SPEED * tan(MAX_WHEELANGLE) / wheelbase);
    ubg.push_back(upper_l[i]);
    ubg.push_back(5);
    ubg.push_back(MAX_HEADINGANGLE);
    ubg.push_back(MAX_SPEED * tan(MAX_WHEELANGLE) / wheelbase);
  }
  // 构造
  SXDict qp = {
      {"x", SX::reshape(U.T(), -1, 1)}, {"f", cost_f}, {"p", P}, {"g", g}};
  // 求解器设置
  Dict qp_opts;
  qp_opts["expand"] =
      true;  // 设置了求解器选项 expand 的值为
             // true。这个选项表示是否将所有符号变量和函数进行展开，以加速求解。
  qp_opts["ipopt.max_iter"] = 100;  // 最大迭代次数为
  qp_opts["ipopt.print_level"] =
      0;  // 设置了 ipopt 求解器的打印级别为
          // 0。这个选项控制了求解器在求解过程中输出的详细程度，0
          // 表示不输出详细信息。
  qp_opts["print_time"] = 0;  // 设置了是否打印求解器运行时间的选项。设置为 0
                              // 表示不打印求解器运行时间。
  qp_opts["ipopt.acceptable_tol"] =
      1e-6;  // 设置了 ipopt
             // 求解器的可接受目标函数变化容差为。这个选项控制了求解器在停止迭代之前所允许的目标函数变化的最小值。
  qp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;

  Function solver = nlpsol("Solver", "ipopt", qp, qp_opts);

  map<std::string, DM> arg;
  arg["lbg"] = lbg;
  arg["ubg"] = ubg;
  arg["lbx"] = lbx;
  arg["ubx"] = ubx;
  arg["p"] = parameters;

  DMDict res = solver(arg);
  return res.at("x");
}