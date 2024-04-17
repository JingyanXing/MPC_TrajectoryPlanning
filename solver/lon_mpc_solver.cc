#include "lon_mpc_solver.h"


using namespace std;
using namespace casadi;

DM LonMpcSolver(double curr_s, double curr_v, double curr_a, double ref_s,
                double ref_v, double ref_a, double max_speed, double max_acc,
                double max_decel, double max_jerk) {
  // 预测步长2s(20步)
  const int N = 20;
  const double T = 0.1;
  vector<double> parameters = {curr_s, curr_v, curr_a, ref_s, ref_v, ref_a};
  // 状态向量构建，分别为纵向位置，纵向速度，加速度
  SX s = SX::sym("s");
  SX v = SX::sym("v");
  SX a = SX::sym("a");
  SX x = vertcat(s, v, a);
  // 控制向量
  SX _j = SX::sym("_j");
  // 运动学模型
  SX ocp_fun = vertcat(v, a, _j);
  Function f =
      Function("ocp_fun", {x, _j}, {ocp_fun}, {"state", "input"}, {"ocp_fun"});
  SX U = SX::sym("U", 1, N);
  SX X = SX::sym(
      "X", 3, N + 1);  // N+1步的系统状态，通常长度#include <algorithm>比控制多1
  // 初始状态，终端状态
  SX P = SX::sym("P", 6);
  X(Slice(), 0) = P(Slice(0, 3, 1));
  for (int i = 0; i < N; i++) {
    vector<SX> input = {X(Slice(), i), U(Slice(), i)};
    X(Slice(), i + 1) = X(Slice(), i) + f(input).at(0) * T;
  }
  // 预测函数
  Function predict_f = Function("predict_f", {reshape(U, -1, 1), P}, {X},
                                {"input_U", "parameter"}, {"states"});
  // reshape第二个参数设为 -1，表示保持原始的行数不变，而将第三个参数设为
  // 1，表示将矩阵重塑为只有一列的列向量

  // 惩罚矩阵
  SX Q = SX::zeros(3, 3);
  SX R = SX::zeros(1, 1);
  Q(0, 0) = 2;
  Q(1, 1) = 10;
  Q(2, 2) = 4;
  R(0, 0) = 0.5;

  // 代价函数
  SX cost_f = SX::sym("cost_f");
  cost_f = 0;
  for (int i = 0; i < N; i++) {
    SX states_err =
        X(Slice(), i) -
        P(Slice(3, 6, 1));  // 从索引 3 开始（包含索引 3），到索引 6
                            // 结束（不包含索引 6），以步长 1 选择元素。
    cost_f = cost_f + SX::mtimes({states_err.T(), Q, states_err}) +
             SX::mtimes({U(i).T(), R, U(i)});  // 这里用控制量输入大小衡量
  }
  // 状态变量约束,输入变量约束
  vector<double> lbg;
  vector<double> ubg;
  vector<double> lbx(N, -max_jerk * T), ubx(N, max_jerk * T);
  SX g;

  double lower_s = curr_s;
  double lower_v = curr_v;
  double lower_a = curr_a;
  double upper_s = curr_s;
  double upper_v = curr_v;
  double upper_a = curr_a;

  for (int i = 0; i < N; i++) {
    g = vertcat(g, X(Slice(), i));
    lbg.push_back(lower_s);
    lbg.push_back(0);
    lbg.push_back(-max_decel);
    ubg.push_back(curr_s + curr_v * T * N + 0.5 * max_acc * pow(N * T, 2));
    ubg.push_back(max_speed);
    ubg.push_back(max_acc);
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
