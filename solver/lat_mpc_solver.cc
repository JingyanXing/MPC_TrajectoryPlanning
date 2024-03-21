#include "lat_mpc_solver.h"

using namespace std;
using namespace casadi;

casadi::DM LatMpcSolver(double curr_s, double curr_l, double curr_heading_angle, double curr_heading_angle_rate, 
                        double ref_s, double ref_l, double ref_heading_angle, double ref_heading_angle_rate, 
                        vector<double> ref_speed, vector<double> lower_l, vector<double> upper_l, 
                        double MAX_WHEELANGLE, double MAX_HEADINGANGLE, double MAX_SPEED, double wheelbase){
    //预测步长2s(20步)
    const int N = 20;
    const double T = 0.1;
    //初始状态，终端状态, 由纵向mpc得到的速度
    vector<double> parameters = {curr_s, curr_l, curr_heading_angle, curr_heading_angle_rate, 
                                ref_s, ref_l, ref_heading_angle, ref_heading_angle_rate};
    for(double& ref : ref_speed){
        parameters.emplace_back(ref);
    }
    SX P = SX::sym("P", parameters.size());
    //状态向量构建，分别为纵向位置，横向位置，航向角，航向角变化率
    SX s = SX::sym("s");
    SX l = SX::sym("l"); 
    SX heading_angle = SX::sym("heading_angle");
    SX heading_angle_rate = SX::sym("heading_angle_rate");
    SX x = vertcat(s, l,heading_angle, heading_angle_rate);
    //控制向量,此处为航向角变化率导数
    SX heading_angle_rate_dot = SX::sym("heading_angle_rate_dot");
    SX U = SX::sym("U", 1, N);
    SX X = SX::sym("X", 4, N + 1); //N+1步的系统状态，长度比控制多1
    
    X(Slice(),0) = P(Slice(0,4,1));
    for(int i = 0; i < N; i++) {
        X(Slice(), i + 1) = X(Slice(), i) + T * vertcat(P(8 + i) * cos(X(Slice(), i)(2)), 
                                                        P(8 + i) * sin(X(Slice(), i)(2)),
                                                        X(Slice(), i)(3),
                                                        U(i));
    }
    //预测函数
    Function predict_f = Function("predict_f", {reshape(U, -1, 1), P}, {X}, {"input_U", "parameter"}, {"states"});  
    //reshape第二个参数设为 -1，表示保持原始的行数不变，而将第三个参数设为 1，表示将矩阵重塑为只有一列的列向量

    //惩罚矩阵
    SX Q = SX::zeros(4,4);
    SX R = SX::zeros(1,1);
    Q(0,0) = 0.1;
    Q(1,1) = 5;
    Q(2,2) = 5;
    Q(3,3) = 1;
    R(0,0) = 0.5;

    // 代价函数
    SX cost_f = SX::sym("cost_f");
    cost_f = 0;
    for(int i = 0; i < N; i++) {
        SX states_err = X(Slice(),i) - P(Slice(4,8,1));//从索引 3 开始（包含索引 3），到索引 6 结束（不包含索引 6），以步长 1 选择元素。
        cost_f = cost_f + SX::mtimes({states_err.T(),Q,states_err}) + SX::mtimes({U(i).T(), R, U(i)}); //这里用控制量输入大小衡量
    }
    //状态变量约束,输入变量约束
    vector<double> lbg; 
    vector<double> ubg;
    vector<double> lbx, ubx;
    SX g;

    for(int i = 0; i < N ; i++){
        lbx.push_back(T * ref_speed[i] * tan(-MAX_WHEELANGLE) / wheelbase);
        ubx.push_back(T * ref_speed[i] * tan(MAX_WHEELANGLE) / wheelbase);
        g = vertcat(g,X(Slice(),i));
        lbg.push_back(curr_s);
        lbg.push_back(lower_l[i]);
        lbg.push_back(-MAX_HEADINGANGLE);
        lbg.push_back(-MAX_SPEED * tan(MAX_WHEELANGLE) / wheelbase);
        ubg.push_back(curr_s + (i + 1) * T * MAX_SPEED);
        ubg.push_back(upper_l[i]);
        ubg.push_back(MAX_HEADINGANGLE);
        ubg.push_back(MAX_SPEED * tan(MAX_WHEELANGLE) / wheelbase);
    }
    //构造
    SXDict qp = { { "x", SX::reshape(U.T(),-1,1) }, { "f", cost_f }, { "p", P }, {"g", g}};
    //求解器设置
    Dict qp_opts;
    qp_opts["expand"] = true;  //设置了求解器选项 expand 的值为 true。这个选项表示是否将所有符号变量和函数进行展开，以加速求解。
    qp_opts["ipopt.max_iter"] = 10;  //最大迭代次数为 
    qp_opts["ipopt.print_level"] = 0;  //设置了 ipopt 求解器的打印级别为 0。这个选项控制了求解器在求解过程中输出的详细程度，0 表示不输出详细信息。
    qp_opts["print_time"] = 0;  //设置了是否打印求解器运行时间的选项。设置为 0 表示不打印求解器运行时间。
    qp_opts["ipopt.acceptable_tol"] =  1e-6; //设置了 ipopt 求解器的可接受目标函数变化容差为。这个选项控制了求解器在停止迭代之前所允许的目标函数变化的最小值。
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