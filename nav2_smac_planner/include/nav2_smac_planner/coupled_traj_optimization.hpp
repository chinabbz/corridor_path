#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>

// ROS
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/path.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Submodules
#include "nav2_smac_planner/optimization.h"
#include "nav2_smac_planner/com_fun.h"
#include "nav2_smac_planner/optimization.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>

using CppAD::AD;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start;
size_t y_start;
size_t theta_start; // 角度
size_t v_start;
size_t omega_start; // 角速度

// std::vector<ros::Publisher> way_point_pub;

std::vector<std::array<int, 3> > relative_pair;
std::vector<int> agent_col_num;

class FG_eval {
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars) {
        // MPC 实现
        // fg a vector of constraints, x is a vector of constraints.
        // NOTE: You'll probably go back and forth between this function and the Solver function below.
        fg[0] = 0;
        // 这里是当前轨迹与参考轨迹之间的偏差，注意没有角度信息
        for (int i = 1; i < N; i++) {
            fg[0] += W_X * CppAD::pow(vars[x_start + i] - plan[i][0], 2);
            fg[0] += W_Y * CppAD::pow(vars[y_start + i] - plan[i][1], 2);
        }

        // Minimize the value gap between sequential actuations.
        // 最小化连续输入之间的差异
        for (int i = 0; i < N - 2; i++) {
            fg[0] += W_DV * CppAD::pow(vars[v_start + i + 1] - vars[v_start + i], 2);
            fg[0] += W_DOMEGA * CppAD::pow(vars[omega_start + i + 1] - vars[omega_start + i], 2);
        }

        // 初始位置限制
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + theta_start] = vars[theta_start];

        // 控制模型的限制
        for (int i = 0; i < N - 1; i++) {
            // t+1时刻的状态
            AD<double> x1 = vars[x_start + i + 1];
            AD<double> y1 = vars[y_start + i + 1];
            AD<double> theta1 = vars[theta_start + i + 1];

            // t时刻的状态
            AD<double> x0 = vars[x_start + i];
            AD<double> y0 = vars[y_start + i];
            AD<double> theta0 = vars[theta_start + i];

            // 只考虑t时刻的控制
            AD<double> v0 = vars[v_start + i];
            AD<double> omega0 = vars[omega_start + i];

            fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * DT); // x1 = x0+vcos(theta)*dt
            fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * DT); // y1 = y0+vsin(theta)*dt
            fg[2 + theta_start + i] = theta1 - (theta0 + omega0 * DT);      // theta1 = theta0+ometa*dt
        }
    }
};

class MPCPlanner {
public:
    std_msgs::msg::Float64MultiArray msgs_traj_info;
    std::vector<std_msgs::msg::Float64MultiArray> msgs_traj_coef;

    MPCPlanner(std::shared_ptr<Corridor> _corridor_obj, std::shared_ptr<nav_msgs::msg::Path> _initTrajPlanner_obj)
        : corridor_obj(std::move(_corridor_obj))
        , initTrajPlanner_obj(std::move(_initTrajPlanner_obj))

    {
        initTraj = *initTrajPlanner_obj.get();
        N = initTraj.poses.size(); // 路径点的数量

        outdim = 2; // the number of outputs (x,y)

        SFC = corridor_obj.get()->SFC;

        for (int i = 0; i < N; i++) {
            plan[i][0] = initTraj.poses[i].pose.position.x;
            plan[i][1] = initTraj.poses[i].pose.position.y;
        }

        // 角度信息
        for (int i = 0; i < N - 1; i++) {
            plan[i][2] = atan2(plan[i + 1][1] - plan[i][1], plan[i + 1][0] - plan[i][0]);
        }
        plan[N - 1][2] = tf2::getYaw((initTraj.poses.end() - 1)->pose.orientation); //最后一个点的角度

        // 角速度的初值
        for (int i = 0; i < N - 1; i++) {
            plan[i][4] = (plan[i + 1][2] - plan[i][2]) / DT;
        }

        // 都是偏移量
        x_start = 0;
        y_start = x_start + N;
        theta_start = y_start + N;
        v_start = theta_start + N;
        omega_start = v_start + (N - 1);
    }

    void Solve() {
        bool ok = true;
        size_t i;
        typedef CPPAD_TESTVECTOR(double) Dvector;

        // 设定模型变量的数量（包括状态和输入）
        // 状态有3个元素，输入有2个元素
        size_t n_vars = N * 3 + (N - 1) * 2;
        // 约束条件的数量
        size_t n_constraints = N * 3;

        // 自变量的初值
        // SHOULD BE 0 besides initial state.
        Dvector vars(n_vars);

        Dvector vars_lowerbound(n_vars);
        Dvector vars_upperbound(n_vars);

        // 变量的上下限，具体限制在后面设置
        for (int i = 0; i < v_start; i++) {
            vars_lowerbound[i] = -BOUND;
            vars_upperbound[i] = BOUND;
        }

        int count = 0;

        for (int i = 0; i < N; i++) {
            // 状态的初始值
            vars[x_start + i] = plan[i][0];
            vars[y_start + i] = plan[i][1];
            vars[theta_start + i] = plan[i][2];
            // 输入的初始值
            if (i < N - 1) {
                vars[v_start + i] = plan[i][3];
                vars[omega_start + i] = plan[i][4];
            }

            if (i > 1 && i == SFC[count + 1].second && count < SFC.size() - 1) {
                count++;
            }

            // 通过走廊来进行上下限设置
            if (plan[i][0] - SFC[count].first[0] > 2)
                vars_lowerbound[x_start + i] = plan[i][0] - 2;
            else
                vars_lowerbound[x_start + i] = SFC[count].first[0];
            if (SFC[count].first[3] - plan[i][0] > 2)
                vars_upperbound[x_start + i] = plan[i][0] + 2;
            else
                vars_upperbound[x_start + i] = SFC[count].first[2];
            if (plan[i][1] - SFC[count].first[1] > 2)
                vars_lowerbound[y_start + i] = plan[i][1] - 2;
            else
                vars_lowerbound[y_start + i] = SFC[count].first[1];
            if (SFC[count].first[4] - plan[i][1] > 2)
                vars_upperbound[y_start + i] = plan[i][1] + 2;
            else
                vars_upperbound[y_start + i] = SFC[count].first[3];
        }

        for (int i = v_start; i < omega_start; i++) {
            vars_lowerbound[i] = 0;
            vars_upperbound[i] = MAXV;
            // 是否允许后退
            if (1) vars_lowerbound[i] = -MAXV;
        }

        for (int i = omega_start; i < n_vars; i++) {
            vars_lowerbound[i] = -MAXOMEGA;
            vars_upperbound[i] = MAXOMEGA;
        }

        // Lower and upper limits for the constraints
        // Should be 0 besides initial state.
        Dvector constraints_lowerbound(n_constraints);
        Dvector constraints_upperbound(n_constraints);
        for (int i = 0; i < n_constraints; i++) {
            constraints_lowerbound[i] = 0;
            constraints_upperbound[i] = 0;
        }

        double startangle;
        for (int next = 5; next < N; next += 5) {
            if ((plan[next][1] == plan[0][1]) & (plan[next][0] == plan[0][0]))
                continue;
            else {
                startangle = atan2(plan[next][1] - plan[0][1], plan[next][0] - plan[0][0]);
                break;
            }
        }

        constraints_lowerbound[x_start] = plan[0][0];
        constraints_lowerbound[y_start] = plan[0][1];
        constraints_upperbound[x_start] = plan[0][0];
        constraints_upperbound[y_start] = plan[0][1];

        constraints_lowerbound[theta_start] = startangle;
        constraints_upperbound[theta_start] = startangle;

        // 目标函数和约束函数都在这里面定义
        FG_eval fg_eval;
        // IPOPT solver的求解项
        std::string options;

        options += "Numeric tol          1e-5\n";
        options += "String linear_solver mumps\n";
        // Uncomment this if you'd like more print information
        options += "Integer print_level  1\n";
        // NOTE: Setting sparse to true allows the solver to take advantage
        // of sparse routines, this makes the computation MUCH FASTER. If you
        // can uncomment 1 of these and see if it makes a difference or not but
        // if you uncomment both the computation time should go up in orders of
        // magnitude.
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";
        // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
        // Change this as you see fit.

        // place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;

        // 求解！
        // option：求解选项
        // vars：变量
        // vars_lowerbound，vars_upperbound：变量的上下限
        // constraints_lowerbound，constraints_upperbound：不等式限制的上下限
        // fg_eval：用来定义
        CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                                              constraints_upperbound, fg_eval, solution);

        // Check some of the solution values
        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
        auto opt_node = rclcpp::Node::make_shared("optimization");

        if (ok) {
            RCLCPP_INFO(opt_node->get_logger(), "Optimization Success!");
            RCLCPP_INFO(opt_node->get_logger(), "Cost:%lf ", solution.obj_value);
        }

        for (int i = 0; i < N; i++) {
            plan[i][0] = solution.x[x_start + i];
            plan[i][1] = solution.x[y_start + i];
            plan[i][2] = solution.x[theta_start + i];
            plan[i][3] = solution.x[v_start + i];
            plan[i][4] = solution.x[omega_start + i];
        }
    }

    bool update() {
        int qi = 0;

        nav_msgs::msg::Path path;
        geometry_msgs::msg::PoseStamped pp;

        // 主函数，进行路径优化
        Solve();

        path.header.frame_id = "map";
        path.header.stamp = opt_node->get_clock()->now();
        for (int i = 0; i < N; i++) {
            pp.pose.position.x = plan[i][0];
            pp.pose.position.y = plan[i][1];
            tf2::Quaternion q_tmp;
            q_tmp.setRPY(0, 0, plan[i][2]);
            pp.pose.orientation = tf2::toMsg(q_tmp);
            path.poses.push_back(pp);

            if (fabs(plan[i][0] - plan[N - 1][0]) < 0.01) {
                if (fabs(plan[i][1] - plan[N - 1][1]) < 0.01) {
                    break;
                }
            }
        }
        auto opt_pub = opt_node->create_publisher<nav_msgs::msg::Path>("opt_path", 10);
        opt_pub->publish(path);

        return true;
    }

private:
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<nav_msgs::msg::Path> initTrajPlanner_obj;

    nav_msgs::msg::Path initTraj;
    std::vector<double> T;
    SFC_t SFC;

    int phi, outdim;
    std::shared_ptr<rclcpp::Node> opt_node = rclcpp::Node::make_shared("optimization");

    // std::shared_ptr<Eigen::MatrixXd> Q_obj, Aeq_obj, Alq_obj, deq_obj, dlq_obj;
    Eigen::MatrixXd Q_base, Aeq_base, Alq, deq, dlq, basis;
    Eigen::MatrixXd dummy;
    std::vector<Eigen::MatrixXd> coef;
};
