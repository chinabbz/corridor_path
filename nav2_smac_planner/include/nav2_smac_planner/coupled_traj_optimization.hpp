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
#include <visualization_msgs/msg/marker_array.hpp>

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Submodules
#include "nav2_smac_planner/com_fun.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>

using CppAD::AD;

#define KMAX 0.6 // 曲率<0.5

size_t x_start;
size_t y_start;
size_t theta_start;
size_t L_start;
size_t k_con;
size_t x_con;
size_t y_con;

class FG_eval {
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    FG_eval(int N_) : N(N_) {}

    void operator()(ADvector& fg, const ADvector& vars) {
        fg[0] = 0;
        // 最小化L的总和

        for (int i = 0; i < 2 * N; i++) {
            fg[0] += CppAD::pow(vars[L_start + i], 2);
        }
        // 曲率限制
        for (int i = 0; i < N; i++) {
            AD<double> L1 = vars[L_start + 2 * i];
            AD<double> L2 = vars[L_start + 2 * i + 1];
            AD<double> alpha = PI - CppAD::abs(vars[theta_start + i + 1] - vars[theta_start + i]);
            fg[1 + k_con + 2 * i] =
                2 * CppAD::sin(alpha) / 3 / L1 / CppAD::pow(CppAD::cos(alpha) * (-0.5) + 0.5, 3 / 2);
            fg[1 + k_con + 2 * i + 1] =
                2 * CppAD::sin(alpha) / 3 / L2 / CppAD::pow(CppAD::cos(alpha) * (-0.5) + 0.5, 3 / 2);
        }
        // x,y限制
        for (int i = 0; i < 2 * N; i++) {
            // t+1时刻的状态
            AD<double> x1 = vars[x_start + i + 1];
            AD<double> y1 = vars[y_start + i + 1];

            // t时刻的状态
            AD<double> x0 = vars[x_start + i];
            AD<double> y0 = vars[y_start + i];

            AD<double> theta = vars[theta_start + (i + 1) / 2];
            AD<double> L = vars[L_start + i];

            fg[1 + x_con + i] = x1 - (x0 + L * CppAD::cos(theta));
            fg[1 + y_con + i] = y1 - (y0 + L * CppAD::sin(theta));
        }
    }

private:
    int N;
};

class MPCPlanner {
public:
    std::shared_ptr<rclcpp::Node> opt_node = rclcpp::Node::make_shared("optimization");
    MPCPlanner(std::shared_ptr<Corridor> _corridor_obj, std::shared_ptr<nav_msgs::msg::Path> _initTrajPlanner_obj)
        : corridor_obj(std::move(_corridor_obj))
        , initTrajPlanner_obj(std::move(_initTrajPlanner_obj))

    {
        initTraj = *initTrajPlanner_obj.get();
        SFC = corridor_obj.get()->SFC;
    }

    bool Solve() {
        RCLCPP_INFO(opt_node->get_logger(), "solve begin");
        bool ok = true;
        size_t i;
        typedef CPPAD_TESTVECTOR(double) Dvector;

        // 设定模型变量的数量{L 2*N} {theta N+1} {x 2*N+1} {y 2*N+1}
        size_t n_vars = 2 * N + N + 1 + 2 * (2 * N + 1);
        // 约束条件的数量：{alpha 2*N}+{x,y 4N}(不包括起始位置)
        size_t n_constraints = 2 * N + 4 * N;

        // 自变量的初值，除了初始状态其余都设为0
        Dvector vars(n_vars);
        Dvector vars_lowerbound(n_vars);
        Dvector vars_upperbound(n_vars);

        double startangle = tf2::getYaw(initTraj.poses[0].pose.orientation);
        double goalangle = tf2::getYaw(initTraj.poses.back().pose.orientation);

        // 提前写好x和y的上下界
        vector<double> x_lowerbound, x_upperbound, y_lowerbound, y_upperbound;
        x_lowerbound.emplace_back(initTraj.poses[0].pose.position.x);
        x_upperbound.emplace_back(initTraj.poses[0].pose.position.x);
        y_lowerbound.emplace_back(initTraj.poses[0].pose.position.y);
        y_upperbound.emplace_back(initTraj.poses[0].pose.position.y);
        for (int i = 1; i < 2 * N; i++) {
            if (i % 2 == 0) {
                x_lowerbound.emplace_back(max(SFC[i / 2 - 1].first[0], SFC[i / 2].first[0]));
                x_upperbound.emplace_back(min(SFC[i / 2 - 1].first[2], SFC[i / 2].first[2]));
                y_lowerbound.emplace_back(max(SFC[i / 2 - 1].first[1], SFC[i / 2].first[1]));
                y_upperbound.emplace_back(min(SFC[i / 2 - 1].first[3], SFC[i / 2].first[3]));
            } else {
                x_lowerbound.emplace_back(SFC[i / 2].first[0]);
                x_upperbound.emplace_back(SFC[i / 2].first[2]);
                y_lowerbound.emplace_back(SFC[i / 2].first[1]);
                y_upperbound.emplace_back(SFC[i / 2].first[3]);
            }
        }
        x_lowerbound.emplace_back(initTraj.poses.back().pose.position.x);
        x_upperbound.emplace_back(initTraj.poses.back().pose.position.x);
        y_lowerbound.emplace_back(initTraj.poses.back().pose.position.y);
        y_upperbound.emplace_back(initTraj.poses.back().pose.position.y);

        // 设定初始值
        vars[x_start] = initTraj.poses[0].pose.position.x;
        vars[y_start] = initTraj.poses[0].pose.position.y;
        vars[x_start + 2 * N] = initTraj.poses.back().pose.position.x;
        vars[y_start + 2 * N] = initTraj.poses.back().pose.position.y;
        vars[theta_start] = startangle;
        vars[theta_start + N + 1] = goalangle;

        double dis = DIS_XY(initTraj.poses[std::abs(SFC[0].second)], initTraj.poses[0]);
        if (SFC[0].second < 0) dis *= -1;
        vars[L_start] = dis / 2;
        vars[L_start + 1] = dis / 2;
        for (int i = 1; i < N; i++) {
            dis = DIS_XY(initTraj.poses[std::abs(SFC[i - 1].second)], initTraj.poses[std::abs(SFC[i].second)]);
            if (SFC[i - 1].second < 0) dis *= -1;
            vars[L_start + 2 * i] = dis / 2;
            vars[L_start + 2 * i + 1] = dis / 2;
        }
        for (int i = 1; i < N + 1; i++) {
            vars[theta_start + i] = tf2::getYaw(initTraj.poses[std::abs(SFC[i - 1].second)].pose.orientation);
        }
        for (int i = 1; i < 2 * N; i++) {
            vars[x_start + i] = vars[x_start + i - 1] + vars[L_start + i / 2] * cos(vars[theta_start + i]);
            vars[y_start + i] = vars[y_start + i - 1] + vars[L_start + i / 2] * sin(vars[theta_start + i]);
        }

        RCLCPP_INFO(opt_node->get_logger(), "initial value set done");

        // 变量的上下限设置
        for (int i = 0; i < 2 * N; i++) {
            if (SFC[i / 2].second >= 0) {
                vars_lowerbound[L_start + i] = 0;
                vars_upperbound[L_start + i] = 10e3;
            } else {
                vars_lowerbound[L_start + i] = -10e2;
                vars_upperbound[L_start + i] = 0;
            }
        }
        for (int i = 1; i < N; i++) {
            vars_lowerbound[theta_start + i] = -PI;
            vars_upperbound[theta_start + i] = PI;
        }
        for (int i = 1; i < 2 * N; i++) {
            vars_lowerbound[x_start + i] = x_lowerbound[i];
            vars_upperbound[x_start + i] = x_upperbound[i];
            vars_lowerbound[y_start + i] = y_lowerbound[i];
            vars_upperbound[y_start + i] = y_upperbound[i];
        }
        vars_lowerbound[x_start] = initTraj.poses[0].pose.position.x;
        vars_upperbound[x_start] = initTraj.poses[0].pose.position.x;
        vars_lowerbound[y_start] = initTraj.poses[0].pose.position.y;
        vars_upperbound[y_start] = initTraj.poses[0].pose.position.y;

        vars_lowerbound[x_start + 2 * N] = initTraj.poses.back().pose.position.x;
        vars_upperbound[x_start + 2 * N] = initTraj.poses.back().pose.position.x;
        vars_lowerbound[y_start + 2 * N] = initTraj.poses.back().pose.position.y;
        vars_upperbound[y_start + 2 * N] = initTraj.poses.back().pose.position.y;

        // 限制末位置的角度
        vars_lowerbound[theta_start] = startangle;
        vars_upperbound[theta_start] = startangle;
        vars_lowerbound[theta_start + N] = goalangle;
        vars_upperbound[theta_start + N] = goalangle;
        RCLCPP_INFO(opt_node->get_logger(), "variant bound set done");

        // 约束条件的上下限设置
        Dvector constraints_lowerbound(n_constraints);
        Dvector constraints_upperbound(n_constraints);
        // k<=0.5
        for (int i = 0; i < 2 * N; i++) {
            constraints_lowerbound[k_con + i] = -KMAX;
            constraints_upperbound[k_con + i] = KMAX;
        }
        // x,y=f(L,theta)
        for (int i = 0; i < 2 * N; i++) {
            constraints_lowerbound[x_con + i] = 0;
            constraints_upperbound[x_con + i] = 0;
            constraints_lowerbound[y_con + i] = 0;
            constraints_upperbound[y_con + i] = 0;
        }
        RCLCPP_INFO(opt_node->get_logger(), "constraints bound set done");

        // 目标函数和约束函数都在这里面定义
        FG_eval fg_eval(N);
        // IPOPT solver的求解项
        std::string options;

        options += "Numeric tol          1e-5\n";
        options += "String linear_solver ma27\n";
        // Uncomment this if you'd like more print information
        options += "Integer print_level  3\n";
        options += "Integer max_iter  4000\n";
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
        if (ok) {
            RCLCPP_INFO(opt_node->get_logger(), "Optimization Success!");
            RCLCPP_INFO(opt_node->get_logger(), "Cost:%lf ", solution.obj_value);
        } else {
            RCLCPP_INFO(opt_node->get_logger(), "Optimization Fail!");
            RCLCPP_INFO(opt_node->get_logger(), "status:%d ", solution.status);
            return false;
        }

        auto opt_node = rclcpp::Node::make_shared("optimization");
        for (int i = 0; i < 2 * N; i++) {
            std::cout << "L" << i << ":" << solution.x[L_start + i] << std::endl;
        }
        for (int i = 0; i < N + 1; i++) {
            std::cout << "theta" << i << ":" << solution.x[theta_start + i] << std::endl;
        }
        // 清除之前的控制点
        auto control_point_node = rclcpp::Node::make_shared("control_point_pub_node");
        auto control_point_publisher =
            control_point_node->create_publisher<visualization_msgs::msg::MarkerArray>("control_point", 10);
        visualization_msgs::msg::MarkerArray points;
        visualization_msgs::msg::Marker apoint;
        apoint.header.frame_id = "map";
        apoint.header.stamp = control_point_node.get()->get_clock()->now();
        apoint.action = visualization_msgs::msg::Marker::DELETEALL;
        points.markers.emplace_back(apoint);
        control_point_publisher->publish(points);
        // 把每个控制点发布出来看一下
        apoint.action = visualization_msgs::msg::Marker::ADD;
        points.markers.clear();
        apoint.type = visualization_msgs::msg::Marker::CUBE;
        apoint.scale.x = 0.1;
        apoint.scale.y = 0.1;
        apoint.scale.z = 0.1;
        apoint.color.a = 1.0;
        apoint.color.r = 255;
        apoint.color.g = 0;
        apoint.color.b = 0;
        for (int i = 0; i < 2 * N + 1; i++) {
            apoint.id = i;
            std::cout << "x,y," << i << ":" << solution.x[x_start + i] << "," << solution.x[y_start + i] << std::endl;
            apoint.pose.position.x = solution.x[x_start + i];
            apoint.pose.position.y = solution.x[y_start + i];
            points.markers.emplace_back(apoint);
        }
        control_point_publisher->publish(points);
        // 发布路径
        auto opt_path = control_point_node->create_publisher<nav_msgs::msg::Path>("opt_path", 10);
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = control_point_node->get_clock()->now();
        geometry_msgs::msg::PoseStamped pp;
        double x_pos, y_pos;
        vector<std::pair<double, double>> cp(5); // 控制点的x,y坐标
        for (int box_i = 0; box_i < N; box_i++) {
            cp[0] = std::make_pair(solution.x[x_start + 2 * box_i], solution.x[y_start + 2 * box_i]);
            cp[1] = std::make_pair((solution.x[x_start + 2 * box_i] + solution.x[x_start + 2 * box_i + 1]) / 2,
                                   (solution.x[y_start + 2 * box_i] + solution.x[y_start + 2 * box_i + 1]) / 2);
            cp[2] = std::make_pair(solution.x[x_start + 2 * box_i + 1], solution.x[y_start + 2 * box_i + 1]);
            cp[3] = std::make_pair((solution.x[x_start + 2 * box_i + 1] + solution.x[x_start + 2 * box_i + 2]) / 2,
                                   (solution.x[y_start + 2 * box_i + 1] + solution.x[y_start + 2 * box_i + 2]) / 2);
            cp[4] = std::make_pair(solution.x[x_start + 2 * box_i + 2], solution.x[y_start + 2 * box_i + 2]);
            for (double t = 0; t <= 1; t += 0.01) {
                x_pos = (1 - t) * (1 - t) * (1 - t) * (1 - t) * cp[0].first;
                x_pos += 4 * t * (1 - t) * (1 - t) * (1 - t) * cp[1].first;
                x_pos += 6 * t * t * (1 - t) * (1 - t) * cp[2].first;
                x_pos += 4 * t * t * t * (1 - t) * cp[3].first;
                x_pos += t * t * t * t * cp[4].first;
                y_pos = (1 - t) * (1 - t) * (1 - t) * (1 - t) * cp[0].second;
                y_pos += 4 * t * (1 - t) * (1 - t) * (1 - t) * cp[1].second;
                y_pos += 6 * t * t * (1 - t) * (1 - t) * cp[2].second;
                y_pos += 4 * t * t * t * (1 - t) * cp[3].second;
                y_pos += t * t * t * t * cp[4].second;
                pp.pose.position.x = x_pos;
                pp.pose.position.y = y_pos;
                path.poses.emplace_back(pp);
            }
        }
        opt_path->publish(path);
        // std::cout << "solve done" << std::endl;
        return true;
    }

    bool update() {
        // 主函数，进行路径优化
        N = SFC.size();
        set_offset(N);
        // std::cout << "before solve" << std::endl;
        if (!Solve()) {
            if (N == 1) {
                SFC.emplace_back(SFC.back());
                N++;
                set_offset(N);
                Solve();
            }
        }
        // std::cout << "out of solve" << std::endl;

        return true;
    }

    void set_offset(int N) {
        std::cout << "corridor nums:" << N << std::endl;
        for (int i = 0; i < N; i++) {
            std::cout << i << " corridor:(" << SFC[i].first[0] << "," << SFC[i].first[1] << "),(" << SFC[i].first[2]
                      << "," << SFC[i].first[3] << ")," << SFC[i].second << ":(";
            std::cout << initTraj.poses[std::abs(SFC[i].second)].pose.position.x << ","
                      << initTraj.poses[std::abs(SFC[i].second)].pose.position.y << ")" << std::endl;
        }
        // 设定偏移量
        L_start = 0;
        theta_start = L_start + 2 * N;
        x_start = theta_start + N + 1; // x和y都是2*N+1
        y_start = x_start + 2 * N + 1;

        k_con = 0;
        x_con = k_con + 2 * N; //没有第一个点
        y_con = x_con + 2 * N;
    }

private:
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<nav_msgs::msg::Path> initTrajPlanner_obj;

    nav_msgs::msg::Path initTraj;
    SFC_t SFC;
    int N;
};
