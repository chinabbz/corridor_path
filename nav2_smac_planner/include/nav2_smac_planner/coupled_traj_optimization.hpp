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
size_t L_start;
size_t alpha_con;
size_t x_con; //没有第一个点
size_t y_con;

// std::vector<ros::Publisher> way_point_pub;

std::vector<std::array<int, 3>> relative_pair;
std::vector<int> agent_col_num;

class FG_eval {
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars) {
        fg[0] = 0;
        // 最小化L的总和

        for (int i = 0; i < 2 * N; i++) {
            fg[0] += vars[L_start + i];
        }
        // alpha限制
        for (int i = 0; i < N; i++) {
            fg[1 + alpha_con + i] = PI - CppAD::abs(vars[theta_start + i + 1] - vars[theta_start + i]);
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
        N = SFC.size(); // 走廊的数量

        // 都是偏移量
        L_start = 0;
        theta_start = L_start + 2 * N;
        x_start = theta_start + N + 1; // x和y都是2*N+1
        y_start = x_start + 2 * N + 1;

        alpha_con = 0;
        x_con = alpha_con + N; //没有第一个点
        y_con = x_con + 2 * N;
    }

    void Solve() {
        RCLCPP_INFO(opt_node->get_logger(), "solve begin");
        bool ok = true;
        size_t i;
        typedef CPPAD_TESTVECTOR(double) Dvector;

        // 设定模型变量的数量{L 2*N} {theta N+1} {x 2*N+1} {y 2*N+1}
        size_t n_vars = 2 * N + N + 1 + 2 * (2 * N + 1);
        // 约束条件的数量：{alpha N}+{x,y 4N}(不包括起始位置)
        size_t n_constraints = N + 4 * N;

        // 自变量的初值，除了初始状态其余都设为0
        Dvector vars(n_vars);
        Dvector vars_lowerbound(n_vars);
        Dvector vars_upperbound(n_vars);

        double startangle = tf2::getYaw(initTraj.poses[0].pose.orientation);

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
        for (int i = 0; i < 2 * N; i++) {
            vars[L_start + i] = LMIN;
        }
        // 暂时未限制末位置的角度
        for (int i = 1; i < N + 1; i++) {
            vars[theta_start + i] = 0;
        }
        for (int i = 1; i < 2 * N; i++) {
            vars[x_start + i] = x_lowerbound[i];
            vars[y_start + i] = y_lowerbound[i];
        }
        vars[x_start] = initTraj.poses[0].pose.position.x;
        vars[y_start] = initTraj.poses[0].pose.position.y;
        vars[x_start + 2 * N] = initTraj.poses.back().pose.position.x;
        vars[y_start + 2 * N] = initTraj.poses.back().pose.position.y;
        vars[theta_start] = startangle;
        RCLCPP_INFO(opt_node->get_logger(), "initial value set done");

        // 变量的上下限设置
        for (int i = 0; i < 2 * N; i++) {
            vars_lowerbound[L_start + i] = LMIN;
            vars_upperbound[L_start + i] = 10e3;
        }
        for (int i = 1; i < N + 1; i++) {
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

        vars_lowerbound[theta_start] = startangle;
        vars_upperbound[theta_start] = startangle;
        RCLCPP_INFO(opt_node->get_logger(), "variant bound set done");

        // 约束条件的上下限设置
        Dvector constraints_lowerbound(n_constraints);
        Dvector constraints_upperbound(n_constraints);
        // alpha>alpha_min
        for (int i = 0; i < N; i++) {
            constraints_lowerbound[alpha_con + i] = ALPHAMIN;
            constraints_upperbound[alpha_con + i] = PI + 0.01;
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
        } else {
            RCLCPP_INFO(opt_node->get_logger(), "Optimization Fail!");
            RCLCPP_INFO(opt_node->get_logger(), "status:%d ", solution.status);
        }
        for (int i = 0; i < 2 * N; i++) {
            std::cout << "L" << i << ":" << solution.x[L_start + i] << std::endl;
        }
        for (int i = 0; i < N + 1; i++) {
            std::cout << "theta" << i << ":" << solution.x[theta_start + i] << std::endl;
        }
        // 把每个控制点发布出来看一下
        auto control_point_node = rclcpp::Node::make_shared("control_point_pub_node");
        auto control_point_publisher =
            control_point_node->create_publisher<visualization_msgs::msg::MarkerArray>("control_point", 10);
        visualization_msgs::msg::MarkerArray points;
        visualization_msgs::msg::Marker apoint;
        for (int i = 0; i < 2 * N + 1; i++) {
            apoint.header.frame_id = "map";
            apoint.header.stamp = control_point_node.get()->get_clock()->now();
            apoint.id = i;
            apoint.type = visualization_msgs::msg::Marker::CUBE;
            apoint.scale.x = 0.2;
            apoint.scale.y = 0.2;
            apoint.scale.z = 0.1;
            apoint.color.a = 1.0;
            apoint.color.r = 255;
            apoint.color.g = 0;
            apoint.color.b = 0;
            // color
            // if (i % 4 == 0) {
            //     apoint.color.r = 255;
            //     apoint.color.g = 0;
            //     apoint.color.b = 0;
            //     apoint.color.a = 1.0;
            // }
            // if (i % 4 == 1) {
            //     apoint.color.r = 0;
            //     apoint.color.g = 255;
            //     apoint.color.b = 0;
            //     apoint.color.a = 1.0;
            // }
            // if (i % 4 == 2) {
            //     apoint.color.r = 0;
            //     apoint.color.g = 0;
            //     apoint.color.b = 255;
            //     apoint.color.a = 1.0;
            // }
            // if (i % 4 == 3) {
            //     apoint.color.r = 255;
            //     apoint.color.g = 255;
            //     apoint.color.b = 0;
            //     apoint.color.a = 1.0;
            // }
            std::cout << "x,y," << i << ":" << solution.x[x_start + i] << "," << solution.x[y_start + i] << std::endl;
            apoint.pose.position.x = solution.x[x_start + i];
            apoint.pose.position.y = solution.x[y_start + i];
            points.markers.emplace_back(apoint);
        }
        control_point_publisher->publish(points);
        // 发布路径
        auto opt_path = control_point_node->create_publisher<nav_msgs::msg::Path>("opt_path", 10);
        nav_msgs::msg::Path path;
        geometry_msgs::msg::PoseStamped pp;
        double x_pos, y_pos;
        vector<std::pair<double, double>> cp(5);
        for (int i = 0; i < N; i++) {
            cp[0] =
                std::make_pair<double, double> solution.x[x_start + 2 * (i - 1)] for (double t = 0; t <= 1; t += 0.01) {
                x_pos = *t * t * t * t + 4 * t * t * t * (1 - t) * solution.x[x_start + 2 * (i - 1) + 1] +
                        6 * t * t * (1 - t) * (1 - t) * solution.x[x_start + 2 * i]
            }
        }
    }

    bool update() {
        // 主函数，进行路径优化
        Solve();

        // path.header.frame_id = "map";
        // path.header.stamp = opt_node->get_clock()->now();
        // for (int i = 0; i < N; i++) {
        //     pp.pose.position.x = plan[i][0];
        //     pp.pose.position.y = plan[i][1];
        //     tf2::Quaternion q_tmp;
        //     q_tmp.setRPY(0, 0, plan[i][2]);
        //     pp.pose.orientation = tf2::toMsg(q_tmp);
        //     path.poses.push_back(pp);

        //     if (fabs(plan[i][0] - plan[N - 1][0]) < 0.01) {
        //         if (fabs(plan[i][1] - plan[N - 1][1]) < 0.01) {
        //             break;
        //         }
        //     }
        // }
        // auto opt_pub = opt_node->create_publisher<nav_msgs::msg::Path>("opt_path", 10);
        // opt_pub->publish(path);

        return true;
    }

private:
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<nav_msgs::msg::Path> initTrajPlanner_obj;

    nav_msgs::msg::Path initTraj;
    SFC_t SFC;
};
