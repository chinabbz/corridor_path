#pragma once

#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include "nav2_smac_planner/optimization.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define SP_EPSILON 1e-9
#define SP_EPSILON_FLOAT 1e-6
#define SP_INFINITY 1e+9

#define SP_PT_RBP 0
#define SP_PT_SCP 1
#define SP_IPT_ECBS 0

struct Point {
    Point(int x_, int y_) : x(x_), y(y_){};
    double x;
    double y;
};

// Set the timestep length
size_t N;
typedef std::vector<std::pair<std::vector<double>, double>> SFC_t;

double plan[500][5];
double quad_size = 0.3; //机器人半径

class Corridor {
public:
    SFC_t SFC; // 安全走廊

    // 初始化函数，
    Corridor(std::shared_ptr<nav_msgs::msg::Path> _initTrajPlanner_obj,
             std::shared_ptr<nav2_costmap_2d::Costmap2D> _costmap_obj)
        : initTrajPlanner_obj(std::move(_initTrajPlanner_obj)), costmap_obj(std::move(_costmap_obj)) {
        initTraj = *initTrajPlanner_obj.get();
        for (int i = 0; i < initTraj.poses.size(); i++) {
            T.emplace_back(i * 0.5); // 时间，假设每段分配0.5s
        }
        makespan = T.back(); // 完成所需时间
        M = T.size() - 1;    // the number of segments

        for (int i = 0; i < M; i++) {
            plan[i][0] = initTraj.poses[i].pose.position.x;
            plan[i][1] = initTraj.poses[i].pose.position.y;
        }
        N = M;
    }

    bool update(bool log) { return updateObsBox(log); }

private:
    std::shared_ptr<nav_msgs::msg::Path> initTrajPlanner_obj; // 初始路径
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_obj;  // 地图

    nav_msgs::msg::Path initTraj;
    std::vector<double> T;
    double makespan;
    int M;

    // 判断box中是否有障碍物，有的话返回true
    bool isObstacleInBox(const std::vector<double>& box, double margin) {
        double x, y;
        int count1 = 0;
        // 遍历box中的每个点
        for (double i = box[0]; i < box[2] + SP_EPSILON_FLOAT; i += 0.1) {
            int count2 = 0;
            for (double j = box[1]; j < box[3] + SP_EPSILON_FLOAT; j += 0.1) {
                x = i;
                y = j;

                // 因为有膨胀层的存在，所以只要有代价，就认为有碰撞风险
                unsigned int mx, my;
                costmap_obj.get()->worldToMap(x, y, mx, my);
                if (costmap_obj.get()->getCost(mx, my) > 0) {
                    return true;
                }
            }
            count2++;
        }
        count1++;

        return false;
    }

    bool isBoxInBoundary(const std::vector<double>& box) {
        // std::cout << "in bound" << std::endl;
        return box[0] > -costmap_obj.get()->getSizeInMetersX() - SP_EPSILON &&
               box[1] > -costmap_obj.get()->getSizeInMetersY() - SP_EPSILON &&
               box[2] < costmap_obj.get()->getSizeInMetersX() + SP_EPSILON &&
               box[3] < costmap_obj.get()->getSizeInMetersY() + SP_EPSILON;
    }

    bool isPointInBox(const Point& point, const std::vector<double>& box) {
        return point.x > box[0] - SP_EPSILON && point.y > box[1] - SP_EPSILON && point.x < box[2] + SP_EPSILON &&
               point.y < box[3] + SP_EPSILON;
    }

    void expand_box(std::vector<double>& box, double margin) {
        std::vector<double> box_cand, box_update;
        std::vector<int> axis_cand{0, 1, 2, 3};

        // visualization_msgs::msg::MarkerArray boxes;
        // auto box_node = rclcpp::Node::make_shared("box_publisher");
        // auto box_publisher = box_node->create_publisher<visualization_msgs::msg::MarkerArray>("boxes", 10);
        // int box_id = 0;

        int i = -1;
        int axis;
        while (!axis_cand.empty()) {
            box_cand = box;
            box_update = box;

            // check update_box only! update_box + current_box = cand_box
            // std::cout << "obs: " << isObstacleInBox(box_update, margin) << ", in: " << isBoxInBoundary(box_update)
            //           << std::endl;
            // std::cout << "meter x: " << costmap_obj.get()->getSizeInMetersX() + SP_EPSILON
            //           << ", meter y: " << costmap_obj.get()->getSizeInMetersY() + SP_EPSILON << std::endl;
            // std::cout << "box 0: " << box_update[0] << ", box 1:" << box_update[1] << ", box 2:" << box_update[2]
            //           << ", box 3:" << box_update[3] << std::endl;
            while (!isObstacleInBox(box_update, margin) && isBoxInBoundary(box_update)) {
                i++;
                if (i >= axis_cand.size()) {
                    i = 0;
                }
                axis = axis_cand[i];

                // 更新当前box
                box = box_cand;
                box_update = box_cand;

                // expand cand_box and get updated part of box(update_box)
                // 扩展下界
                if (axis < 2) {
                    box_update[axis + 2] =
                        box_cand[axis]; // 因为box_update只考虑增长的大小，所以这里将update的上界设为box_cand的下界
                    box_cand[axis] = box_cand[axis] - 0.1; // 下界扩展res
                    box_update[axis] = box_cand[axis];
                } else {
                    box_update[axis - 2] = box_cand[axis];
                    box_cand[axis] = box_cand[axis] + 0.1;
                    box_update[axis] = box_cand[axis];
                }
                // 把每个box_update打印出来
                // visualization_msgs::msg::Marker abox;
                // abox.header.frame_id = "map";
                // abox.header.stamp = box_node.get()->get_clock()->now();
                // abox.id = box_id++;
                // abox.type = visualization_msgs::msg::Marker::CUBE;
                // abox.scale.x = box_update[2] - box_update[0];
                // abox.scale.y = box_update[3] - box_update[1];
                // abox.scale.z = 0.1;
                // abox.color.a = 0.5;
                // if (axis == 0) {
                //     abox.color.r = 5 * box_id;
                //     abox.color.g = 0;
                //     abox.color.b = 0;
                //     abox.color.a = 0.02 * (box_id + 2);
                // }
                // if (axis == 1) {
                //     abox.color.r = 0;
                //     abox.color.g = 5 * box_id;
                //     abox.color.b = 0;
                //     abox.color.a = 0.02 * (box_id + 2);
                // }
                // if (axis == 2) {
                //     abox.color.r = 0;
                //     abox.color.g = 0;
                //     abox.color.b = 5 * box_id;
                //     abox.color.a = 0.02 * (box_id + 2);
                // }
                // if (axis == 3) {
                //     abox.color.r = 5 * box_id;
                //     abox.color.g = 5 * box_id;
                //     abox.color.b = 0;
                //     abox.color.a = 0.02 * (box_id + 2);
                // }

                // abox.pose.position.x = (box_update[2] + box_update[0]) / 2;
                // abox.pose.position.y = (box_update[3] + box_update[1]) / 2;
                // boxes.markers.emplace_back(abox);
                // std::cout << "box 0: " << box_update[0] << ", box 1:" << box_update[1] << ", box 2:" << box_update[2]
                //           << ", box 3:" << box_update[3] << std::endl;
                // std::cout << i << std::endl;
            }
            axis_cand.erase(axis_cand.begin() + i);
            if (i > 0) {
                i--;
            } else {
                i = axis_cand.size() - 1;
            }
        }
        // box_publisher->publish(boxes);
    }

    bool updateObsBox(bool log) {
        double x_next, y_next;

        int count = 0;

        std::vector<double> box_prev{0, 0, 0, 0};
        for (int i = 0; i < N - 1; i++) {
            double x = plan[i][0];
            double y = plan[i][1];

            std::vector<double> box;

            x_next = plan[i + 1][0];
            y_next = plan[i + 1][1];

            // 判断当前点是否在上一个box中
            if (isPointInBox(Point(x_next, y_next), box_prev)) {
                continue;
            }

            // 初始化box={x下界，y下界，x上界 ，y上界}
            box.emplace_back(std::min(x, x_next));
            box.emplace_back(std::min(y, y_next));
            box.emplace_back(std::max(x, x_next));
            box.emplace_back(std::max(y, y_next));

            // 判断box里是否有障碍物，quad_size是机器人半径
            if (isObstacleInBox(box, quad_size + 0.1)) {
                // 有障碍物就将box置为next点
                box.clear();
                box.emplace_back(round(x_next));
                box.emplace_back(round(y_next));
                box.emplace_back(round(x_next));
                box.emplace_back(round(y_next));
            }
            expand_box(box, quad_size + 0.1);

            if (box[2] == box[0] || box[3] == box[1]) { // 如果扩展之后还是一个点
                box.emplace_back(round(x_next));
                box.emplace_back(round(y_next));
                box.emplace_back(round(x_next));
                box.emplace_back(round(y_next));
                expand_box(box, quad_size + 0.05);
                count++;
            }
            // box加入到走廊
            SFC.emplace_back(std::make_pair(box, i + 1));
            std::cout << i << " corridor:" << box[0] << "," << box[1] << "," << box[2] << "," << box[3] << std::endl;

            box_prev = box;
        }

        std::cout << "Safe Corridor construction size: " << count << std::endl;

        return true;
    }
};
