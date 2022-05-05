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
    Point(double x_, double y_) : x(x_), y(y_){};
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
    bool isObstacleInBox(const std::vector<double>& box) {
        double x, y;
        // 遍历box中的每个点
        for (double i = box[0]; i < box[2] + SP_EPSILON_FLOAT; i += 0.1) {
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
        }

        return false;
    }
    bool isObstacleInBox(const std::vector<int>& box_int) {
        std::vector<double> box;
        for (int bound : box_int) {
            box.emplace_back(bound * costmap_obj.get()->getResolution());
        }
        double x, y;
        // 遍历box中的每个点
        for (double i = box[0]; i < box[2] + SP_EPSILON_FLOAT; i += 0.1) {
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
        }

        return false;
    }

    bool isBoxInBoundary(const std::vector<double>& box) {
        // std::cout << "in bound" << std::endl;
        return box[0] > -costmap_obj.get()->getSizeInMetersX() - SP_EPSILON &&
               box[1] > -costmap_obj.get()->getSizeInMetersY() - SP_EPSILON &&
               box[2] < costmap_obj.get()->getSizeInMetersX() + SP_EPSILON &&
               box[3] < costmap_obj.get()->getSizeInMetersY() + SP_EPSILON;
    }
    bool isBoxInBoundary(const std::vector<int>& box_int) {
        std::vector<double> box;
        for (int bound : box_int) {
            box.emplace_back(bound * costmap_obj.get()->getResolution());
        }
        return box[0] > -costmap_obj.get()->getSizeInMetersX() - SP_EPSILON &&
               box[1] > -costmap_obj.get()->getSizeInMetersY() - SP_EPSILON &&
               box[2] < costmap_obj.get()->getSizeInMetersX() + SP_EPSILON &&
               box[3] < costmap_obj.get()->getSizeInMetersY() + SP_EPSILON;
    }

    bool isPointInBox(const Point& point, const std::vector<double>& box) {
        // return point.x > box[0] - SP_EPSILON && point.y > box[1] - SP_EPSILON && point.x < box[2] + SP_EPSILON &&
        //        point.y < box[3] + SP_EPSILON;
        // return point.x > box[0] && point.y > box[1] && point.x < box[2] && point.y < box[3];
        if (point.x < box[0]) {
            std::cout << point.x << "<" << box[0] << std::endl;
            return false;
        }
        if (point.x > box[2]) {
            std::cout << point.x << ">" << box[2] << std::endl;
            return false;
        }
        if (point.y < box[1]) {
            std::cout << point.y << "<" << box[1] << std::endl;
            return false;
        }
        if (point.y > box[3]) {
            std::cout << point.y << ">" << box[3] << std::endl;
            return false;
        }
        return true;
    }

    void expand_box(std::vector<double>& box) {
        double res = costmap_obj.get()->getResolution();
        std::vector<int> box_int;
        for (double bound : box) {
            box_int.emplace_back(round(bound / res)); //将原来的边界转化为以res为基元的整形边界
        }
        std::vector<int> box_update;
        std::vector<bool> axis_cand(4, true); // 左下右上
        int left_axis = 4;

        int i = -1;
        int axis = 0;
        if (isObstacleInBox(box) || !isBoxInBoundary(box)) {
            return;
        }
        while (left_axis > 0) {
            i = (i + 1) % 4;
            if (axis_cand[i] == false) {
                continue;
            }
            box_update = box_int;
            switch (i) {
                case 0:
                    box_update[2] = box_int[0];
                    box_update[0]--;
                    break;
                case 1:
                    box_update[3] = box_int[1];
                    box_update[1]--;
                    break;
                case 2:
                    box_update[0] = box_int[2];
                    box_update[2]++;
                    break;
                case 3:
                    box_update[1] = box_int[3];
                    box_update[3]++;
                    break;
                default:
                    break;
            }
            // std::cout << " box_update " << i << ":" << box_update[0] << "," << box_update[1] << "," << box_update[2]
            //           << "," << box_update[3] << std::endl;
            if (!isObstacleInBox(box_update) && isBoxInBoundary(box_update)) {
                box_int[i] = box_update[i];
            } else {
                axis_cand[i] = false;
                left_axis--;
            }
        }
        for (int i = 0; i < 4; i++) {
            box[i] = box_int[i] * res;
        }
        std::cout << SFC.size() << " box:" << box[0] << "," << box[1] << "," << box[2] << "," << box[3] << std::endl;
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
            std::cout << x_next << "," << y_next << " not in " << box_prev[0] << "," << box_prev[1] << ","
                      << box_prev[2] << "," << box_prev[3] << std::endl;

            // 初始化box={x下界，y下界，x上界 ，y上界}
            box.emplace_back(std::min(x, x_next));
            box.emplace_back(std::min(y, y_next));
            box.emplace_back(std::max(x, x_next));
            box.emplace_back(std::max(y, y_next));

            // 判断box里是否有障碍物
            if (isObstacleInBox(box)) {
                // 有障碍物就将box置为next点
                box.clear();
                box.emplace_back(round(x_next));
                box.emplace_back(round(y_next));
                box.emplace_back(round(x_next));
                box.emplace_back(round(y_next));
            }
            expand_box(box);

            if (box[2] == box[0] || box[3] == box[1]) { // 如果扩展之后还是一个点
                continue;
            }

            // box加入到走廊
            SFC.emplace_back(std::make_pair(box, i + 1));
            // std::cout << i << " corridor:" << box[0] << "," << box[1] << "," << box[2] << "," << box[3] << std::endl;

            box_prev = box;
        }

        return true;
    }
};
