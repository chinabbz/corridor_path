#pragma once

#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define SP_EPSILON 1e-9
#define SP_EPSILON_FLOAT 1e-6
#define SP_INFINITY 1e+9

const double robot_radius = 0.3;

struct Point {
    Point(double x_, double y_) : x(x_), y(y_){};
    double x;
    double y;
};

typedef std::vector<std::pair<std::vector<double>, int>> SFC_t;

// 只要一个box在另一个内部，就认为是同一个，只判断边界，不判断方向
bool boxEqual(std::vector<double> box1, std::vector<double> box2,
              std::shared_ptr<nav2_costmap_2d::Costmap2D> _costmap_obj) {
    std::vector<unsigned int> box_int1(4, 0), box_int2(4, 0);
    int res = _costmap_obj.get()->getResolution();
    _costmap_obj.get()->worldToMap(box1[0], box1[1], box_int1[0], box_int1[1]);
    _costmap_obj.get()->worldToMap(box1[2], box1[3], box_int1[2], box_int1[3]);
    _costmap_obj.get()->worldToMap(box2[0], box2[1], box_int2[0], box_int2[1]);
    _costmap_obj.get()->worldToMap(box2[2], box2[3], box_int2[2], box_int2[3]);
    // 判断1是否在2内
    if (box_int1[0] >= box_int2[0] - res && box_int1[1] >= box_int2[1] - res && box_int1[2] <= box_int2[2] + res &&
        box_int1[3] <= box_int2[3] + res) {
        return true;
    }
    if (box_int2[0] >= box_int1[0] - res && box_int2[1] >= box_int1[1] - res && box_int2[2] <= box_int1[2] + res &&
        box_int2[3] <= box_int1[3] + res) {
        return true;
    }
    return false;
}

class Corridor {
public:
    SFC_t SFC;                                               // 安全走廊
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_obj; // 地图

    // 初始化函数，
    Corridor(std::shared_ptr<nav_msgs::msg::Path> _initTrajPlanner_obj,
             std::shared_ptr<nav2_costmap_2d::Costmap2D> _costmap_obj)
        : initTrajPlanner_obj(std::move(_initTrajPlanner_obj)), costmap_obj(std::move(_costmap_obj)) {
        initTraj = *initTrajPlanner_obj.get();
        N = initTraj.poses.size() - 1; // the number of segments
    }

    bool update(std::vector<bool> for_or_back) { return updateObsBox(for_or_back); }

private:
    std::shared_ptr<nav_msgs::msg::Path> initTrajPlanner_obj; // 初始路径
    nav_msgs::msg::Path initTraj;
    int N;

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
                if (costmap_obj.get()->getCost(mx, my) == 254) {
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
                if (costmap_obj.get()->getCost(mx, my) == 254) {
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
        return box[0] > costmap_obj.get()->getOriginX() && box[1] > costmap_obj.get()->getOriginY() &&
               box[2] < costmap_obj.get()->getSizeInMetersX() + SP_EPSILON &&
               box[3] < costmap_obj.get()->getSizeInMetersY() + SP_EPSILON;
    }

    bool isPointInBox(const Point& point, const std::vector<double>& box) {
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

    bool expand_box(std::vector<double>& box) {
        double res = costmap_obj.get()->getResolution();
        std::vector<int> box_int;
        for (double bound : box) {
            box_int.emplace_back(round(bound / res)); //将原来的边界转化为以res为步长的整形边界
        }
        std::vector<int> box_initial(box_int);
        std::vector<int> box_update;
        std::vector<bool> axis_cand(4, true); // 左下右上
        int left_axis = 4;

        int i = -1;
        int axis = 0;
        if (isObstacleInBox(box) || !isBoxInBoundary(box)) {
            return false;
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
            if (isBoxInBoundary(box_update) && !isObstacleInBox(box_update)) {
                box_int[i] = box_update[i];
            } else {
                axis_cand[i] = false;
                left_axis--;
            }
            // 看看是否扩展过大了
            if (std::abs(box_int[i] - box_initial[i]) * res > 5) {
                axis_cand[i] = false;
                left_axis--;
            }
        }
        for (int i = 0; i < 4; i++) {
            box[i] = box_int[i] * res;
        }
        std::cout << SFC.size() << " box:" << box[0] << "," << box[1] << "," << box[2] << "," << box[3] << std::endl;
        return true;
    }

    bool updateObsBox(std::vector<bool> for_or_back) {
        double x_next, y_next;

        int count = 0;

        std::vector<double> box_prev{0, 0, 0, 0};
        for (int i = 1; i < N; i++) {
            double x = initTraj.poses[i].pose.position.x;
            double y = initTraj.poses[i].pose.position.y;

            std::vector<double> box;

            x_next = initTraj.poses[i + 1].pose.position.x;
            y_next = initTraj.poses[i + 1].pose.position.y;

            // 判断下一个点是否在上一个box中
            if (isPointInBox(Point(x_next, y_next), box_prev) && for_or_back[i] == for_or_back[i - 1]) {
                continue;
            }

            // std::cout << i << " corridor:" << initTraj.poses[i].pose.position.x << ","
            //           << initTraj.poses[i].pose.position.y << std::endl;
            std::cout << i << ":" << x_next << "," << y_next << " not in " << box_prev[0] << "," << box_prev[1] << ","
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
            if (!expand_box(box)) continue;
            box[0] += robot_radius;
            box[1] += robot_radius;
            box[2] -= robot_radius;
            box[3] -= robot_radius;

            // 如果扩展之后还是一个点
            if (box[2] == box[0] || box[3] == box[1]) {
                continue;
            }
            // 如果两个box相等
            if (SFC.size() > 0 && for_or_back[i] == for_or_back[i - 1] &&
                boxEqual(box, SFC.back().first, costmap_obj)) {
                continue;
            }
            if (SFC.size() > 0) SFC.back().second *= i - 1;
            // box加入到走廊

            SFC.emplace_back(std::make_pair(box, 0)); // SFC元素的second表示该box中最后一个路径点的index
            if (for_or_back[i]) {
                SFC.back().second = 1;
            } else {
                SFC.back().second = -1;
            }

            // std::cout << i << " corridor:" << box[0] << "," << box[1] << "," << box[2] << "," << box[3] << std::endl;

            box_prev = box;
        }
        SFC.back().second *= initTraj.poses.size() - 1;

        return true;
    }
};
