// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <fstream>

#include "Eigen/Core"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner {

using namespace std::chrono; // NOLINT
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

SmacPlannerHybrid::SmacPlannerHybrid()
    : _a_star(nullptr)
    , _collision_checker(nullptr, 1)
    , _smoother(nullptr)
    , _costmap(nullptr)
    , _costmap_downsampler(nullptr) {}

SmacPlannerHybrid::~SmacPlannerHybrid() {
    RCLCPP_INFO(_logger, "Destroying plugin %s of type SmacPlannerHybrid", _name.c_str());
}

void SmacPlannerHybrid::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                                  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
                                  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    _node = parent;
    auto node = parent.lock();
    _logger = node->get_logger();
    _clock = node->get_clock();
    _costmap = costmap_ros->getCostmap();
    _costmap_ros = costmap_ros;
    _name = name;
    _global_frame = costmap_ros->getGlobalFrameID();

    int angle_quantizations;

    // 从params.yaml中读取参数
    nav2_util::declare_parameter_if_not_declared(node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
    nav2_util::declare_parameter_if_not_declared(node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
    node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

    nav2_util::declare_parameter_if_not_declared(node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
    node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
    _angle_bin_size = 2.0 * M_PI / angle_quantizations; // 一个bin的大小
    _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

    nav2_util::declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
    node->get_parameter(name + ".allow_unknown", _allow_unknown);
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
    node->get_parameter(name + ".max_iterations", _max_iterations);

    nav2_util::declare_parameter_if_not_declared(node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
    node->get_parameter(name + ".minimum_turning_radius", _search_info.minimum_turning_radius);
    nav2_util::declare_parameter_if_not_declared(node, name + ".cache_obstacle_heuristic",
                                                 rclcpp::ParameterValue(false));
    node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
    nav2_util::declare_parameter_if_not_declared(node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
    node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".change_penalty", rclcpp::ParameterValue(0.15));
    node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.50));
    node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".cost_penalty", rclcpp::ParameterValue(1.7));
    node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
    node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);

    nav2_util::declare_parameter_if_not_declared(node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
    node->get_parameter(name + ".max_planning_time", _max_planning_time);
    nav2_util::declare_parameter_if_not_declared(node, name + ".lookup_table_size", rclcpp::ParameterValue(1.0));
    node->get_parameter(name + ".lookup_table_size", _lookup_table_size);

    nav2_util::declare_parameter_if_not_declared(node, name + ".motion_model_for_search",
                                                 rclcpp::ParameterValue(std::string("DUBIN")));
    node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);
    _motion_model = fromString(_motion_model_for_search);
    if (_motion_model == MotionModel::UNKNOWN) {
        RCLCPP_WARN(_logger,
                    "Unable to get MotionModel search type. Given '%s', "
                    "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
                    _motion_model_for_search.c_str());
    }

    if (_max_iterations <= 0) {
        RCLCPP_INFO(_logger,
                    "maximum iteration selected as <= 0, "
                    "disabling maximum iterations.");
        _max_iterations = std::numeric_limits<int>::max();
    }

    // convert to grid coordinates
    if (!_downsample_costmap) {
        _downsampling_factor = 1;
    }
    const double minimum_turning_radius_global_coords = _search_info.minimum_turning_radius;
    // 最小转弯半径
    // _search_info.minimum_turning_radius = _search_info.minimum_turning_radius / _downsampling_factor;
    RCLCPP_INFO(_logger, "!!!!minimum_turning_radius is: %.4f, costmap resolution is %.4f, downsampe parameter is %.4f",
                _search_info.minimum_turning_radius, _costmap->getResolution(), _downsampling_factor);
    _search_info.minimum_turning_radius =
        _search_info.minimum_turning_radius / (_costmap->getResolution() * _downsampling_factor);
    _lookup_table_dim = static_cast<float>(_lookup_table_size) / _downsampling_factor;
    // _lookup_table_dim =
    // static_cast<float>(_lookup_table_size) / static_cast<float>(_costmap->getResolution() * _downsampling_factor);
    RCLCPP_INFO(_logger, "!!!!minimum_turning_radius is: %.4f, costmap resolution is %.4f, downsampe parameter is %.4f",
                _search_info.minimum_turning_radius, _costmap->getResolution(), _downsampling_factor);

    // Make sure its a whole number
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

    // Make sure its an odd number
    if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
        RCLCPP_INFO(_logger, "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
                    _lookup_table_dim);
        _lookup_table_dim += 1.0;
    }

    // Initialize collision checker
    _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations);
    _collision_checker.setFootprint(_costmap_ros->getRobotFootprint(), _costmap_ros->getUseRadius(),
                                    findCircumscribedCost(_costmap_ros));

    // 这里的NodeHybrid区分了混合A*与A*
    _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
    _a_star->initialize(_allow_unknown, _max_iterations, std::numeric_limits<int>::max(), _lookup_table_dim,
                        _angle_quantizations);

    // Initialize path smoother
    SmootherParams params;
    params.get(node, name);
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(minimum_turning_radius_global_coords);

    // Initialize costmap downsampler
    if (_downsample_costmap && _downsampling_factor > 1) {
        _costmap_downsampler = std::make_unique<CostmapDownsampler>();
        std::string topic_name = "downsampled_costmap";
        _costmap_downsampler->on_configure(node, _global_frame, topic_name, _costmap, _downsampling_factor);
    }

    _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

    // Setup callback for changes to parameters.
    _parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
        node->get_node_base_interface(), node->get_node_topics_interface(), node->get_node_graph_interface(),
        node->get_node_services_interface());

    _parameter_event_sub =
        _parameters_client->on_parameter_event(std::bind(&SmacPlannerHybrid::on_parameter_event_callback, this, _1));

    RCLCPP_INFO(_logger,
                "Configured plugin %s of type SmacPlannerHybrid with "
                "maximum iterations %i, and %s. Using motion model: %s.",
                _name.c_str(), _max_iterations,
                _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
                toString(_motion_model).c_str());
}

void SmacPlannerHybrid::activate() {
    RCLCPP_INFO(_logger, "Activating plugin %s of type SmacPlannerHybrid", _name.c_str());
    _raw_plan_publisher->on_activate();
    if (_costmap_downsampler) {
        _costmap_downsampler->on_activate();
    }
}

void SmacPlannerHybrid::deactivate() {
    RCLCPP_INFO(_logger, "Deactivating plugin %s of type SmacPlannerHybrid", _name.c_str());
    _raw_plan_publisher->on_deactivate();
    if (_costmap_downsampler) {
        _costmap_downsampler->on_deactivate();
    }
}

void SmacPlannerHybrid::cleanup() {
    RCLCPP_INFO(_logger, "Cleaning up plugin %s of type SmacPlannerHybrid", _name.c_str());
    _a_star.reset();
    _smoother.reset();
    if (_costmap_downsampler) {
        _costmap_downsampler->on_cleanup();
        _costmap_downsampler.reset();
    }
    _raw_plan_publisher.reset();
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(const geometry_msgs::msg::PoseStamped& start,
                                                  const geometry_msgs::msg::PoseStamped& goal) {
    std::lock_guard<std::mutex> lock_reinit(_mutex);
    steady_clock::time_point a = steady_clock::now();

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

    // Downsample costmap, if required
    nav2_costmap_2d::Costmap2D* costmap = _costmap;
    if (_costmap_downsampler) {
        costmap = _costmap_downsampler->downsample(_downsampling_factor);
        _collision_checker.setCostmap(costmap);
    }

    // Set collision checker and costmap information
    _a_star->setCollisionChecker(&_collision_checker);

    // Set starting point, in A* bin search coordinates
    unsigned int mx, my;
    costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
    double orientation_bin = tf2::getYaw(start.pose.orientation) / _angle_bin_size;
    while (orientation_bin < 0.0) {
        orientation_bin += static_cast<float>(_angle_quantizations);
    }
    // This is needed to handle precision issues
    if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
        orientation_bin -= static_cast<float>(_angle_quantizations);
    }
    unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
    RCLCPP_INFO(_logger, "start orientation bin id is %d", orientation_bin_id);
    _a_star->setStart(mx, my, orientation_bin_id);

    // Set goal point, in A* bin search coordinates
    costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
    orientation_bin = tf2::getYaw(goal.pose.orientation) / _angle_bin_size;
    while (orientation_bin < 0.0) {
        orientation_bin += static_cast<float>(_angle_quantizations);
    }
    // This is needed to handle precision issues
    if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
        orientation_bin -= static_cast<float>(_angle_quantizations);
    }
    orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
    _a_star->setGoal(mx, my, orientation_bin_id);

    // Setup message
    nav_msgs::msg::Path plan;
    plan.header.stamp = _clock->now();
    plan.header.frame_id = _global_frame;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    // Compute plan
    NodeHybrid::CoordinateVector path;
    int num_iterations = 0;
    std::string error;
    try {
        if (!_a_star->createPath(path, num_iterations, 0.0)) {
            if (num_iterations < _a_star->getMaxIterations()) {
                error = std::string("no valid path found");
            } else {
                error = std::string("exceeded maximum iterations");
            }
        }
    } catch (const std::runtime_error& e) {
        error = "invalid use: ";
        error += e.what();
    }

    if (!error.empty()) {
        RCLCPP_WARN(_logger, "%s: failed to create plan, %s.", _name.c_str(), error.c_str());
        return plan;
    }

    // Convert to world coordinates
    plan.poses.reserve(path.size() + 1);
    plan.poses.emplace_back(start);
    for (int i = path.size() - 1; i >= 0; --i) {
        pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
        pose.pose.orientation = getWorldOrientation(path[i].theta, _angle_bin_size);
        plan.poses.push_back(pose);
    }

    // Publish raw path for debug
    if (_raw_plan_publisher->get_subscription_count() > 0) {
        _raw_plan_publisher->publish(plan);
    }

    // Find how much time we have left to do smoothing
    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    RCLCPP_INFO(_logger, "Time for planning is: %lf", time_span);
    RCLCPP_INFO(_logger, "plan size is: %d", plan.poses.size());
    double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
    std::cout << "It took " << time_span.count() * 1000 << " milliseconds with " << num_iterations << " iterations."
              << std::endl;
#endif

    // // Smooth plan
    // if (num_iterations > 1 && plan.poses.size() > 6) {
    //     _smoother->smooth(plan, costmap, time_remaining);
    // }

#ifdef BENCHMARK_TESTING
    steady_clock::time_point c = steady_clock::now();
    duration<double> time_span2 = duration_cast<duration<double>>(c - b);
    std::cout << "It took " << time_span2.count() * 1000 << " milliseconds to smooth path." << std::endl;
#endif

    // 原来的path路径点之间距离不一致
    // {
    //     double dx, dy, dis;
    //     auto path_node = rclcpp::Node::make_shared("path_pub");
    //     auto path_pub = path_node->create_publisher<nav_msgs::msg::Path>("path_unifor", 10);
    //     nav_msgs::msg::Path path_uniformity;
    //     path_uniformity.header.frame_id = "map";
    //     path_uniformity.header.stamp = path_node->get_clock()->now();
    //     path_uniformity.poses.emplace_back(plan.poses[0]);
    //     for (int i = 0; i < plan.poses.size(); i++) {
    //         for (int j = i + 1; j < plan.poses.size() - 1; j++) {
    //             dx = plan.poses[j].pose.position.x - plan.poses[i].pose.position.x;
    //             dy = plan.poses[j].pose.position.y - plan.poses[i].pose.position.y;
    //             dis = dx * dx + dy * dy;
    //             // 0.2m一个点
    //             if (dis > 0.04) {
    //                 path_uniformity.poses.emplace_back(plan.poses[j]);
    //                 i = j;
    //                 break;
    //             }
    //         }
    //     }
    //     path_uniformity.poses.emplace_back(plan.poses[plan.poses.size() - 1]);

    //     path_pub->publish(path_uniformity);
    // }

    auto corridor_node = rclcpp::Node::make_shared("corridor_publisher");

    // 判断每个路径点是前进还是后退
    auto reverse_publisher = corridor_node->create_publisher<visualization_msgs::msg::MarkerArray>("reverse_point", 10);
    visualization_msgs::msg::MarkerArray points;
    visualization_msgs::msg::Marker apoint;
    apoint.header.frame_id = "map";
    apoint.header.stamp = corridor_node.get()->get_clock()->now();
    apoint.action = visualization_msgs::msg::Marker::DELETEALL;
    points.markers.emplace_back(apoint);
    reverse_publisher->publish(points);
    // 把每个reverse点发布出来看一下
    apoint.action = visualization_msgs::msg::Marker::ADD;
    points.markers.clear();
    apoint.type = visualization_msgs::msg::Marker::CUBE;
    apoint.scale.x = 0.1;
    apoint.scale.y = 0.1;
    apoint.scale.z = 0.1;
    apoint.color.a = 1.0;
    apoint.color.r = 0;
    apoint.color.g = 0;
    apoint.color.b = 0;
    vector<bool> for_or_back;
    double last_yaw = tf2::getYaw(start.pose.orientation), last_x = start.pose.position.x,
           last_y = start.pose.position.y;
    for (int i = 0; i < plan.poses.size() - 1; i++) {
        double now_x = plan.poses[i].pose.position.x;
        double now_y = plan.poses[i].pose.position.y;
        double now_yaw = tf2::getYaw(plan.poses[i].pose.orientation);

        unsigned int mx, my;
        costmap->worldToMap(now_x, now_y, mx, my);
        std::cout << i << ":(" << now_x << "," << now_y << "," << now_yaw
                  << "):" << static_cast<int>(costmap->getCost(mx, my)) << std::endl;
        // 判断是前进还是后退
        if (now_yaw >= -M_PI_4 && now_yaw <= M_PI_4) { // 朝向x正方向
            if (plan.poses[i + 1].pose.position.x > now_x)
                for_or_back.emplace_back(true);
            else
                for_or_back.emplace_back(false);
        }
        if (now_yaw >= M_PI_4 * 3 || now_yaw <= -M_PI_4 * 3) { // 朝向x负方向
            if (plan.poses[i + 1].pose.position.x < now_x)
                for_or_back.emplace_back(true);
            else
                for_or_back.emplace_back(false);
        }
        if (now_yaw >= M_PI_4 && now_yaw <= M_PI_4 * 3) { // 朝向y正方向
            if (plan.poses[i + 1].pose.position.y > now_y)
                for_or_back.emplace_back(true);
            else
                for_or_back.emplace_back(false);
        }
        if (now_yaw >= -M_PI_4 * 3 && now_yaw <= -M_PI_4) { // 朝向y负方向
            if (plan.poses[i + 1].pose.position.y < now_y)
                for_or_back.emplace_back(true);
            else
                for_or_back.emplace_back(false);
        }
        apoint.id = i;
        apoint.pose.position.x = now_x;
        apoint.pose.position.y = now_y;
        if (for_or_back.back()) {
            apoint.color.g = 255;
        } else {
            apoint.color.b = 255;
        }
        points.markers.emplace_back(apoint);
    }
    reverse_publisher->publish(points);

    steady_clock::time_point c = steady_clock::now();

    // 建立走廊
    std::shared_ptr<Corridor> corridor_obj; // 走廊
    corridor_obj.reset(new Corridor(std::make_shared<nav_msgs::msg::Path>(plan),
                                    std::make_shared<nav2_costmap_2d::Costmap2D>(*costmap)));
    if (!corridor_obj.get()->update(for_or_back)) {
        RCLCPP_ERROR(_logger, "no corridor!!!");
    }
    // 清除之前的走廊
    auto corridor_publisher = corridor_node->create_publisher<visualization_msgs::msg::MarkerArray>("corridor", 10);
    visualization_msgs::msg::MarkerArray boxes;
    visualization_msgs::msg::Marker abox;
    abox.header.frame_id = "map";
    abox.header.stamp = corridor_node.get()->get_clock()->now();
    abox.id = 0;
    abox.action = visualization_msgs::msg::Marker::DELETEALL;
    boxes.markers.emplace_back(abox);
    corridor_publisher->publish(boxes);
    // 发布走廊
    boxes.markers.clear();
    abox.action = visualization_msgs::msg::Marker::ADD;
    for (int i = 0; i < corridor_obj.get()->SFC.size(); i++) {
        abox.header.frame_id = "map";
        abox.header.stamp = corridor_node.get()->get_clock()->now();
        abox.id = i;
        abox.type = visualization_msgs::msg::Marker::CUBE;
        abox.scale.x = corridor_obj.get()->SFC[i].first[2] - corridor_obj.get()->SFC[i].first[0];
        abox.scale.y = corridor_obj.get()->SFC[i].first[3] - corridor_obj.get()->SFC[i].first[1];
        abox.scale.z = 0.1;
        abox.color.a = 0.2;
        // color
        if (i % 4 == 0) {
            abox.color.r = 255;
            abox.color.g = 0;
            abox.color.b = 0;
        }
        if (i % 4 == 1) {
            abox.color.r = 0;
            abox.color.g = 255;
            abox.color.b = 0;
        }
        if (i % 4 == 2) {
            abox.color.r = 0;
            abox.color.g = 0;
            abox.color.b = 255;
        }
        if (i % 4 == 3) {
            abox.color.r = 255;
            abox.color.g = 255;
            abox.color.b = 0;
        }
        abox.pose.position.x = (corridor_obj.get()->SFC[i].first[2] + corridor_obj.get()->SFC[i].first[0]) / 2;
        abox.pose.position.y = (corridor_obj.get()->SFC[i].first[3] + corridor_obj.get()->SFC[i].first[1]) / 2;
        std::cout << i << " corridor:(" << corridor_obj.get()->SFC[i].first[0] << ","
                  << corridor_obj.get()->SFC[i].first[1] << "),(" << corridor_obj.get()->SFC[i].first[2] << ","
                  << corridor_obj.get()->SFC[i].first[3] << ")," << corridor_obj.get()->SFC[i].second << std::endl;
        boxes.markers.emplace_back(abox);
    }
    corridor_publisher->publish(boxes);
    // 优化
    std::cout << "ready to optimization" << std::endl;
    std::shared_ptr<MPCPlanner> MPCPlanner_obj; // 轨迹优化
    MPCPlanner_obj.reset(new MPCPlanner(corridor_obj, std::make_shared<nav_msgs::msg::Path>(plan)));
    if (MPCPlanner_obj.get()->update(true, plan)) {
        tf2::Quaternion q;
        for (int i = 0; i < plan.poses.size() - 1; i++) {
            float yaw = atan2(plan.poses[i + 1].pose.position.y - plan.poses[i].pose.position.y,
                              plan.poses[i + 1].pose.position.x - plan.poses[i].pose.position.x);

            q.setRPY(0, 0, yaw);
            plan.poses[i].pose.set__orientation(tf2::toMsg(q));
        }
        plan.poses.back().pose.set__orientation(tf2::toMsg(q));
    }

    steady_clock::time_point d = steady_clock::now();
    duration<double> time_span_corridor = duration_cast<duration<double>>(d - c);
    std::cout << "It took " << time_span_corridor.count() * 1000 << " milliseconds to smooth path." << std::endl;
    return plan;
} // namespace nav2_smac_planner

void SmacPlannerHybrid::on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    std::lock_guard<std::mutex> lock_reinit(_mutex);

    bool reinit_collision_checker = false;
    bool reinit_a_star = false;
    bool reinit_downsampler = false;
    bool reinit_smoother = false;

    for (auto& changed_parameter : event->changed_parameters) {
        const auto& type = changed_parameter.value.type;
        const auto& name = changed_parameter.name;
        const auto& value = changed_parameter.value;

        if (type == ParameterType::PARAMETER_DOUBLE) {
            if (name == _name + ".max_planning_time") {
                _max_planning_time = value.double_value;
            } else if (name == _name + ".lookup_table_size") {
                reinit_a_star = true;
                _lookup_table_size = value.double_value;
            } else if (name == _name + ".minimum_turning_radius") {
                reinit_a_star = true;
                reinit_smoother = true;
                _search_info.minimum_turning_radius = static_cast<float>(value.double_value);
            } else if (name == _name + ".reverse_penalty") {
                reinit_a_star = true;
                _search_info.reverse_penalty = static_cast<float>(value.double_value);
            } else if (name == _name + ".change_penalty") {
                reinit_a_star = true;
                _search_info.change_penalty = static_cast<float>(value.double_value);
            } else if (name == _name + ".non_straight_penalty") {
                reinit_a_star = true;
                _search_info.non_straight_penalty = static_cast<float>(value.double_value);
            } else if (name == _name + ".cost_penalty") {
                reinit_a_star = true;
                _search_info.cost_penalty = static_cast<float>(value.double_value);
            } else if (name == _name + ".analytic_expansion_ratio") {
                reinit_a_star = true;
                _search_info.analytic_expansion_ratio = static_cast<float>(value.double_value);
            }
        } else if (type == ParameterType::PARAMETER_BOOL) {
            if (name == _name + ".downsample_costmap") {
                reinit_downsampler = true;
                _downsample_costmap = value.bool_value;
            } else if (name == _name + ".allow_unknown") {
                reinit_a_star = true;
                _allow_unknown = value.bool_value;
            } else if (name == _name + ".cache_obstacle_heuristic") {
                reinit_a_star = true;
                _search_info.cache_obstacle_heuristic = value.bool_value;
            }
        } else if (type == ParameterType::PARAMETER_INTEGER) {
            if (name == _name + ".downsampling_factor") {
                reinit_a_star = true;
                reinit_downsampler = true;
                _downsampling_factor = value.integer_value;
            } else if (name == _name + ".max_iterations") {
                reinit_a_star = true;
                _max_iterations = value.integer_value;
                if (_max_iterations <= 0) {
                    RCLCPP_INFO(_logger,
                                "maximum iteration selected as <= 0, "
                                "disabling maximum iterations.");
                    _max_iterations = std::numeric_limits<int>::max();
                }
            } else if (name == _name + ".angle_quantization_bins") {
                reinit_collision_checker = true;
                reinit_a_star = true;
                int angle_quantizations = value.integer_value;
                _angle_bin_size = 2.0 * M_PI / angle_quantizations;
                _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
            }
        } else if (type == ParameterType::PARAMETER_STRING) {
            if (name == _name + ".motion_model_for_search") {
                reinit_a_star = true;
                _motion_model = fromString(value.string_value);
                if (_motion_model == MotionModel::UNKNOWN) {
                    RCLCPP_WARN(_logger,
                                "Unable to get MotionModel search type. Given '%s', "
                                "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
                                _motion_model_for_search.c_str());
                }
            }
        }
    }

    // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
    if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
        // convert to grid coordinates
        if (!_downsample_costmap) {
            _downsampling_factor = 1;
        }
        const double minimum_turning_radius_global_coords = _search_info.minimum_turning_radius;
        // _search_info.minimum_turning_radius = _search_info.minimum_turning_radius / _downsampling_factor;
        _search_info.minimum_turning_radius / (_costmap->getResolution() * _downsampling_factor);
        _lookup_table_dim = static_cast<float>(_lookup_table_size) / _downsampling_factor;
        // static_cast<float>(_costmap->getResolution() * _downsampling_factor);

        // Make sure its a whole number
        _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

        // Make sure its an odd number
        if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
            RCLCPP_INFO(_logger, "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
                        _lookup_table_dim);
            _lookup_table_dim += 1.0;
        }

        // Re-Initialize A* template
        if (reinit_a_star) {
            _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
            _a_star->initialize(_allow_unknown, _max_iterations, std::numeric_limits<int>::max(), _lookup_table_dim,
                                _angle_quantizations);
        }

        // Re-Initialize costmap downsampler
        if (reinit_downsampler) {
            if (_downsample_costmap && _downsampling_factor > 1) {
                auto node = _node.lock();
                std::string topic_name = "downsampled_costmap";
                _costmap_downsampler = std::make_unique<CostmapDownsampler>();
                _costmap_downsampler->on_configure(node, _global_frame, topic_name, _costmap, _downsampling_factor);
            }
        }

        // Re-Initialize collision checker
        if (reinit_collision_checker) {
            _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations);
            _collision_checker.setFootprint(_costmap_ros->getRobotFootprint(), _costmap_ros->getUseRadius(),
                                            findCircumscribedCost(_costmap_ros));
        }

        // Re-Initialize smoother
        if (reinit_smoother) {
            auto node = _node.lock();
            SmootherParams params;
            params.get(node, _name);
            _smoother = std::make_unique<Smoother>(params);
            _smoother->initialize(minimum_turning_radius_global_coords);
        }
    }
}

} // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerHybrid, nav2_core::GlobalPlanner)
