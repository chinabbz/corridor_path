/*
//Title: Common funcation (Delta-NTU Lab)
//Author: Chen, Chun-Lin
//Data: 2015/06/17
//Update: 2016/03/31
*/

#ifndef COMM_H
#define COMM_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
using namespace std;
#define PI 3.1415926
#define DTR PI / 180.0 // Degree to Rad
#define RTD 180.0 / PI // Rad to Degree

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// Euler angle to Quaternion
geometry_msgs::msg::Quaternion Euler_to_Quat(double roll, double pitch, double yaw) {
    tf2::Quaternion tf_quat;
    geometry_msgs::msg::Quaternion msg_quat;
    tf_quat.setRPY(roll, pitch, yaw);
    tf2::convert(tf_quat, msg_quat);
    return msg_quat;
}

// Quaternion to one Euler angle
double Quat_to_Euler(geometry_msgs::msg::Quaternion msg_quat, int sel) {
    tf2::Quaternion tf_quat;
    double roll, pitch, yaw;
    tf2::convert(msg_quat, tf_quat);

    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    if (sel == 0)
        return roll;
    else if (sel == 1)
        return pitch;
    else if (sel == 2)
        return yaw;
}

// Quaternion to Yaw
double Quat_to_Yaw(geometry_msgs::msg::Quaternion msg_quat) {
    double yaw = tf2::getYaw(msg_quat);
    return yaw;
}

// Calculate Distance in XY plane
double DIS_XY(geometry_msgs::msg::PoseStamped pa, geometry_msgs::msg::PoseStamped pb) {
    double distance, dx, dy;
    dx = pa.pose.position.x - pb.pose.position.x;
    dy = pa.pose.position.y - pb.pose.position.y;
    distance = sqrt(dx * dx + dy * dy);
    return distance;
}

// Calculate Distance in XYZ plane
double DIS_XYZ(double pa_x, double pa_y, double pa_z, double pb_x, double pb_y, double pb_z) {
    double distance;
    distance = pow(pow(pa_x - pb_x, 2) + pow(pa_y - pb_y, 2) + pow(pa_z - pb_z, 2), 0.5);
    return distance;
}
#endif // COMM_H
