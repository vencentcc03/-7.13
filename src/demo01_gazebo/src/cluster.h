#ifndef CLUSTER_H
#define CLUSTER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// type表示集合类型，1为一个方块；2为横着；3为竖着；4为四个方块
// vel_x表示x方向速度
class cluster
{
public:
    cluster() : type(0),
                index(4, 0),
                vel_x(4, 0.0),
                vel_y(4, 0.0),
                nto_next_target(4, false),
                sta_point(4, std::vector<double>(2, 0.0)),
                tar_point(4, std::vector<double>(2, 0.0)),
                now_point(4, std::vector<double>(2, 0.0)),
                next_target(4, std::vector<double>(2, 0.0)),
                motor_topic(4, ""),
                vel_msg(4) {}
    int type;
    std::vector<int> index;
    std::vector<double>vel_x;
    std::vector<double> vel_y;
    std::vector<bool> nto_next_target;
    std::vector<std::vector<double>> sta_point;
    std::vector<std::vector<double>> tar_point;
    std::vector<std::vector<double>> now_point;
    std::vector<std::vector<double>> next_target;
    std::vector<std::string> motor_topic;
    std::vector<geometry_msgs::Twist> vel_msg;

    void printValues();
};
#endif