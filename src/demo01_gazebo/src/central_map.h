#ifndef ROBOTS_H
#define ROBOTS_H

#include <string>
#include <vector>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <fstream>
#include <sstream>
#include "instrument.h"
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind/bind.hpp>
#include "cluster.h"
#include <std_msgs/Float64.h>
#include <thread>
class central_map
{
private:
public:
    std::vector<std::vector<double>> coordinates;
    std::vector<std::vector<double>> start_coordinates;
    std::vector<std::vector<double>> target_coordinates;
    std::vector<cluster> clusters;
    int mapsize_X;
    int mapsize_y;
    double vel_x;
    double vel_y;
    std::vector<ros::Subscriber> subscribers; // 一组订阅者用于订阅机器人的位置信息
    ros::NodeHandle nh;
    std::vector<double> closestPoint1;
    std::vector<double> closestPoint2;
    std::vector<double> centerPoint;
    double dilation_coe=2;
    double search_radius = 2.5;

    central_map()
    {
        upload_startcoo();
        upload_target_coo();
        spawnRobot();
        // upload_realtime_coos();
    };

    void upload_startcoo();
    void upload_target_coo();
    void spawnRobot(); // 传入参数：存储坐标的向量，序号
    void spawnRobots(const std::vector<double> &coordinates, int robotIndex);
    void upload_realtime_coos_cluster();
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, int topic_index);
    int find_nearest_point(const std::vector<std::vector<double>> &start_coordinates, const std::vector<std::vector<double>> &target_coordinates);
    void module_divition();
    void find_centerpoint();
    void dilation();
    void move_thread(cluster &c);
    void move_astep();
    void publishVelocity(int topicNumber);
};
#endif