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
    double mapsize_minx;
    double mapsize_maxx;
    double mapsize_miny;
    double mapsize_maxy;
    double vel = 1;
    std::vector<ros::Subscriber> subscribers; // 一组订阅者用于订阅机器人的位置信息
    std::vector<ros::Publisher> pubs;
    ros::NodeHandle nh;
    std::vector<double> closestPoint1;
    std::vector<double> closestPoint2;
    std::vector<double> centerPoint;
    double dilation_coe = 2;
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
    void upload_realtime_coos();
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, int robot_index, int cluster_index);
    int find_nearest_point(const std::vector<std::vector<double>> &start_coordinates, const std::vector<std::vector<double>> &target_coordinates);
    void module_divition();
    void find_centerpoint();
    void set_boundary();
    void dilation();
    void complete_next_tar(cluster &c);
    void move_next();
    void generate_pubs();
    void incrementNextTarget();
    void update_coordinates();
    // 以下是a星算法的函数
    int a_star(const Node &start, const Node &goal, int type);
    void is_goal_attainable(const Node &goal, int type,std::vector<std::vector<double>> &closedset);
    void expand_node(std::priority_queue<Node, std::vector<Node>, CompareNodes> &openSet,
                     std::vector<std::vector<double>> &closedSet,
                     Node &current, const Node &goal,int type);
    bool isValidNode(double x, double y, const std::vector<std::vector<double>> &closedset,int type);                                 // 检查是否在边界内，或是障碍物
    bool isInOpenSet(double x, double y, const std::priority_queue<Node, std::vector<Node>, CompareNodes> &openSet);            // 辅助函数：检查节点是否已经在打开列表中
};
#endif