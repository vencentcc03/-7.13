#ifndef CLUSTER_H
#define CLUSTER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <queue>
#include <cmath>
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
    int cluster_index;
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
class Node
{
public:
    double x, y;
    int g; // 从起始点到当前节点的步数
    int h; // 从当前节点到目标点的预估步数
    int f; // f = g + h

    Node(double x, double y) : x(x), y(y), g(0), h(0), f(0) {
    }
    void node_print(){
        ROS_INFO("x,y:%f,%f",x,y);
        ROS_INFO("g,h,f:%d,%d,%d",g,h,f);
    }
};

// 计算当前节点到目标节点的预估步数（这里使用曼哈顿距离）
int heuristic(Node &current, Node goal)
{
    int a = abs(current.x - goal.x) + abs(current.y - goal.y);
     return a;
}

// 定义比较函数对象用于优先队列
struct CompareNodes
{
    bool operator()(const Node &a, const Node &b)
    {
        // 比较两个 Node 的 f 值，优先级为 f 值较小的节点
        return a.f > b.f;
    }
};
#endif