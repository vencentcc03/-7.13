#include "cluster.h"
#include <ros/ros.h>
#include <string>

// 成员函数，用ROS_INFO打印所有成员变量的值
void cluster::printValues()
{
    // ROS_INFO("type: %d", type);
    // ROS_INFO("cluster_index: %d", cluster_index);

    // ROS_INFO("index: ");
    // for (int i : index)
    // {
    //     ROS_INFO("%d ", i);
    // }

    // ROS_INFO("vel_x: ");
    // for (double v : vel_x)
    // {
    //     ROS_INFO("%f ", v);
    // }

    // ROS_INFO("vel_y: ");
    // for (double v : vel_y)
    // {
    //     ROS_INFO("%f ", v);
    // }

    // ROS_INFO("nto_next_target: ");
    // for (bool b : nto_next_target)
    // {
    //     ROS_INFO("%s ", b ? "true" : "false");
    // }

    // // 二维向量的打印
    // ROS_INFO("sta_point: ");
    // for (const auto &point : sta_point)
    // {
    //     ROS_INFO("(%f, %f) ", point[0], point[1]);
    // }

    // // 二维向量的打印
    // ROS_INFO("tar_point: ");
    // for (const auto &point : tar_point)
    // {
    //     ROS_INFO("(%f, %f) ", point[0], point[1]);
    // }

    // // 二维向量的打印
    // ROS_INFO("now_point: ");
    // for (const auto &point : now_point)
    // {
    //     ROS_INFO("(%f, %f) ", point[0], point[1]);
    // }

    // // 三维向量的打印
    // ROS_INFO("next_target: ");
    // for (const auto &point : next_target)
    // {
    //     ROS_INFO("(%f, %f) ", point[0], point[1]);
    // }

    // ROS_INFO("motor_topic: ");
    // for (const std::string &topic : motor_topic)
    // {
    //     ROS_INFO("%s ", topic.c_str());
    // }

    // // 其他成员变量的打印
    // // ...

    // ROS_INFO("");
}
