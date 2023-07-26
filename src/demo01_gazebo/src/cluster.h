#ifndef CLUSTER_H
#define CLUSTER_H

#include <string>
#include <vector>
// type表示集合类型，1为一个方块；2为横着；3为竖着；4为四个方块
// vel_x表示x方向速度
class cluster
{
public:
    int type;
    double vel_x = 0;
    double vel_y = 0;
    bool nto_next_target = true;
    int index;
    std::vector<double> sta_point;
    std::vector<double> tar_point;
    std::vector<double> now_point;
    std::vector<double> next_target;
    std::string motor_topic1;
    std::string motor_topic2;
    std::string motor_topic3;
    std::string motor_topic4;
    geometry_msgs::Twist vel_msg;
};
#endif