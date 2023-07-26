#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <vector>

std::vector<double> robotCoordinates;  // 全局变量，用于存储机器人的位置坐标

// 回调函数，接收机器人位置信息
void poseCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  // 查找机器人在ModelStates消息中的索引
  int index = -1;
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i] == "mycar")  // 将 "your_robot_name" 替换为你的机器人模型的名称
    {
      index = i;
      break;
    }
  }

  // 提取机器人的位置坐标
  if (index != -1)
  {
    geometry_msgs::Pose pose = msg->pose[index];
    double x = pose.position.x;
    double y = pose.position.y;

    // 存储位置坐标到向量中
    robotCoordinates = {x, y};

    // 将机器人坐标存储到参数服务器
    ros::NodeHandle nh("~");
    nh.setParam("robot_coordinates", robotCoordinates);
  }
}

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "gazebo_robot_coordinates");

  // 创建节点句柄
  ros::NodeHandle nh;

  // 创建订阅者，订阅Gazebo插件发布的base_pose_ground_truth主题
  ros::Subscriber sub = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 10, poseCallback);

  // 循环等待回调函数
  ros::spin();

  return 0;
}
