#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <fstream>
#include <sstream>
#include "readfile.h"
#include <ros/package.h>
void spawnRobot(const std::vector<double>& coordinates, int robotIndex) {//传入参数：存储坐标的向量，序号
//依次打开第一个1到第i个urdf文件
robotIndex++;
std::string baseModelPath = "/home/haichao/demo_ws/src/demo01_gazebo/urdf_create/mycar_";
    std::string fileExtension = ".urdf";
        std::stringstream ss;
        ss << baseModelPath << robotIndex << fileExtension;
        std::string modelPath = ss.str();
  std::ifstream file(modelPath);
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string modelXml = buffer.str();
//将坐标信息存储到gazebo_msg的消息类型中
  gazebo_msgs::SpawnModel srv;
  srv.request.model_xml = modelXml;
  srv.request.model_name = "robot_" + std::to_string(robotIndex);
  srv.request.robot_namespace = "/";
  srv.request.initial_pose.position.x = coordinates[0];
  srv.request.initial_pose.position.y = coordinates[1];
  srv.request.reference_frame = "world";
//向gazebo中加载urdf模型
  ros::service::waitForService("/gazebo/spawn_urdf_model");
  ros::ServiceClient spawnClient = ros::NodeHandle().serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

  if (spawnClient.call(srv)) {
    ROS_INFO("Successfully spawned robot_%d", robotIndex);
  } else {
    ROS_ERROR("Failed to spawn robot_%d", robotIndex);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "spawn_robots");
  ros::NodeHandle nh;

  std::string filepath = "/home/haichao/demo_ws/src/demo01_gazebo/coordinate/start_coordinates.csv"; // 你的坐标文件路径
  std::vector<std::vector<double>> coordinates;//定义一个向量用于储存从文本文件中读取的坐标信息
  read_file(filepath, coordinates);//自定义函数，传入存储坐标信息的文本文件的路径，返回坐标信息并存储于向量中

  // 生成机器人
  for (int i = 0; i < coordinates.size(); ++i) {
    spawnRobot(coordinates[i], i);//循环调用spawnrobot函数生成机器人
  }
  ros::spin();
  return 0;
}

