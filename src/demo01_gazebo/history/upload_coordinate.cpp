#include <iostream>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include "readfile.h"  // 包含新创建的头文件

int main(int argc, char** argv) {
  ros::init(argc, argv, "upload_coordinates_node");
  ros::NodeHandle nh;

  std::string filePath = "src/demo01_gazebo/coordinate/start_coordinates.csv";
  std::vector<std::vector<double>> coordinates;

  read_file(filePath, coordinates);  // 调用新的函数，将坐标存储在coordinates中

  // 将coordinates转换为XmlRpcValue对象
  XmlRpc::XmlRpcValue xmlRpcCoordinates;
  xmlRpcCoordinates.setSize(coordinates.size());

  for (size_t i = 0; i < coordinates.size(); ++i) {
    XmlRpc::XmlRpcValue xmlRpcPoint;
    xmlRpcPoint.setSize(2);
    xmlRpcPoint[0] = coordinates[i][0];
    xmlRpcPoint[1] = coordinates[i][1];
    xmlRpcCoordinates[i] = xmlRpcPoint;
  }

  // 将XmlRpcValue对象上传到参数服务器
  nh.setParam("/coordinates", xmlRpcCoordinates);

  std::cout << "Coordinates uploaded to parameter server with key: " << "/coordinates" << std::endl;

  ros::spin();

  return 0;
}
