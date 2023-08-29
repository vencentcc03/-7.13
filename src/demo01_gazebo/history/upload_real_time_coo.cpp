#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <XmlRpcValue.h>
// 定义全局vector来存储坐标
std::vector<std::vector<double>> coordinates;

// 回调函数处理接收到的消息
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg, int topic_index)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // 将坐标存储在vector中
    coordinates[topic_index][0] = x;
    coordinates[topic_index][1] = y;
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
  ros::param::set("/coordinates", xmlRpcCoordinates);

  std::cout << "Coordinates uploaded to parameter server with key: " << "/coordinates" << std::endl;
  

}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_subscriber");
    ros::NodeHandle nh;

    // 创建订阅者和vector
    std::vector<ros::Subscriber> subscribers;
    coordinates.resize(10, std::vector<double>(2, 0.0));

    for (int i = 0; i < 10; ++i)
    {
        std::string topic_name = "base_pose_ground_truth_" + std::to_string(i + 1);
        ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(topic_name, 1,
            boost::bind(&odometryCallback, _1, i));
        subscribers.push_back(sub);
    }


    // 进入ROS循环
    ros::spin();

    return 0;
}
