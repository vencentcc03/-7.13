#include "central_map.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <XmlRpcValue.h>
#include <ros/console.h>
#include "instrument.h"
// 读取初始坐标
void central_map::upload_startcoo()
{
  std::string filePath = "/home/haichao/demo_ws/src/demo01_gazebo/coordinate/start_coordinates.csv";
  read_file(filePath, start_coordinates); // 读取文件，将初始坐标存储在start_coordinates中
}
void central_map::upload_target_coo()
{
  std::string filePath = "/home/haichao/demo_ws/src/demo01_gazebo/coordinate/target_coordinates.csv";
  read_file(filePath, target_coordinates); // 读取文件，将初始坐标存储在target_coordinates中
}
// 获取实时坐标，存入cluster_now_point中
void central_map::upload_realtime_coos_cluster()
{

  for (int i = 0; i < clusters.size(); ++i)
  {
    std::string topic_name = "base_pose_ground_truth_" + std::to_string(clusters[i].index);
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(topic_name, 1,
                                                           boost::bind(&central_map::odometryCallback, this, _1, i));
    subscribers.push_back(sub);
  }

  ros::Rate rate(100); // 设置循环的频率为10Hz

  while (ros::ok())
  {
    ros::spinOnce(); // 处理回调函数

    rate.sleep(); // 控制循环的频率
  }
}
// 获取坐标信息的回调函数
void central_map::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, int cluster_index)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // 将坐标存储在vector中
  clusters[cluster_index].now_point[0] = x;
  clusters[cluster_index].now_point[1] = y;
  // for (const auto &coordinates1 : coordinates)
  // {
  //   for (const auto &coordinate : coordinates1)
  //   {
  //     ROS_INFO("%.2f ", coordinate);
  //   }
  //   ROS_INFO("\n");
  // }
}
// 将机器人模型加载入gazebo
void central_map::spawnRobot()
{
  // 生成机器人
  for (int i = 0; i < start_coordinates.size(); ++i)
  {
    ROS_INFO("i=%d", i);
    ROS_INFO("size=%d", start_coordinates.size());
    spawnRobots(start_coordinates[i], i);
  }
  ROS_INFO("spawn_model");
}
// 将模型加载进gazebo
void central_map::spawnRobots(const std::vector<double> &coordinate, int robotIndex)
{ // 传入参数：存储坐标的向量，序号
  // 依次打开第一个1到第i个urdf文件
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
  // 将坐标信息存储到gazebo_msg的消息类型中
  gazebo_msgs::SpawnModel srv;
  srv.request.model_xml = modelXml;
  srv.request.model_name = "robot_" + std::to_string(robotIndex);
  srv.request.robot_namespace = "/";
  srv.request.initial_pose.position.x = coordinate[0];
  srv.request.initial_pose.position.y = coordinate[1];
  srv.request.reference_frame = "world";
  // 向gazebo中加载urdf模型
  ros::service::waitForService("/gazebo/spawn_urdf_model");
  ros::ServiceClient spawnClient = ros::NodeHandle().serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

  if (spawnClient.call(srv))
  {
    ROS_INFO("Successfully spawned robot_%d", robotIndex);
  }
  else
  {
    ROS_ERROR("Failed to spawn robot_%d", robotIndex);
  }
}
// 寻找最近点
int central_map::find_nearest_point(const std::vector<std::vector<double>> &start_coo, const std::vector<std::vector<double>> &target_coo)
{
  int randomX = rand() % 11;
  int randomY = rand() % 11;
  int flag = 0;

  // 初始化变量以存储最近距离和点坐标
  double closestDistance1 = std::numeric_limits<double>::max();
  double closestDistance2 = std::numeric_limits<double>::max();

  // 遍历起始坐标点和目标坐标点，找到最近的点
  for (const auto &startPoint : start_coo)
  {
    double distance = sqrt(pow(randomX - startPoint[0], 2) + pow(randomY - startPoint[1], 2));
    // 如果距离小于迄今找到的最近距离，更新最近的点
    if (distance < closestDistance1)
    {
      closestDistance1 = distance;
      closestPoint1 = startPoint;
    }
  }
  for (const auto &targetPoint : target_coo)
  {
    double distance = sqrt(pow(randomX - targetPoint[0], 2) + pow(randomY - targetPoint[1], 2));
    // 如果距离小于迄今找到的最近距离，更新最近的点
    if (distance < closestDistance2)
    {
      closestDistance2 = distance;
      closestPoint2 = targetPoint;
    }
  }
  return flag;
}

// 模块划分
void central_map::module_divition()
{
  std::vector<std::vector<double>> start_coo = start_coordinates; // 用于存储初始坐标和最终坐标
  std::vector<std::vector<double>> target_coo = target_coordinates;
  std::vector<std::vector<double>> sta;
  std::vector<std::vector<double>> tar;
  // printDoubleVector(start_coordinates);
  // printDoubleVector(target_coordinates);
  // printDoubleVector(start_coo);
  // printDoubleVector(target_coo);

  while (start_coo.size() && target_coo.size())
  {
    find_nearest_point(start_coo, target_coo);

    // printVector(closestPoint1);
    // printVector(closestPoint2);

    for (const auto &startPoint : start_coo)
    {
      double distance = sqrt(pow(closestPoint1[0] - startPoint[0], 2) + pow(closestPoint1[1] - startPoint[1], 2));
      if (distance <= search_radius)
      {
        sta.push_back(startPoint);
      }
    }
    for (const auto &targetPoint : target_coo)
    { // 将搜索半径范围内的点装进sta和tar中
      double distance = sqrt(pow(closestPoint2[0] - targetPoint[0], 2) + pow(closestPoint2[1] - targetPoint[1], 2));
      if (distance <= search_radius)
      {
        tar.push_back(targetPoint);
      }
    }

  break_label:
    //   ROS_INFO("hello");
    // printDoubleVector(sta);
    // printDoubleVector(tar);
    // printDoubleVector(start_coo);
    // printDoubleVector(target_coo);
    // ROS_INFO("sta.size:%d",sta.size());
    // ROS_INFO("tar.size:%d",tar.size());
    // ROS_INFO("start_coo.size:%d",start_coo.size());
    // ROS_INFO("target_coo.size:%d",target_coo.size());
    //   for (const auto& g : clusters) {
    // ROS_INFO("Type: %d", g.type);
    // ROS_INFO("Now Point: %f, %f", g.now_point[0], g.now_point[1]);
    // ROS_INFO("target Point: %f, %f", g.tar_point[0], g.tar_point[1]);
    //   }
    // ros::Duration(1.0).sleep();
    while (sta.size() && tar.size())
    { // 遍历sta和tar如果满足条件就将满足条件的点从sta、tar、start_coo、target_coo中剔除，并生成相应类型的cluster存储进vector<cluster>中

      for (const auto &startPoint1 : sta)
      {
        std::vector<double> startPoint = startPoint1;
        if (isCoordinateExists(sta, startPoint[0] + 1, startPoint[1]) && isCoordinateExists(sta, startPoint[0] + 1, startPoint[1] + 1) && isCoordinateExists(sta, startPoint[0], startPoint[1] + 1))
        {
          for (const auto &targetPoint1 : tar)
          {
            std::vector<double> targetPoint = targetPoint1;
            if (isCoordinateExists(tar, targetPoint[0] + 1, targetPoint[1]) && isCoordinateExists(tar, targetPoint[0] + 1, targetPoint[1] + 1) && isCoordinateExists(tar, targetPoint[0], targetPoint[1] + 1))
            {
              cluster cluster1;

              int i = findCoordIndex(start_coordinates, startPoint[0], startPoint[1]);
              cluster1.index = i;
              std::string i_str = std::to_string(i);
              cluster1.motor_topic1 = "/cari/cmd_vel";
              cluster1.motor_topic1.replace(4, 1, i_str);
              i = findCoordIndex(start_coordinates, startPoint[0] + 1, startPoint[1]);
              i_str = std::to_string(i);
              cluster1.motor_topic2 = "/cari/cmd_vel";
              cluster1.motor_topic2.replace(4, 1, i_str);
              i = findCoordIndex(start_coordinates, startPoint[0] + 1, startPoint[1] + 1);
              i_str = std::to_string(i);
              cluster1.motor_topic3 = "/cari/cmd_vel";
              cluster1.motor_topic3.replace(4, 1, i_str);
              i = findCoordIndex(start_coordinates, startPoint[0], startPoint[1] + 1);
              i_str = std::to_string(i);
              cluster1.motor_topic4 = "/cari/cmd_vel";
              cluster1.motor_topic4.replace(4, 1, i_str);

              cluster1.type = 4; // 四个机器人
              cluster1.sta_point = startPoint;
              cluster1.tar_point = targetPoint;
              cluster1.now_point = startPoint;
              cluster1.vel_x = 0;
              cluster1.vel_y = 0;
              clusters.push_back(cluster1);
              removeCoordinate(sta, startPoint[0] + 1, startPoint[1]);
              removeCoordinate(sta, startPoint[0] + 1, startPoint[1] + 1);
              removeCoordinate(sta, startPoint[0], startPoint[1] + 1);
              removeCoordinate(sta, startPoint[0], startPoint[1]);
              removeCoordinate(tar, targetPoint[0] + 1, targetPoint[1]);
              removeCoordinate(tar, targetPoint[0] + 1, targetPoint[1] + 1);
              removeCoordinate(tar, targetPoint[0], targetPoint[1]);
              removeCoordinate(tar, targetPoint[0], targetPoint[1] + 1);
              removeCoordinate(start_coo, startPoint[0] + 1, startPoint[1]);
              removeCoordinate(start_coo, startPoint[0] + 1, startPoint[1] + 1);
              removeCoordinate(start_coo, startPoint[0], startPoint[1] + 1);
              removeCoordinate(start_coo, startPoint[0], startPoint[1]);
              removeCoordinate(target_coo, targetPoint[0] + 1, targetPoint[1]);
              removeCoordinate(target_coo, targetPoint[0] + 1, targetPoint[1] + 1);
              removeCoordinate(target_coo, targetPoint[0], targetPoint[1]);
              removeCoordinate(target_coo, targetPoint[0], targetPoint[1] + 1);
              goto break_label;
            }
          }
        }
        else if (isCoordinateExists(sta, startPoint[0], startPoint[1] + 1))
        {
          for (const auto &targetPoint1 : tar)
          {
            std::vector<double> targetPoint = targetPoint1;
            if (isCoordinateExists(tar, targetPoint[0], targetPoint[1] + 1))
            {
              cluster cluster1;
              int i = findCoordIndex(start_coordinates, startPoint[0], startPoint[1]);
              cluster1.index = i;
              std::string i_str = std::to_string(i);
              cluster1.motor_topic1 = "/cari/cmd_vel";
              cluster1.motor_topic1.replace(4, 1, i_str);
              i = findCoordIndex(start_coordinates, startPoint[0], startPoint[1] + 1);
              i_str = std::to_string(i);
              cluster1.motor_topic2 = "/cari/cmd_vel";
              cluster1.motor_topic2.replace(4, 1, i_str);

              cluster1.type = 2; // 横着
              cluster1.sta_point = startPoint;
              cluster1.tar_point = targetPoint;
              cluster1.now_point = startPoint;
              cluster1.vel_x = 0;
              cluster1.vel_y = 0;
              clusters.push_back(cluster1);

              removeCoordinate(sta, startPoint[0], startPoint[1] + 1);
              removeCoordinate(sta, startPoint[0], startPoint[1]);
              removeCoordinate(tar, targetPoint[0], targetPoint[1] + 1);
              removeCoordinate(tar, targetPoint[0], targetPoint[1]);

              removeCoordinate(start_coo, startPoint[0], startPoint[1] + 1);
              removeCoordinate(start_coo, startPoint[0], startPoint[1]);
              removeCoordinate(target_coo, targetPoint[0], targetPoint[1] + 1);
              removeCoordinate(target_coo, targetPoint[0], targetPoint[1]);
              goto break_label;
            }
          }
        }
        else if (isCoordinateExists(sta, startPoint[0] + 1, startPoint[1]))
        {
          for (const auto &targetPoint1 : tar)
          {
            std::vector<double> targetPoint = targetPoint1;
            if (isCoordinateExists(tar, targetPoint[0] + 1, targetPoint[1]))
            {
              cluster cluster1;
              int i = findCoordIndex(start_coordinates, startPoint[0], startPoint[1]);
              cluster1.index = i;
              std::string i_str = std::to_string(i);
              cluster1.motor_topic1 = "/cari/cmd_vel";
              cluster1.motor_topic1.replace(4, 1, i_str);
              i = findCoordIndex(start_coordinates, startPoint[0] + 1, startPoint[1]);
              i_str = std::to_string(i);
              cluster1.motor_topic2 = "/cari/cmd_vel";
              cluster1.motor_topic2.replace(4, 1, i_str);
              cluster1.type = 3; // 竖着
              cluster1.sta_point = startPoint;
              cluster1.tar_point = targetPoint;
              cluster1.now_point = startPoint;
              cluster1.vel_x = 0;
              cluster1.vel_y = 0;
              clusters.push_back(cluster1);
              // printVector(startPoint);
              removeCoordinate(sta, startPoint[0] + 1, startPoint[1]);
              removeCoordinate(sta, startPoint[0], startPoint[1]);
              removeCoordinate(tar, targetPoint[0] + 1, targetPoint[1]);
              removeCoordinate(tar, targetPoint[0], targetPoint[1]);
              // printVector(startPoint);
              removeCoordinate(start_coo, startPoint[0] + 1, startPoint[1]);
              removeCoordinate(start_coo, startPoint[0], startPoint[1]);
              removeCoordinate(target_coo, targetPoint[0] + 1, targetPoint[1]);
              removeCoordinate(target_coo, targetPoint[0], targetPoint[1]);
              goto break_label;
            }
          }
        }
        else
        {

          cluster cluster1;
          int i = findCoordIndex(start_coordinates, startPoint[0], startPoint[1]);
          cluster1.index = i;
          std::string i_str = std::to_string(i);
          cluster1.motor_topic1 = "/cari/cmd_vel";
          cluster1.motor_topic1.replace(4, 1, i_str);

          cluster1.type = 1; // 单个
          cluster1.sta_point = startPoint;
          cluster1.tar_point = tar[0];
          cluster1.now_point = startPoint;
          cluster1.vel_x = 0;
          cluster1.vel_y = 0;
          clusters.push_back(cluster1);
          std::vector<double> tar1 = tar[0];
          removeCoordinate(sta, startPoint[0], startPoint[1]);
          removeCoordinate(tar, tar1[0], tar1[1]);
          removeCoordinate(start_coo, startPoint[0], startPoint[1]);
          removeCoordinate(target_coo, tar1[0], tar1[1]);
          goto break_label;
        }
      }
    }
  }
}
// 寻找中心点
void central_map::find_centerpoint()
{
  double minX = 100000, maxX = 1, minY = 10000, maxY = 0;
  for (const auto &point : start_coordinates)
  {
    minX = std::min(minX, point[0]);
    maxX = std::max(maxX, point[0]);

    minY = std::min(minY, point[1]);
    maxY = std::max(maxY, point[1]);
  }
  // ROS_INFO("%f,%f,%f,%f",minX,minY,maxX,maxY);
  double centerX = (minX + maxX) / 2.0;
  double centerY = (minY + maxY) / 2.0;
  centerPoint = {centerX, centerY};
}
void central_map::dilation()
{
  find_centerpoint();
  for (auto &c : clusters)
  {

    // 计算当前起点和中心点的偏移
    double xDiff = c.sta_point[0] - centerPoint[0];
    double yDiff = c.sta_point[1] - centerPoint[1];

    // 进行扩散变换
    double xTrans = centerPoint[0] + xDiff * dilation_coe;
    double yTrans = centerPoint[1] + yDiff * dilation_coe;

    // 生成变换后的坐标
    std::vector<double> transPoint = {xTrans, yTrans};

    // 保存到目标向量
    c.next_target = transPoint;
  }

  // ROS_INFO("in the while()");
  // for (const auto &c : clusters)
  // {

  //   ROS_INFO("Type: %d", c.type);
  //   ROS_INFO("Topic 1: %s", c.motor_topic1.c_str());
  //   ROS_INFO("Topic 2: %s", c.motor_topic2.c_str());
  //   ROS_INFO("Topic 3: %s", c.motor_topic3.c_str());
  //   ROS_INFO("Topic 4: %s", c.motor_topic4.c_str());
  // }
  int match_num = 0;
  std::vector<ros::Publisher> pubs;
  std::vector<geometry_msgs::Twist> msgs;
  ros::Rate rate(500);
  for (auto &c : clusters)
  {
    if (c.type == 4 && c.nto_next_target)
    {

      // 创建用于motor话题的发布器
      ros::Publisher motor1_pub = nh.advertise<geometry_msgs::Twist>(c.motor_topic1, 1);
      ros::Publisher motor2_pub = nh.advertise<geometry_msgs::Twist>(c.motor_topic2, 1);
      ros::Publisher motor3_pub = nh.advertise<geometry_msgs::Twist>(c.motor_topic3, 1);
      ros::Publisher motor4_pub = nh.advertise<geometry_msgs::Twist>(c.motor_topic4, 1);
      pubs.push_back(motor1_pub);
      pubs.push_back(motor2_pub);
      pubs.push_back(motor3_pub);
      pubs.push_back(motor4_pub);
    }
    if ((c.type == 2 || c.type == 3) && c.nto_next_target)
    {
      // 创建用于motor话题的发布器
      ros::Publisher motor1_pub = nh.advertise<geometry_msgs::Twist>(c.motor_topic1, 1);
      ros::Publisher motor2_pub = nh.advertise<geometry_msgs::Twist>(c.motor_topic2, 1);
      pubs.push_back(motor1_pub);
      pubs.push_back(motor2_pub);
    }
    if (c.type == 1 && c.nto_next_target)
    {
      // 创建用于motor话题的发布器
      ros::Publisher motor1_pub = nh.advertise<geometry_msgs::Twist>(c.motor_topic1, 1);
      pubs.push_back(motor1_pub);
    }
  }
  
  ROS_INFO("pubs.size:%d",pubs.size());
  while (match_num < clusters.size())
  {
    match_num=0;
    msgs.clear();
    ROS_INFO("now_points::::::::");
    for (auto &c : clusters){
      printVector(c.now_point);
    }
      for (auto &c : clusters)
      {
        if (abs(c.now_point[0] -c.next_target[0])<0.01)
        {
          c.vel_x = 0;
          c.vel_msg.linear.x = c.vel_x;
        }

        else
        {
          if (c.now_point[0] < c.next_target[0])
            c.vel_x = 0.1;
          if (c.now_point[0] > c.next_target[0])
            c.vel_x = -0.1;
          c.vel_msg.linear.x = c.vel_x;
        }
        if (abs(c.now_point[1] - c.next_target[1]) < 0.01)
        {
          c.vel_y = 0;
          c.vel_msg.linear.y = c.vel_y;
        }
        else
        {
          if (c.now_point[1] < c.next_target[1])
            c.vel_y = 0.1;
          if (c.now_point[1] > c.next_target[1])
            c.vel_y = -0.1;
          c.vel_msg.linear.y = c.vel_y;
        }
        if (c.vel_x == 0&& c.vel_y == 0)
        {
          // printVector(c.now_point);
          // printVector(c.next_target);yt
          match_num++;
        }

        switch (c.type)
        {
        case 1:
          msgs.push_back(c.vel_msg);
          break;
        case 2:
          msgs.push_back(c.vel_msg);
          msgs.push_back(c.vel_msg);
          break;
        case 3:
          msgs.push_back(c.vel_msg);
          msgs.push_back(c.vel_msg);
          break;
        case 4:
          msgs.push_back(c.vel_msg);
          msgs.push_back(c.vel_msg);
          msgs.push_back(c.vel_msg);
          msgs.push_back(c.vel_msg);
          break;
        }
      }
    // ROS_INFO("msgs.size:%d",msgs.size());
    // ROS_INFO("match_num:%d,%d",match_num,clusters.size());
    for (int i = 0; i < pubs.size(); ++i)
    {
      pubs[i].publish(msgs[i]);
    }
  }
  rate.sleep();
}
