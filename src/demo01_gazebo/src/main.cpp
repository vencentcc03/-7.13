#include "central_map.h"
#include <nav_msgs/Odometry.h>
#include <thread>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_map");

    ros::NodeHandle nh;
    // RandomCoordinates(7, 7, 0.5, "/home/haichao/demo_ws/src/demo01_gazebo/coordinate/start_coordinates.csv");
    // RandomCoordinates(7, 7, 0.5, "/home/haichao/demo_ws/src/demo01_gazebo/coordinate/target_coordinates.csv");
    central_map *map = new central_map(); // 创建对象，构造函数已经传入初始坐标与目标坐标，并将机器人加载入gazebo
    map->module_divition();               // 模块划分算法
    ROS_INFO("finish divition");

    std::thread t([&]()
                  { map->upload_realtime_coos(); }); // 将获取实时坐标加入新线程
    map->generate_pubs();                            // 部署速度信息发布器
    ros::Duration(10).sleep();                       // 等待足够的时间，保证所有发布器部署完成
    // // for (int i=0;i<10;i++){
    // for (auto &c : map->clusters)
    // {
    //     c.printValues();
    // }
    // ros::Duration(0.3).sleep();
    // }
    map->dilation(); // 扩散
    map->clusters[0].printValues();
    ros::Duration(1).sleep();
    // map->incrementNextTarget();
    // map->move_next();
    map->set_boundary(); // 设置边界大小
    ROS_INFO("mapsize::%f,%f,%f,%f", map->mapsize_maxx, map->mapsize_maxy, map->mapsize_minx, map->mapsize_miny);
    ROS_INFO("tarcenter::%d,%d", map->centerPoint_target[0], map->centerPoint_target[1]);
    ROS_INFO("stacenter::%d,%d", map->centerPoint[0], map->centerPoint[1]);
    ROS_INFO("target2:::");
    // printDoubleVector(map->target_coordinates_2);

    // printDoubleVector(map->coordinates);
    map->a_star_astep();
    ROS_INFO("33333333");
    if (t.joinable())
    {
        t.join();
    }
}
// ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/car1/cmd_vel", 1);

// // 创建一个Twist消息，并设置x轴线速度为1，y轴和z轴线速度为0
// geometry_msgs::Twist cmd_msg;
// cmd_msg.linear.x = 0.1;
// cmd_msg.linear.y = 0.0;
// cmd_msg.linear.z = 0.0;
// cmd_msg.angular.x = 0.0;
// cmd_msg.angular.y = 0.0;
// cmd_msg.angular.z = 0.0;

// // // 等待一小段时间，让发布者和订阅者之间建立连接
// ros::Duration(1.0).sleep();

// // 通过引用传递x_value给回调函数

// // int previous_i = -1;
// // double cur_x = -1.0;
// // ros::Subscriber odom_sub = nh.subscribe(
// //     "/base_pose_ground_truth_1", 1, odomCallback);
// // // 发送50次控制消息，使机器人x轴线速度为1

// cmd_pub.publish(cmd_msg);
// ros::Duration(0.01).sleep(); // 0.1秒的时间间隔
// cmd_pub.publish(cmd_msg);
//     // ros::Subscriber odom_sub = nh.subscribe("/base_pose_ground_truth_1", 1, [&](const nav_msgs::Odometry::ConstPtr &msg)
//     //                                         {
//     //                                             // 在这里进行对接收到的信息的处理
//     //                                             // 例如，比较i值是否发生变化
//     //                                             cur_x = msg->pose.pose.position.x; });
//     ros::spinOnce();
// // // 循环等待消