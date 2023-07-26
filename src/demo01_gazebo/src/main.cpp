#include "central_map.h"
#include <nav_msgs/Odometry.h>
#include <thread>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_map");

    ros::NodeHandle nh;

    central_map *map = new central_map();
    map->module_divition();
    ROS_INFO("finish divition");
    
        // for (const auto &g : map->clusters)
        // {
        //     ROS_INFO("Type: %d", g.type);
        //     ROS_INFO("Now Point: %f, %f", g.now_point[0], g.now_point[1]);
        //     ROS_INFO("target Point: %f, %f", g.tar_point[0], g.tar_point[1]);
        // }
        std::thread t([&]()
                      { map->upload_realtime_coos(); });
        // for (int i=0;i<10;i++){
        //     for (auto &c : map->clusters)
        // {
        //   c.printValues();
        // }
        // ros::Duration(0.3).sleep();
        // }
        map->dilation();
        ROS_INFO("finish1");
        if (t.joinable())
        {
            t.join();
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
    // // // 循环等待消息到来


    // // return 0;
    return 0;
}