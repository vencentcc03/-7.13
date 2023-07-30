#include "central_map.h"
#include <nav_msgs/Odometry.h>
#include <thread>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_map");

    ros::NodeHandle nh;
    // RandomCoordinates(7, 7, 0.5, "/home/haichao/demo_ws/src/demo01_gazebo/coordinate/start_coordinates.csv");
    // RandomCoordinates(7, 7, 0.5, "/home/haichao/demo_ws/src/demo01_gazebo/coordinate/target_coordinates.csv");
    central_map *map = new central_map();
    map->module_divition();
    ROS_INFO("finish divition");

    std::thread t([&]()
                  { map->upload_realtime_coos(); });
    map->generate_pubs();
    ros::Duration(10).sleep();
    // // for (int i=0;i<10;i++){
    // for (auto &c : map->clusters)
    // {
    //     c.printValues();
    // }

    // ros::Duration(0.3).sleep();
    // }
    map->dilation();
    map->clusters[0].printValues();
    ros::Duration(1).sleep();
    // map->incrementNextTarget();
    // map->move_next();
    map->set_boundary();
    // printDoubleVector(map->coordinates);
    std::vector<std::pair<int, int>> pairVector;
    double x = map->clusters[0].now_point[0][0];
    double y = map->clusters[0].now_point[0][1];
    int dx[] = {-1, 0, 1, 0}; // 下右上左
    int dy[] = {0, 1, 0, -1};
    Node tar(map->clusters[0].tar_point[0][0], map->clusters[0].tar_point[0][1]);
    tar.node_print();
    while (!(map->clusters[0].now_point[0] == map->clusters[0].tar_point[0]))
    {
        pairVector.clear();
        for (int i = 0; i < 4; i++)
        {

            Node current(x + dx[i], y + dy[i]);
            current.h=heuristic(current,tar);
            current.f=current.h;
            current.node_print();
            std::pair<int, int> a = std::make_pair(i, map->a_star(current, tar, map->clusters[0].type));
            pairVector.push_back(a);
        }
        int minSecondParam = pairVector[0].second;
        int minFirstParam = pairVector[0].first;
        ROS_INFO("i===%d",minFirstParam);
        for (const auto &p : pairVector)
        {
            if (p.second < minSecondParam)
            {
                minSecondParam = p.second;
                minFirstParam = p.first;
            }
        }
        map->clusters[0].next_target[0][0] = map->clusters[0].now_point[0][0]+dx[minFirstParam];
        map->clusters[0].next_target[0][1] = map->clusters[0].now_point[0][1] + dy[minFirstParam];
        ros::Duration(1).sleep();
        map->move_next();
    }
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
// // // 循环等待消息到来