#include "central_map.h"
#include <thread>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_map");

    ros::NodeHandle nh;
    // RandomCoordinates(7, 7, 0.5, "/home/haichao/demo_ws/src/demo01_gazebo/coordinate/start_coordinates.csv");
    // RandomCoordinates(7, 7, 0.5, "/home/haichao/demo_ws/src/demo01_gazebo/coordinate/target_coordinates.csv");
    central_map *map = new central_map();
    map->module_divition();
    std::thread t([&](){ map->upload_realtime_coos_cluster(); });
    map->dilation();
    ROS_INFO("finish1");
    if (t.joinable())
    {
        t.join();
    }
    return 0;
}