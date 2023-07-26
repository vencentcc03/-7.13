#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

void generateURDFFiles()
{
    std::string templateURDF = R"(
<!-- 
    创建一个机器人模型(盒状即可)，显示在 Gazebo 中 
-->

<robot name="mycar_{number}">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.9 0.9 0.2" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
            <material name="yellow">
                <color rgba="0.5 0.3 0.0 1" />
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.01 0.01 0.002" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
        </collision>
        <inertial>
            <origin xyz="0 0 0" />
            <mass value="100" />
            
            <inertia ixx="10" ixy="0" ixz="0" iyy="10" iyz="0" izz="10" />
        </inertial>
    </link>
    <gazebo reference="base_link">
        <material>Gazebo/Black</material>
    </gazebo>
    <gazebo>
    <plugin name="mobile_base" filename="libgazebo_ros_planar_move.so">
      <commandTopic>car{number}/cmd_vel</commandTopic>
      <odometryTopic>car{number}/odom</odometryTopic>
      <odometryFrame>world</odometryFrame>
        <robotBaseFrame>my_base_link</robotBaseFrame>
  <odometryRate>1</odometryRate>
  <cmdTimeout>-1</cmdTimeout>
    </plugin>
  </gazebo>
    <gazebo>
        <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>100</updateRate>
            <bodyName>base_link</bodyName>
            <topicName>base_pose_ground_truth_{number}</topicName>
            <gaussianNoise>0.0</gaussianNoise>
            <frameName>map</frameName>
            <xyzOffsets>0 0 0</xyzOffsets>
            <rpyOffsets>0 0 0</rpyOffsets>
        </plugin>
    </gazebo>
</robot>
)";

    std::string outputDirectory = "/home/haichao/demo_ws/src/demo01_gazebo/urdf_create";

    if (system(("mkdir -p " + outputDirectory).c_str()) != 0)
    {
        std::cerr << "Failed to create output directory." << std::endl;
        return;
    }

    for (int number = 1; number <= 100; ++number)
    {
        std::string urdfContent = templateURDF;
        size_t pos = urdfContent.find("{number}");
        while (pos != std::string::npos)
        {
            urdfContent.replace(pos, 8, std::to_string(number));
            pos = urdfContent.find("{number}", pos + 1);
        }

        std::string filename = outputDirectory + "/mycar_" + std::to_string(number) + ".urdf";
        std::ofstream file(filename);
        if (file)
        {
            file << urdfContent;
            file.close();
            std::cout << "Generated URDF file: " << filename << std::endl;
        }
        else
        {
            std::cerr << "Failed to create URDF file: " << filename << std::endl;
        }
    }
}

int main()
{
    generateURDFFiles();
    return 0;
}
