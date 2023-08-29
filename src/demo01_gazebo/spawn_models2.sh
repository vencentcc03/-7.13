#!/bin/bash

# 设置循环生成节点的数量
numNodes=10
# 创建launch文件头部
launchFile="/home/haichao/demo_ws/src/demo01_gazebo/launch/03demoshell.launch"
echo "<launch>" > "$launchFile"

# 循环生成节点
for ((i=1; i<=numNodes; i++))
do
    for ((j=1; j<=numNodes; j++))
    # 生成节点参数
    do
    model_name="mycar${i}${j}"
    x_pos=$i
    y_pos=$j

    # 创建节点
    node="<node pkg=\"gazebo_ros\" type=\"spawn_model\" name=\"model${i}${j}\" args=\"-urdf -model ${model_name} -param robot_description -x ${x_pos} -y ${y_pos} -z 0\"/>"

    # 将节点信息添加到launch文件
    echo $node >> "$launchFile"
    done
done

# 创建launch文件尾部
echo "</launch>" >> "$launchFile"
echo "Generated launch file: "$launchFile""
