
#include <iostream>
#include "instrument.h"
#include <algorithm>
#include <ros/ros.h>

// 打开文件以读取坐标,传入参数：文件地址和一个向量，将读取到的坐标储存进向量中
void read_file(const std::string &filepath, std::vector<std::vector<double>> &coordinates)
{
  std::ifstream file(filepath); // 打开文件以读取坐标

  if (file.is_open())
  {
    std::string line;

    while (std::getline(file, line))
    {
      std::istringstream iss(line);
      std::string token;
      std::vector<double> point;

      while (std::getline(iss, token, ','))
      {
        point.push_back(std::stod(token));
      }

      coordinates.push_back(point);
    }

    file.close();
  }
  else
  {
    std::cout << "Failed to open file: " << filepath << std::endl;
  }
}
// 检测坐标点是否在向量内
bool isCoordinateExists(const std::vector<std::vector<double>> &coordinates, double x, double y)
{
  for (const auto &point : coordinates)
  {
    if (point[0] == x && point[1] == y)
    {
      return true; // 找到匹配的坐标
    }
  }
  return false; // 没有找到匹配的坐标
}
// 将指定坐标从向量中剔除
void removeCoordinate(std::vector<std::vector<double>> &coordinates, double x, double y)
{
  coordinates.erase(
      std::remove_if(coordinates.begin(), coordinates.end(),
                     [x, y](const std::vector<double> &point)
                     {
                       return point[0] == x && point[1] == y;
                     }),
      coordinates.end());
}
// 打印double-vector
void printDoubleVector(const std::vector<std::vector<double>> &data)

{

  ROS_INFO("Printing double vector data:");

  for (const auto &innerVector : data)

  {

    std::string vectorStr = "[";

    for (const auto &value : innerVector)

    {

      vectorStr += std::to_string(value) + ", ";
    }

    vectorStr = vectorStr.substr(0, vectorStr.length() - 2); // Remove the trailing comma and space

    vectorStr += "]";

    ROS_INFO_STREAM(vectorStr);
  }
}
// 打印vector
void printVector(const std::vector<double> &data)
{
  std::cout << "Printing vector data:" << std::endl;

  for (const auto &value : data)
  {
    std::cout << value << ", ";
  }
  std::cout << std::endl;
}
// 传入参数是存有多个坐标的vector和一个坐标，返回该坐标在这个向量中的序数
int findCoordIndex(const std::vector<std::vector<double>> &coords, double x, double y)
{

  for (int i = 0; i < coords.size(); i++)
  {

    if (coords[i][0] == x && coords[i][1] == y)
    {
      return ++i;
    }
  }

  return -1;
}