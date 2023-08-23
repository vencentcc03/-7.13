#include <iostream>
#include <fstream>
#include <vector>

void storeCoordinatesToFile(const std::vector<std::vector<double>>& coordinates, const std::string& filename) {
  std::ofstream file(filename, std::ios::trunc);  // 使用 std::ios::trunc 模式打开文件

  if (file.is_open()) {
    for (const auto& point : coordinates) {
      file << point[0] << "," << point[1] << std::endl;
    }
    file.close();
    std::cout << "Coordinates stored to file: " << filename << std::endl;
  } else {
    std::cout << "Failed to open file: " << filename << std::endl;
  }
}

int main() {
  std::vector<std::vector<double>> coordinates = {
    {1.0, 2.0},
    {3.0, 4.0},
    {5.0, 6.0},
    // 添加剩下的坐标...
  };

  std::string filePath = "src/demo01_gazebo/coordinate/start_coordinates.csv";  // 指定文件的完整路径
  storeCoordinatesToFile(coordinates, filePath);

  return 0;
}
