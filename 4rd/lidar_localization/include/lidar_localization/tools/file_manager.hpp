/*
 * @Description: 读写文件管理
 * @Author: Ren Qian
 * @Date: 2020-02-24 19:22:53
 */
#ifndef LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_
#define LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace lidar_localization {
class FileManager {
public:
  static bool CreateFile(std::ofstream &ofs, std::string file_path);
  static bool InitDirectory(std::string directory_path, std::string use_for);
  static bool CreateDirectory(std::string directory_path, std::string use_for);
  static bool CreateDirectory(std::string directory_path);
  static bool ReadFile(std::ifstream &ifs, std::string file_path);
  static std::vector<std::string> split(const std::string &str,
                                        const std::string &delim);
};
} // namespace lidar_localization

#endif
