/*
 * @Description: 一些文件读写的方法
 * @Author: Ren Qian
 * @Date: 2020-02-24 20:09:32
 */
#include "lidar_localization/tools/file_manager.hpp"

#include "glog/logging.h"
#include <boost/filesystem.hpp>

namespace lidar_localization {
bool FileManager::CreateFile(std::ofstream &ofs, std::string file_path) {
  ofs.close();
  boost::filesystem::remove(file_path.c_str());

  ofs.open(file_path.c_str(), std::ios::out);
  if (!ofs) {
    LOG(WARNING) << "无法生成文件: " << std::endl
                 << file_path << std::endl
                 << std::endl;
    return false;
  }

  return true;
}

bool FileManager::ReadFile(std::ifstream &ifs, std::string file_path) {
  ifs.open(file_path.c_str(), std::ios::in);
  if (!ifs) {
    LOG(WARNING) << "无法打开文件: " << std::endl
                 << file_path << std::endl
                 << std::endl;
    return false;
  }

  return true;
}

bool FileManager::InitDirectory(std::string directory_path,
                                std::string use_for) {
  if (boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::remove_all(directory_path);
  }

  return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path,
                                  std::string use_for) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    LOG(WARNING) << "CANNOT create directory " << std::endl
                 << directory_path << std::endl
                 << std::endl;
    return false;
  }

  std::cout << use_for << " output path:" << std::endl
            << directory_path << std::endl
            << std::endl;
  return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    LOG(WARNING) << "CANNOT create directory " << std::endl
                 << directory_path << std::endl
                 << std::endl;
    return false;
  }

  return true;
}

std::vector<std::string> FileManager::split(const std::string &str,
                                            const std::string &delim) {
  std::vector<std::string> res;
  if ("" == str)
    return res;
  //先将要切割的字符串从string类型转换为char*类型
  char *strs = new char[str.length() + 1]; //不要忘了
  strcpy(strs, str.c_str());

  char *d = new char[delim.length() + 1];
  strcpy(d, delim.c_str());

  char *p = strtok(strs, d);
  while (p) {
    std::string s = p; //分割得到的字符串转换为string类型
    res.push_back(s);  //存入结果数组
    p = strtok(NULL, d);
  }

  return res;
}
} // namespace lidar_localization