#ifndef CALIBRATION_TOOLBOX_UTILITY_HPP_
#define CALIBRATION_TOOLBOX_UTILITY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <string>
#include <yaml-cpp/yaml.h>

namespace colorful_char {

std::string info(std::string input_str) {
  return "\033[1;32m>> " + input_str + " \033[0m";
}

std::string warning(std::string input_str) {
  return "\033[1;35m>> WARNING: " + input_str + " \033[0m";
}

std::string error(std::string input_str) {
  return "\033[1;31m>> ERROR: " + input_str + " \033[0m";
}

}  // namespace colorful_char

namespace YAML {

template<> struct convert<Eigen::Affine3d> {
  static Node encode(const Eigen::Affine3d & rhs) {
    YAML::Node node = YAML::Load("[]");
    for (size_t idx = 0; idx < 16; ++idx) node[idx] = rhs(idx / 4, idx % 4);
    return node;
  }

  static bool decode(const Node & node, Eigen::Affine3d & rhs) {
    if (!node.IsSequence() || node.size() != 16) return false;
    // clang-format off
    Eigen::Matrix4d mtx;
    mtx <<  node[0].as<double>(),  node[1].as<double>(),  node[2].as<double>(),  node[3].as<double>(), 
            node[4].as<double>(),  node[5].as<double>(),  node[6].as<double>(),  node[7].as<double>(),
            node[8].as<double>(),  node[9].as<double>(), node[10].as<double>(), node[11].as<double>(),
           node[12].as<double>(), node[13].as<double>(), node[14].as<double>(), node[15].as<double>();
    rhs = Eigen::Affine3d(mtx);
    // clang-format on
    return true;
  }
};

}  // namespace YAML

namespace fs = std::filesystem;

// Check whether the directory/file path is absolute path or relative path, as well as its validness.
bool directory_path_check(std::string & path) {
  if (path.back() != '/') path += '/';
  if (!fs::exists(path)) {
    if (path.front() != '/') path = '/' + path;
    path = ros::package::getPath("mpl_calibration_toolbox") + path;
  }
  if (!fs::exists(path)) {
    std::cerr << colorful_char::error("Invalid directory path: " + path) << std::endl;
    return false;
  }
  return true;
}

bool file_path_check(std::string & path) {
  if (!fs::exists(path)) {
    if (path.front() != '/') path = '/' + path;
    path = ros::package::getPath("mpl_calibration_toolbox") + path;
  }
  if (!fs::exists(path)) {
    std::cerr << colorful_char::error("Invalid file path: " + path) << std::endl;
    return false;
  }
  return true;
}

// First apply the coordinate transformation, then project 3D points onto 2D image frame with projection matrix.
cv::Point2d ProjectPoints(const cv::Point3d & point, const cv::Mat & projection_matrix, const Eigen::Affine3d & transformation) {
  Eigen::Vector3d obj_p(point.x, point.y, point.z);
  obj_p    = transformation.rotation() * obj_p + transformation.translation();
  double u = projection_matrix.at<double>(0, 0) * obj_p.x() / obj_p.z() + projection_matrix.at<double>(0, 2);
  double v = projection_matrix.at<double>(1, 1) * obj_p.y() / obj_p.z() + projection_matrix.at<double>(1, 2);
  return cv::Point2d(u, v);
}

// Calculate the reprojection error between a detected corner and a reprojected corner.
double CalcReprojectionError(cv::Point2d & detected_corner, cv::Point2d & reprojected_corner) {
  return std::sqrt((detected_corner - reprojected_corner).dot(detected_corner - reprojected_corner));
}

#endif  // CALIBRATION_TOOLBOX_UTILITY_HPP_
