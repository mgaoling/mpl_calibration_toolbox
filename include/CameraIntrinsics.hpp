#ifndef CALIB_TOOLBOX_CAMERA_INTRINSICS_HPP_
#define CALIB_TOOLBOX_CAMERA_INTRINSICS_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <utility.hpp>

class CameraIntrinsics {
private:
  bool        valid_;
  std::string name_;
  cv::Size    size_;
  cv::Mat     cam_mtx_;       // Camera Matrix
  cv::Mat     dist_coeff_;    // Distortion Coefficients (k1, k2, p1, p2 [,k3 [,k4, k5, k6]])
  cv::Mat     rect_mtx_;      // Rectification Matrix
  cv::Mat     proj_mtx_;      // Projection Matrix (by calculation, not from inputs)
  cv::Mat     undist_map_x_;  // Undistortion Map on X axis
  cv::Mat     undist_map_y_;  // Undistortion Map on Y axis

public:
  CameraIntrinsics(std::string yaml_path);
  bool        status() { return valid_; }
  std::string name() { return name_; }
  int         width() { return size_.width; }
  int         height() { return size_.height; }
  cv::Size    size() { return size_; }
  cv::Mat     camera_matrix() { return cam_mtx_.clone(); }
  cv::Mat     distortion_coefficients() { return dist_coeff_.clone(); }
  cv::Mat     rectification_matrix() { return rect_mtx_.clone(); }
  cv::Mat     projection_matrix() { return proj_mtx_.clone(); }
  cv::Mat     undistortion_map_x() { return undist_map_x_.clone(); }
  cv::Mat     undistortion_map_y() { return undist_map_y_.clone(); }
};

// Construct the Camera Intrinsic instance by reading from the yaml file and calculating the projection matrix plus undistortion map.
CameraIntrinsics::CameraIntrinsics(std::string yaml_path) : valid_(true) {
  valid_ = file_path_check(yaml_path);
  if (!valid_) return;
  YAML::Node config = YAML::LoadFile(yaml_path);
  if (config.IsNull()) {
    valid_ = false;
    std::cerr << colorful_char::error("Could not read the yaml: " + yaml_path) << std::endl;
    return;
  }

  name_ = config["camera_name"].as<std::string>();
  size_ = cv::Size(config["image_width"].as<int>(), config["image_height"].as<int>());

  std::vector<double> cam_vec, dist_vec, rect_vec;
  cam_vec     = config["camera_matrix"]["data"].as<std::vector<double>>();
  dist_vec    = config["distortion_coefficients"]["data"].as<std::vector<double>>();
  rect_vec    = config["rectification_matrix"]["data"].as<std::vector<double>>();
  cam_mtx_    = cv::Mat(3, 3, CV_64F, cam_vec.data()).clone();
  dist_coeff_ = cv::Mat(1, dist_vec.size(), CV_64F, dist_vec.data()).clone();
  rect_mtx_   = cv::Mat(3, 3, CV_64F, rect_vec.data()).clone();
  proj_mtx_   = cv::getOptimalNewCameraMatrix(cam_mtx_, dist_coeff_, size_, 0);
  cv::initUndistortRectifyMap(cam_mtx_, dist_coeff_, rect_mtx_, proj_mtx_, size_, CV_32FC1, undist_map_x_, undist_map_y_);
}

#endif  // CALIB_TOOLBOX_CAMERA_INTRINSICS_HPP_
