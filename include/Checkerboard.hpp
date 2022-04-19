#ifndef CALIBRATION_TOOLBOX_CHECKERBOARD_HPP_
#define CALIBRATION_TOOLBOX_CHECKERBOARD_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

class Checkerboard {
private:
  double                   sq_len_;
  cv::Size                 size_;
  std::vector<cv::Point3d> obj_pts_;

public:
  Checkerboard(int width, int height, double square_size);
  int                      width() { return size_.width; }
  int                      height() { return size_.height; }
  double                   square_length() { return sq_len_; }
  cv::Size                 size() { return size_; }
  std::vector<cv::Point3d> object_points() { return obj_pts_; }
  cv::Point3d              object_points(size_t index) { return obj_pts_[index]; }
};

// Construct a 3D point vector to represent the checkerboard pattern in world frame (with z = 0) under the same order as OpenCV 2D corners.
Checkerboard::Checkerboard(int width, int height, double square_size) : sq_len_(square_size), size_(width, height) {
  for (int idx_y = size_.height - 1; idx_y >= 0; --idx_y) {
    for (size_t idx_x = 0; idx_x < size_.width; ++idx_x) obj_pts_.emplace_back(sq_len_ * idx_x, sq_len_ * idx_y, 0);
  }
}

#endif  // CALIBRATION_TOOLBOX_CHECKERBOARD_HPP_
