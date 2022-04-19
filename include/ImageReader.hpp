#ifndef CALIBRATION_TOOLBOX_IMAGE_READER_HPP_
#define CALIBRATION_TOOLBOX_IMAGE_READER_HPP_

#include <utility.hpp>

class ImageReader {
private:
  bool                     valid_;
  std::string              dir_name_;
  std::vector<cv::Mat>     img_vec_;
  std::vector<std::string> img_name_vec_;
  std::vector<std::string> img_path_vec_;

public:
  ImageReader(std::string dir_path);
  bool        status() { return valid_; }
  std::string directory_name() { return dir_name_; }
  int         size() { return img_vec_.size(); }
  cv::Mat &   image(size_t index) { return img_vec_[index]; }
  std::string image_name(size_t index) { return img_name_vec_[index]; }
  std::string image_path(size_t index) { return img_path_vec_[index]; }
};

// Validate and read each image in the input directory.
ImageReader::ImageReader(std::string dir_path) : valid_(true) {
  valid_ = directory_path_check(dir_path);
  if (!valid_) return;
  dir_path.pop_back();
  dir_name_ = dir_path.substr(dir_path.find_last_of("/\\") + 1);
  dir_path += '/';

  // Read each image's absolute path from the input directory.
  fs::path               dir(dir_path);
  fs::directory_iterator img_path_it(dir);
  fs::directory_iterator end_it;
  for (; img_path_it != end_it; ++img_path_it) {
    if (fs::is_regular_file(img_path_it->status()) && img_path_it->path().extension().string() == ".png") {
      img_name_vec_.emplace_back(img_path_it->path().stem().string());
      img_path_vec_.emplace_back(img_path_it->path().string());
    }
  }
  std::sort(img_name_vec_.begin(), img_name_vec_.end());
  std::sort(img_path_vec_.begin(), img_path_vec_.end());
  for (auto & img_path : img_path_vec_) {
    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
    if (img.empty()) {
      valid_ = false;
      std::cerr << colorful_char::error("Could not read the image: " + img_path) << std::endl;
      continue;
    }
    img_vec_.emplace_back(img);
  }
}

#endif  // CALIBRATION_TOOLBOX_IMAGE_READER_HPP_
