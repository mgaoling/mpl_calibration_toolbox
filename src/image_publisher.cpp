#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <utility.hpp>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  ros::Publisher  pub_img = nh.advertise<sensor_msgs::Image>("/recorded_image", 1000);

  std::string dir_path;
  ros::param::get("/image_directory_path", dir_path);
  if (dir_path.back() != '/') dir_path += '/';

  // Read each image's absolute path from the input directory.
  boost::filesystem::path               dir(dir_path);
  boost::filesystem::directory_iterator it_file(dir);
  boost::filesystem::directory_iterator end_it_file;
  std::vector<std::string>              file_path_vec;
  std::vector<std::string>              file_name_vec;
  for (; it_file != end_it_file; ++it_file) {
    if (boost::filesystem::is_regular_file(it_file->status()) && it_file->path().extension().string() == ".png") {
      file_path_vec.emplace_back(it_file->path().string());
      file_name_vec.emplace_back(it_file->path().stem().string());
    }
  }
  if (file_path_vec.empty()) {
    std::cerr << colorful_char::error("Found no images under the directory: " + dir_path) << std::endl << std::endl;
    ros::shutdown();
    return -1;
  } else {
    std::sort(file_path_vec.begin(), file_path_vec.end());
    std::sort(file_name_vec.begin(), file_name_vec.end());
  }

  // Publish each image via ROS communication.
  for (size_t idx = 0; idx < file_path_vec.size(); ++idx) {
    cv::Mat img = cv::imread(file_path_vec[idx], cv::IMREAD_COLOR);
    if (img.empty()) {
      ROS_ERROR_STREAM(colorful_char::error("Could not read the image: " + file_name_vec[idx]));
      continue;
    }
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header   header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(img_msg);
    pub_img.publish(img_msg);
    cv::waitKey(1000);
  }

  ros::shutdown();
  return 0;
}