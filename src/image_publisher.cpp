#include <ImageReader.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  ros::Publisher  pub_img = nh.advertise<sensor_msgs::Image>("/recorded_image", 1000);

  int speed;
  ros::param::get("playback_speed", speed);

  // Read camera's images from the input directory path.
  std::string dir_path;
  ros::param::get("/image_directory_path", dir_path);
  ImageReader reader = ImageReader(dir_path);
  if (!reader.status()) {
    ros::shutdown();
    return 0;
  }

  // Publish each image via ROS communication.
  for (size_t idx = 0; idx < reader.size(); ++idx) {
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header   header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, reader.image(idx));
    img_bridge.toImageMsg(img_msg);
    pub_img.publish(img_msg);
    cv::waitKey(speed);
  }

  ros::shutdown();
  return 0;
}
