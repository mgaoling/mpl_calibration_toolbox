#include <CameraIntrinsics.hpp>
#include <Checkerboard.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "handeye_calibrator");
  ros::NodeHandle nh;

  bool vis_on;
  ros::param::get("show_visualization", vis_on);

  // Read checkerboard's basic info.
  int    board_width, board_height;
  double board_square_size;
  ros::param::get("checkerboard_width", board_width);
  ros::param::get("checkerboard_height", board_height);
  ros::param::get("checkerboard_square_size", board_square_size);
  Checkerboard board = Checkerboard(board_width, board_height, board_square_size);

  // Read camera's intrinsic result from the input path.
  std::string intrinsics_path;
  ros::param::get("/intrinsic_yaml_path", intrinsics_path);
  if (!fs::exists(intrinsics_path)) {
    if (intrinsics_path.front() != '/') intrinsics_path = '/' + intrinsics_path;
    intrinsics_path = ros::package::getPath("mpl_calibration_toolbox") + intrinsics_path;
  }
  CameraIntrinsics intrinsics = CameraIntrinsics(intrinsics_path);
  if (!intrinsics.status()) {
    ros::shutdown();
    return -1;
  }

  // Prepare rosbag for viewing.
  std::string bag_path;
  ros::param::get("/rosbag_path", bag_path);
  if (!file_path_check(bag_path)) {
    ros::shutdown();
    return -1;
  }
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  rosbag::View view(bag);

  // Read images and mocap poses from rosbag.
  int         num_blurred_imgs = 0;
  int         num_valid_imgs   = 0;
  int         num_skipped_imgs = 0;
  int         num_imgs         = 0;
  int         num_curr_gt_msgs = 0;
  int         img_freq, gt_freq, skipped_interval;
  std::string img_topic, gt_topic;
  double      residual_thershold;
  ros::param::get("/camera_frequency", img_freq);
  ros::param::get("/mocap_frequency", gt_freq);
  ros::param::get("/camera_topic", img_topic);
  ros::param::get("/mocap_topic", gt_topic);
  ros::param::get("/skipped_interval", skipped_interval);
  ros::param::get("/pnp_residual_thershold", residual_thershold);
  std::queue<cv::Mat>                     img_queue;
  std::queue<std::pair<cv::Mat, cv::Mat>> gt_pose_queue;
  std::vector<cv::Mat> cam_rotation_vec, cam_translation_vec, mocap_rotation_vec, mocap_translation_vec;  // absolute pose
  for (const rosbag::MessageInstance & msg : view) {
    if (msg.getTopic() == img_topic) {
      img_queue.emplace(cv_bridge::toCvCopy(msg.instantiate<sensor_msgs::Image>(), "bgr8")->image);
      ++num_imgs;
    } else if (msg.getTopic() == gt_topic) {
      // Skip the extra mocap poses with respect to image frequency.
      if (num_curr_gt_msgs % (gt_freq / img_freq) == 0) {
        geometry_msgs::PoseStamped::Ptr gt_msg    = msg.instantiate<geometry_msgs::PoseStamped>();
        geometry_msgs::Quaternion       gt_orient = gt_msg->pose.orientation;
        geometry_msgs::Point            gt_pos    = gt_msg->pose.position;
        cv::Mat                         gt_r_mtx, gt_t_vec;
        cv::eigen2cv(Eigen::Quaterniond(gt_orient.w, gt_orient.x, gt_orient.y, gt_orient.z).toRotationMatrix(), gt_r_mtx);
        cv::eigen2cv(Eigen::Translation3d(gt_pos.x, gt_pos.y, gt_pos.z).vector(), gt_t_vec);
        gt_pose_queue.emplace(gt_r_mtx, gt_t_vec);
        num_curr_gt_msgs = 0;
      }
      ++num_curr_gt_msgs;
    }

    // Skip the extra readings on both sensors.
    if (img_queue.size() >= skipped_interval && gt_pose_queue.size() >= skipped_interval) {
      cv::Mat img               = img_queue.front();
      cv::Mat mocap_rotation    = gt_pose_queue.front().first;
      cv::Mat mocap_translation = gt_pose_queue.front().second;
      cv::Mat cam_r_vec, cam_t_vec;
      for (size_t idx = 0; idx < skipped_interval; ++idx) {
        img_queue.pop();
        gt_pose_queue.pop();
      }

      // Detect checkerboard pattern on each image, solve the 2D-3D PnP problem.
      cv::Mat gray_img, corners, proj_pts;
      if (img.channels() == 1) gray_img = img.clone();
      else
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
      if (!cv::findChessboardCorners(gray_img, board.size(), corners)) {
        ROS_WARN("%s", colorful_char::warning("No pattern found.").c_str());
        ++num_blurred_imgs;
        if (vis_on) {
          cv::imshow("Checkerboard Pattern Visualization", img);
          cv::waitKey(0);
        }
        continue;
      }
      cv::cornerSubPix(gray_img, corners, cv::Size(11, 11), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.01));
      cv::drawChessboardCorners(img, board.size(), corners, true);
      if (!cv::solvePnP(board.object_points(), corners, intrinsics.camera_matrix(), intrinsics.distortion_coefficients(), cam_r_vec,
                        cam_t_vec)) {
        ROS_WARN("%s", colorful_char::warning("Could not solve the PnP problem.").c_str());
        ++num_blurred_imgs;
        if (vis_on) {
          cv::imshow("Checkerboard Pattern Visualization", img);
          cv::waitKey(0);
        }
        continue;
      }
      cv::projectPoints(board.object_points(), cam_r_vec, cam_t_vec, intrinsics.camera_matrix(), intrinsics.distortion_coefficients(),
                        proj_pts);
      double residual = 0;
      for (size_t idx = 0; idx < board.object_points().size(); ++idx) {
        double dx = corners.at<cv::Point2f>(idx, 0).x - proj_pts.at<cv::Point2d>(idx, 0).x;
        double dy = corners.at<cv::Point2f>(idx, 0).y - proj_pts.at<cv::Point2d>(idx, 0).y;
        residual += std::sqrt(dx * dx + dy * dy);
        cv::circle(img, proj_pts.at<cv::Point2d>(idx, 0), 2, cv::Scalar(255, 0, 0), 3);
      }
      residual /= board.object_points().size();
      cv::putText(img, "Residuals = " + std::to_string(residual) + "pix", cv::Point(cv::Size(20, 35)), cv::FONT_HERSHEY_DUPLEX, 1,
                  cv::Scalar(0, 255, 0), 2);
      if (residual >= residual_thershold) {
        ROS_WARN("%s", colorful_char::warning("The overall reprojection error is higher than threshold.").c_str());
        ++num_blurred_imgs;
        if (vis_on) {
          cv::imshow("Checkerboard Pattern Visualization", img);
          cv::waitKey(0);
        }
        continue;
      }
      ++num_valid_imgs;

      // Store rotation matrix and translation vector of the camera and mocap readings.
      cv::Mat cam_r_mtx;
      cv::Rodrigues(cam_r_vec, cam_r_mtx);
      cam_rotation_vec.emplace_back(cam_r_mtx);
      cam_translation_vec.emplace_back(cam_t_vec);
      mocap_rotation_vec.emplace_back(mocap_rotation);
      mocap_translation_vec.emplace_back(mocap_translation);
    }
  }
  num_skipped_imgs = num_imgs - num_blurred_imgs - num_valid_imgs;
  ROS_INFO("%s", colorful_char::info("Number of images:         " + std::to_string(num_imgs)).c_str());
  ROS_INFO("%s", colorful_char::info("Number of skipped images: " + std::to_string(num_skipped_imgs)).c_str());
  ROS_INFO("%s", colorful_char::info("Number of blurred images: " + std::to_string(num_blurred_imgs)).c_str());
  ROS_INFO("%s", colorful_char::info("Number of valid images:   " + std::to_string(num_valid_imgs)).c_str());
  if (num_valid_imgs == 0) {
    ROS_ERROR("%s", colorful_char::error("No valid image. Please reset the pnp_residual_thershold.").c_str());
    ros::shutdown();
    return -1;
  }

  // Handeye Calibration.
  cv::Mat r_cam_body, t_cam_body;  // Transformation from the camera frame to the body frame (mocap frame)
  cv::calibrateHandEye(mocap_rotation_vec, mocap_translation_vec, cam_rotation_vec, cam_translation_vec, r_cam_body, t_cam_body,
                       cv::CALIB_HAND_EYE_DANIILIDIS);
  Eigen::Matrix3d r_mtx;
  Eigen::Vector3d t_vec;
  cv::cv2eigen(r_cam_body, r_mtx);
  cv::cv2eigen(t_cam_body, t_vec);
  Eigen::Affine3d T = Eigen::Translation3d(t_vec) * r_mtx;

  // Output extrinsic results both on terminal and in yaml file.
  std::ofstream fout(ros::package::getPath("mpl_calibration_toolbox") + "/data/camera_mocap_extrinsic_results.yaml");
  YAML::Node    output_yaml;
  output_yaml["camera_name"] = intrinsics.name();
  std::cout << colorful_char::info("Transformation from " + intrinsics.name() + " to body: ") << std::endl;
  std::cout << T.matrix() << std::endl;
  output_yaml["T_body_cam"] = T;
  std::cout << colorful_char::info("Transformation from body to " + intrinsics.name() + ": ") << std::endl;
  std::cout << T.inverse().matrix() << std::endl;
  output_yaml["T_cam_body"] = T.inverse();
  fout << output_yaml;
  fout.close();
  ROS_INFO("%s", colorful_char::info("Extrinsic results are saved in /data/camera_mocap_extrinsic_results.yaml").c_str());

  ros::shutdown();
  return 0;
}
