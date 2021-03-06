#include <CameraIntrinsics.hpp>
#include <CeresCostFunctor.hpp>
#include <Checkerboard.hpp>
#include <ImageReader.hpp>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "joint_extrinsic_calibrator");
  ros::NodeHandle nh;

  bool vis_on;
  ros::param::get("/show_visualization", vis_on);

  // Read checkerboard's basic info.
  int    board_width, board_height;
  double board_square_size;
  ros::param::get("/checkerboard_width", board_width);
  ros::param::get("/checkerboard_height", board_height);
  ros::param::get("/checkerboard_square_size", board_square_size);
  Checkerboard board = Checkerboard(board_width, board_height, board_square_size);

  // Read each camera's intrinsic result from the input path.
  std::vector<CameraIntrinsics> intrinsics_vec;
  std::vector<std::string>      intrinsics_path_vec;
  ros::param::get("/intrinsic_yaml_path", intrinsics_path_vec);
  for (std::string intrinsics_path : intrinsics_path_vec) {
    intrinsics_vec.emplace_back(intrinsics_path);
    if (!intrinsics_vec.back().status()) {
      ros::shutdown();
      return -1;
    }
  }
  int cam_num = intrinsics_vec.size();

  // Read each camera's images from the input image directory path.
  std::vector<ImageReader> img_reader_vec;
  std::vector<std::string> img_dir_path_vec;
  ros::param::get("/image_directory_path", img_dir_path_vec);
  for (size_t idx = 0; idx < img_dir_path_vec.size(); ++idx) {
    img_reader_vec.emplace_back(img_dir_path_vec[idx]);
    if (!img_reader_vec.back().status()) {
      ros::shutdown();
      return -1;
    } else if (idx >= cam_num) {
      ROS_ERROR("%s", colorful_char::error("The number of the intrisic yaml does not match with the image directory.").c_str());
      ros::shutdown();
      return -1;
    } else if (img_reader_vec.back().directory_name() != intrinsics_vec[idx].name()) {
      ROS_ERROR("%s", colorful_char::error("The order of the intrisic yaml does not match with the image directory.").c_str());
      ros::shutdown();
      return -1;
    } else if (img_reader_vec.front().size() != img_reader_vec.back().size()) {
      ROS_ERROR("%s", colorful_char::error("The number of images under each directory is not consistent with each other.").c_str());
      ros::shutdown();
      return -1;
    }
  }
  int img_num = img_reader_vec.front().size();

  // ----------------------------------------------------------------------- //

  // Detect checkerboard pattern on each undistorted image.
  bool                                               warning_detected = false;
  std::vector<std::vector<std::vector<cv::Point2d>>> corners_vec;
  std::vector<Eigen::Quaterniond>                    cam_rotation_vec;     // initial guess calculated by PnP.
  std::vector<cv::Mat>                               cam_translation_vec;  // --> from the Board frame to the first Camera fram
  for (size_t cam_idx = 0; cam_idx < cam_num; ++cam_idx) {
    corners_vec.emplace_back(std::vector<std::vector<cv::Point2d>>());
    for (size_t img_idx = 0; img_idx < img_num; ++img_idx) {
      cv::Mat img = img_reader_vec[cam_idx].image(img_idx);
      cv::remap(img, img, intrinsics_vec[cam_idx].undistortion_map_x(), intrinsics_vec[cam_idx].undistortion_map_y(), cv::INTER_LINEAR);
      cv::Mat gray_img;
      if (img.channels() == 1) gray_img = img.clone();
      else
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

      cv::Mat corners, cam_r_vec, cam_t_vec;
      corners_vec[cam_idx].emplace_back(std::vector<cv::Point2d>());
      if (cv::findChessboardCorners(gray_img, board.size(), corners)) {
        cv::cornerSubPix(gray_img, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.01));
        cv::drawChessboardCorners(img, board.size(), corners, true);
        for (size_t pt_idx = 0; pt_idx < board_width * board_height; ++pt_idx)
          corners_vec[cam_idx][img_idx].emplace_back(corners.at<cv::Point2f>(pt_idx, 0).x, corners.at<cv::Point2f>(pt_idx, 0).y);
      } else {
        warning_detected = true;
        ROS_WARN("%s", colorful_char::warning("No pattern found in image: " + img_reader_vec[cam_idx].image_path(img_idx)).c_str());
      }

      if (!cv::solvePnP(board.object_points(), corners, intrinsics_vec[cam_idx].camera_matrix(), cv::noArray(), cam_r_vec, cam_t_vec)) {
        warning_detected = true;
        ROS_WARN("%s", colorful_char::warning("Could not solve the PnP problem.").c_str());
      }
      cv::Mat cam_r_mtx;
      cv::Rodrigues(cam_r_vec, cam_r_mtx);
      Eigen::Matrix3d eigen_r_mtx;
      cv::cv2eigen(cam_r_mtx, eigen_r_mtx);
      Eigen::Quaterniond q(eigen_r_mtx);
      cam_rotation_vec.emplace_back(q);
      cam_translation_vec.emplace_back(cam_t_vec);
      cv::putText(img, "Press any Key to Continue.", cv::Point(cv::Size(20, 35)), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 255), 2);
      cv::putText(img, intrinsics_vec[cam_idx].name() + " " + img_reader_vec[cam_idx].image_name(img_idx), cv::Point(cv::Size(20, 90)),
                  cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2);
      if (vis_on) {
        cv::imshow("Checkerboard Pattern", img);
        cv::waitKey(0);
      }
    }
  }
  if (vis_on) cv::destroyAllWindows();
  if (warning_detected) {
    ROS_ERROR("%s", colorful_char::error("Calibration terminated. Please remove the unwanted images.").c_str());
    ros::shutdown();
    return -1;
  }

  // ----------------------------------------------------------------------- //

  // Optimization Objective: minimize the 2D-3D reprojection errors (PNP) for each camera, plus the correspondence matching errors (ICP) for
  // point cloud scannings (only in Camera-LiDAR calibration).
  ceres::Problem      problem;
  std::vector<double> unit_q{1, 0, 0, 0};  // Unit Quaternion
  std::vector<double> zero_t{0, 0, 0};     // Zero Translation
  std::vector<double> q_cr_b_vec;          // Coordinate Transformation
  std::vector<double> t_cr_b_vec;          // --> from the Board frame (world frame) to the first Camera frame (Reference frame)
  std::vector<double> q_cx_cr_vec;         // Camera Relative Transformation
  std::vector<double> t_cx_cr_vec;         // --> from the first Camera pose (Reference pose) to any other Camera X pose

  // Setup initial values for parameters to be solved.
  q_cr_b_vec.reserve(4 * img_num);
  t_cr_b_vec.reserve(3 * img_num);
  for (size_t img_idx = 0; img_idx < img_num; ++img_idx) {
    q_cr_b_vec.emplace_back(cam_rotation_vec[img_idx].w());
    q_cr_b_vec.emplace_back(cam_rotation_vec[img_idx].x());
    q_cr_b_vec.emplace_back(cam_rotation_vec[img_idx].y());
    q_cr_b_vec.emplace_back(cam_rotation_vec[img_idx].z());
    t_cr_b_vec.emplace_back(cam_translation_vec[img_idx].at<double>(0, 0));
    t_cr_b_vec.emplace_back(cam_translation_vec[img_idx].at<double>(0, 1));
    t_cr_b_vec.emplace_back(cam_translation_vec[img_idx].at<double>(0, 2));
    double * param_q_cr_b = q_cr_b_vec.data() + img_idx * 4;
    double * param_t_cr_b = t_cr_b_vec.data() + img_idx * 3;
    problem.AddParameterBlock(param_q_cr_b, 4, new ceres::QuaternionParameterization());
    problem.AddParameterBlock(param_t_cr_b, 3);
  }
  q_cx_cr_vec.reserve(4 * (cam_num - 1));
  t_cx_cr_vec.reserve(3 * (cam_num - 1));
  for (size_t cam_idx = 0; cam_idx < cam_num - 1; ++cam_idx) {
    q_cx_cr_vec.insert(q_cx_cr_vec.end(), unit_q.begin(), unit_q.end());
    t_cx_cr_vec.insert(t_cx_cr_vec.end(), zero_t.begin(), zero_t.end());
    double * param_q_cx_cr = q_cx_cr_vec.data() + cam_idx * 4;
    double * param_t_cx_cr = t_cx_cr_vec.data() + cam_idx * 3;
    problem.AddParameterBlock(param_q_cx_cr, 4, new ceres::QuaternionParameterization());
    problem.AddParameterBlock(param_t_cx_cr, 3);
  }

  // Setup cost functions for optimization.
  for (size_t img_idx = 0; img_idx < img_num; ++img_idx) {
    double * param_q_cr_b = q_cr_b_vec.data() + img_idx * 4;
    double * param_t_cr_b = t_cr_b_vec.data() + img_idx * 3;
    for (size_t pt_idx = 0; pt_idx < board_width * board_height; ++pt_idx) {
      problem.AddResidualBlock(
        CamRefPNP::create(board.object_points(pt_idx), corners_vec[0][img_idx][pt_idx], intrinsics_vec[0].projection_matrix()), nullptr,
        param_q_cr_b, param_t_cr_b);  // At here, 0 represents the first camera (as reference).
    }
  }
  for (size_t img_idx = 0; img_idx < img_num; ++img_idx) {
    double * param_q_cr_b = q_cr_b_vec.data() + img_idx * 4;
    double * param_t_cr_b = t_cr_b_vec.data() + img_idx * 3;
    for (size_t pt_idx = 0; pt_idx < board_width * board_height; ++pt_idx) {
      for (size_t cam_idx = 0; cam_idx < cam_num - 1; ++cam_idx) {
        double * q_cx_cr = q_cx_cr_vec.data() + cam_idx * 4;
        double * t_cx_cr = t_cx_cr_vec.data() + cam_idx * 3;
        problem.AddResidualBlock(CamXPNP::create(board.object_points(pt_idx), corners_vec[cam_idx + 1][img_idx][pt_idx],
                                                 intrinsics_vec[cam_idx + 1].projection_matrix()),  // Note that camera starts from 1.
                                 nullptr, param_q_cr_b, param_t_cr_b, q_cx_cr, t_cx_cr);
      }
    }
  }

  // Run the ceres solver.
  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Construct the optimized results in the form of 3*4 Transformation Matrix
  std::vector<Eigen::Affine3d> T_cr_b_vec, T_cx_cr_vec;
  for (size_t img_idx = 0; img_idx < img_num; ++img_idx)
    T_cr_b_vec.emplace_back(
      Eigen::Translation3d(t_cr_b_vec[img_idx * 3], t_cr_b_vec[img_idx * 3 + 1], t_cr_b_vec[img_idx * 3 + 2])
      * Eigen::Quaterniond(q_cr_b_vec[img_idx * 4], q_cr_b_vec[img_idx * 4 + 1], q_cr_b_vec[img_idx * 4 + 2], q_cr_b_vec[img_idx * 4 + 3])
          .normalized());
  for (size_t cam_idx = 0; cam_idx < cam_num - 1; ++cam_idx)
    T_cx_cr_vec.emplace_back(Eigen::Translation3d(t_cx_cr_vec[cam_idx * 3], t_cx_cr_vec[cam_idx * 3 + 1], t_cx_cr_vec[cam_idx * 3 + 2])
                             * Eigen::Quaterniond(q_cx_cr_vec[cam_idx * 4], q_cx_cr_vec[cam_idx * 4 + 1], q_cx_cr_vec[cam_idx * 4 + 2],
                                                  q_cx_cr_vec[cam_idx * 4 + 3])
                                 .normalized());

  // ----------------------------------------------------------------------- //

  // Output extrinsic results both on terminal and in yaml file.
  std::ofstream fout(ros::package::getPath("mpl_calibration_toolbox") + "/data/joint_camera_extrinsic_results.yaml");
  YAML::Node    output_yaml;
  output_yaml["camera_number"] = cam_num;
  for (size_t idx = 0; idx < cam_num; ++idx) {
    output_yaml["cam" + std::to_string(idx)]["camera_name"] = intrinsics_vec[idx].name();
    if (idx == 0) continue;
    std::cout << colorful_char::info("Transformation from " + intrinsics_vec[idx].name() + " to " + intrinsics_vec[0].name() + ": ")
              << std::endl;
    std::cout << T_cx_cr_vec[idx - 1].inverse().matrix() << std::endl;
    output_yaml["cam" + std::to_string(idx)]["T_cam0_cam" + std::to_string(idx)] = T_cx_cr_vec[idx - 1].inverse();
  }
  fout << output_yaml;
  fout.close();
  std::cout << colorful_char::info("Extrinsic results are saved in /data/joint_camera_extrinsic_results.yaml") << std::endl;

  // Run validation-used visualization on reprojection errors.
  for (size_t cam_idx = 0; cam_idx < cam_num; ++cam_idx) {
    double residual = 0;
    for (size_t img_idx = 0; img_idx < img_num; ++img_idx) {
      for (size_t pt_idx = 0; pt_idx < board_width * board_height; ++pt_idx) {
        cv::Point2d proj_corner;
        if (cam_idx == 0)
          proj_corner = ProjectPoints(board.object_points(pt_idx), intrinsics_vec[cam_idx].projection_matrix(), T_cr_b_vec[img_idx]);
        else
          proj_corner = ProjectPoints(board.object_points(pt_idx), intrinsics_vec[cam_idx].projection_matrix(),
                                      T_cx_cr_vec[cam_idx - 1] * T_cr_b_vec[img_idx]);
        residual += CalcReprojectionError(corners_vec[cam_idx][img_idx][pt_idx], proj_corner);
        cv::circle(img_reader_vec[cam_idx].image(img_idx), proj_corner, 2, cv::Scalar(255, 0, 0), 3);
      }
      residual /= board_width * board_height;
      cv::putText(img_reader_vec[cam_idx].image(img_idx), "Residuals = " + std::to_string(residual) + "pix", cv::Point(cv::Size(20, 125)),
                  cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2);
      if (residual >= 2)
        ROS_WARN("%s", colorful_char::warning("The overall reprojection error is higher than 2pix on image: "
                                              + img_reader_vec[cam_idx].image_path(img_idx))
                         .c_str());
      if (vis_on) {
        cv::imshow("Reprojection Results", img_reader_vec[cam_idx].image(img_idx));
        cv::waitKey(0);
      }
    }
  }
  if (vis_on) cv::destroyAllWindows();

  ros::shutdown();
  return 0;
}
