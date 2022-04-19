#ifndef CALIBRATION_TOOLBOX_CERES_COST_FUNCTOR_HPP_
#define CALIBRATION_TOOLBOX_CERES_COST_FUNCTOR_HPP_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

// PNP-based Cost Functor for the first camera (reference Camera)
class CamRefPNP {
private:
  cv::Point3d obj_pts_;
  cv::Point2d img_pts_;
  cv::Mat     proj_mtx_;

public:
  CamRefPNP(const cv::Point3d & object_points, const cv::Point2d & image_points, const cv::Mat & projection_matrix) :
    obj_pts_(object_points), img_pts_(image_points), proj_mtx_(projection_matrix) { }

  template<typename T> bool operator()(const T * const q_cr_b, const T * const t_cr_b, T * residual) const {
    // Coordinate transformation from the Board frame (world frame) to the first Camera frame (Reference frame)
    // P_CAM_REF_3D = T_CAM_REF_BOARD(q_cr_b, t_cr_b) * P_BOARD_3D
    T pt_in[3], pt_out[3];
    pt_in[0] = T(obj_pts_.x);
    pt_in[1] = T(obj_pts_.y);
    pt_in[2] = T(obj_pts_.z);
    ceres::QuaternionRotatePoint(q_cr_b, pt_in, pt_out);
    pt_out[0] += t_cr_b[0];
    pt_out[1] += t_cr_b[1];
    pt_out[2] += t_cr_b[2];

    // Project 3D points onto 2D image frame with projection matrix.
    // P_2D = Projection_Matrix * P_3D
    T proj_u = proj_mtx_.at<double>(0, 0) * pt_out[0] / pt_out[2] + proj_mtx_.at<double>(0, 2);
    T proj_v = proj_mtx_.at<double>(1, 1) * pt_out[1] / pt_out[2] + proj_mtx_.at<double>(1, 2);

    // Construct the residuals by minimizing the reprojection errors.
    T corner_u  = T(img_pts_.x);
    T corner_v  = T(img_pts_.y);
    residual[0] = proj_u - corner_u;
    residual[1] = proj_v - corner_v;
    return true;
  }

  static ceres::CostFunction * create(const cv::Point3d object_points, const cv::Point2d image_points, const cv::Mat projection_matrix) {
    return new ceres::AutoDiffCostFunction<CamRefPNP, 2, 4, 3>(new CamRefPNP(object_points, image_points, projection_matrix));
  }
};

// PNP-based Cost Functor for any other camera
class CamXPNP {
private:
  cv::Point3d obj_pts_;
  cv::Point2d img_pts_;
  cv::Mat     proj_mtx_;

public:
  CamXPNP(const cv::Point3d object_points, const cv::Point2d image_points, const cv::Mat projection_matrix) :
    obj_pts_(object_points), img_pts_(image_points), proj_mtx_(projection_matrix) { }

  template<typename T>
  bool operator()(const T * const q_cr_b, const T * const t_cr_b, const T * const q_cx_cr, const T * const t_cx_cr, T * residual) const {
    // Construct coordinate transformation from the Board frame (world frame) to Camera X frame
    // T_CAM_X_BOARD(q_cx_b, t_cx_b) = T_CAM_X_CAM_REF(q_cx_cr, t_cx_cr) * T_CAM_REF_BOARD(q_cr_b, t_cr_b)
    T q_cx_b[4], t_cx_b[3];
    ceres::QuaternionProduct(q_cx_cr, q_cr_b, q_cx_b);
    ceres::QuaternionRotatePoint(q_cx_cr, t_cr_b, t_cx_b);
    t_cx_b[0] += t_cx_cr[0];
    t_cx_b[1] += t_cx_cr[1];
    t_cx_b[2] += t_cx_cr[2];

    // Coordinate transformation from the Board frame (world frame) to Camera X frame
    // P_CAM_X_3D = T_CAM_X_BOARD(q_cx_b, t_cx_b) * P_BOARD_3D
    T pt_in[3], pt_out[3];
    pt_in[0] = T(obj_pts_.x);
    pt_in[1] = T(obj_pts_.y);
    pt_in[2] = T(obj_pts_.z);
    ceres::QuaternionRotatePoint(q_cx_b, pt_in, pt_out);
    pt_out[0] += t_cx_b[0];
    pt_out[1] += t_cx_b[1];
    pt_out[2] += t_cx_b[2];

    // Project 3D points onto 2D image frame with projection matrix.
    // P_2D = Projection_Matrix * P_3D
    T proj_u = proj_mtx_.at<double>(0, 0) * pt_out[0] / pt_out[2] + proj_mtx_.at<double>(0, 2);
    T proj_v = proj_mtx_.at<double>(1, 1) * pt_out[1] / pt_out[2] + proj_mtx_.at<double>(1, 2);

    // Construct the residuals by minimizing the reprojection errors.
    T corner_u  = T(img_pts_.x);
    T corner_v  = T(img_pts_.y);
    residual[0] = proj_u - corner_u;
    residual[1] = proj_v - corner_v;
    return true;
  }

  static ceres::CostFunction * create(const cv::Point3d object_points, const cv::Point2d image_points, const cv::Mat projection_matrix) {
    return new ceres::AutoDiffCostFunction<CamXPNP, 2, 4, 3, 4, 3>(new CamXPNP(object_points, image_points, projection_matrix));
  }
};

#endif  // CALIBRATION_TOOLBOX_CERES_COST_FUNCTOR_HPP_
