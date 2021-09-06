#pragma once

#include "opencv2/opencv.hpp"

class MultiView
{ 
public:
    explicit MultiView();
    ~MultiView();

    bool calib_cam(const std::vector<cv::Mat> &calib_imgs_,
                   const cv::Size             &board_size_, 
                   float                       square_size_,
                   cv::Mat                    &camera_matrix_, 
                   cv::Mat                    &dist_coeffs_);

    bool calib_ground(const cv::Mat &cam_mat_, 
					  const cv::Mat &dist_coeff_,
                      const std::vector<cv::Point2f> &corners_img_,
	                  const std::vector<cv::Point3f> &corners_ground_,
                      cv::Mat &mat_ground_to_cam_ );
    
    bool triangulte(const cv::Mat &mat_ground_to_cam_0_,
                    const cv::Mat &cam_mat_0_,
                    const cv::Point2d &point_0_,
                    const cv::Mat &mat_ground_to_cam_1_,
                    const cv::Mat &cam_mat_1_,
                    const cv::Point2d &point_1_,
                    cv::Point3d &point_3d_);

private:
    MultiView(const MultiView &);
    const MultiView &operator=(const MultiView &);

    class Impl;
    Impl *_impl;
};
