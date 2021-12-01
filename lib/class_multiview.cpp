#include "class_multiview.h"
#include "class_multiview_.h"

class MultiView::Impl
{
public:
    _MultiView _multiview;
};

MultiView::MultiView() : _impl(new MultiView::Impl())
{
}

MultiView::~MultiView()
{
    delete _impl;
    _impl = NULL;
}

bool MultiView::calib_cam(const std::vector<cv::Mat> &calib_imgs_,
                          const cv::Size             &board_size_, 
                          float                       square_size_,
                          cv::Mat                    &camera_matrix_, 
                          cv::Mat                    &dist_coeffs_)
{
    return _impl->_multiview.calib_cam(calib_imgs_, 
                                       board_size_, 
                                       square_size_, 
                                       camera_matrix_, 
                                       dist_coeffs_);
}

bool MultiView::calib_ground(const cv::Mat &cam_mat_, 
                                  const cv::Mat &dist_coeff_,
                                  const std::vector<cv::Point2f> &corners_img_,
                                  const std::vector<cv::Point3f> &corners_ground_,
                                  cv::Mat &mat_ground_to_cam_ )
{
    return _impl->_multiview.calib_ground(cam_mat_, 
                                          dist_coeff_, 
                                          corners_img_, 
                                          corners_ground_, 
                                          mat_ground_to_cam_);
}

bool MultiView::triangulate(const cv::Mat &mat_ground_to_cam_0_,
                    const cv::Mat &cam_mat_0_,
                    const cv::Point2d &point_0_,
                    const cv::Mat &mat_ground_to_cam_1_,
                    const cv::Mat &cam_mat_1_,
                    const cv::Point2d &point_1_,
                    cv::Point3d &point_3d_)
{
    return _impl->_multiview.triangulate(mat_ground_to_cam_0_,
                                        cam_mat_0_,
                                        point_0_,
                                        mat_ground_to_cam_1_,
                                        cam_mat_1_,
                                        point_1_,
                                        point_3d_);
}
