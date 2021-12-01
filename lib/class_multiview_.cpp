#include "class_multiview_.h"

static void define_calib_frame(const cv::Size             &board_size_, 
                           float                       square_size_,
						   std::vector<cv::Point3f> &object_points_)
	{
		for (int j = 0; j < board_size_.height; j++)
		{
			for (int k = 0; k < board_size_.width; k++)
			{
				object_points_.push_back(cv::Point3f(k * square_size_, j * square_size_, 0));
			}
		}
	}

static bool find_corners(const std::vector<cv::Mat>             &calib_imgs_,
						 const cv::Size                         &board_size_, 
                         std::vector<std::vector<cv::Point2f> > &image_points_)
{
    for (int i = 0; i < calib_imgs_.size(); i++)
    {
		cv::Mat img_gray;
    	cv::cvtColor(calib_imgs_[i], img_gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> pointbuf;
        bool is_found = cv::findChessboardCorners(img_gray, 
                                                    board_size_, 
                                                    pointbuf,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH | 
                                                    cv::CALIB_CB_FAST_CHECK |
                                                    cv::CALIB_CB_NORMALIZE_IMAGE);
        if (is_found)
        {
            cv::cornerSubPix(img_gray, 
                pointbuf, 
                cv::Size(11, 11),
                cv::Size(-1, -1), 
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            image_points_.push_back(pointbuf);
        }
        else
        {
			std::cout << "find corner failed" << i << std::endl;
            return false;
        }
    }

    return true;
}

_MultiView::_MultiView() {}
_MultiView::~_MultiView() {}

bool _MultiView::calib_cam(const std::vector<cv::Mat> &calib_imgs_,
						   const cv::Size             &board_size_, 
                           float                       square_size_,
                           cv::Mat                    &camera_matrix_, 
                           cv::Mat                    &dist_coeffs_)
{
    if (calib_imgs_.size() < 3)
	{
		return false;
	}
	
	// 
	std::vector<std::vector<cv::Point3f> > vec_object_points(1);
	define_calib_frame(board_size_, square_size_, vec_object_points[0]);
	vec_object_points.resize(calib_imgs_.size(), vec_object_points[0]);

	//
	std::vector<std::vector<cv::Point2f> > image_points;
	bool is_found = find_corners(calib_imgs_, board_size_, image_points);
	if (!is_found)
	{
		return false;
	}

	//
	cv::Size img_size = calib_imgs_[0].size();
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	double rms = cv::calibrateCamera(vec_object_points,
									image_points, 
									img_size, 
									camera_matrix_,
									dist_coeffs_,
									rvecs,
									tvecs, 
									cv::CALIB_FIX_K3 +
									cv::CALIB_FIX_K4 +
									cv::CALIB_FIX_K5 +
									//cv::CALIB_USE_INTRINSIC_GUESS +
									//cv::CALIB_FIX_ASPECT_RATIO +
									cv::CALIB_ZERO_TANGENT_DIST
									);

	return true;
}

bool _MultiView::calib_ground(const cv::Mat &cam_mat_, 
							   const cv::Mat &dist_coeff_,
							   const std::vector<cv::Point2f> &corners_img_,
							   const std::vector<cv::Point3f> &corners_ground_,
							   cv::Mat &mat_ground_to_cam_ )
{
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

	cv::solvePnP(corners_ground_, 
				 corners_img_,
				 cam_mat_,
				 dist_coeff_,
				 rvec, 
				 tvec, 
				 false, 
				 CV_ITERATIVE);

	double rm[9];
	cv::Mat mat_rot(3, 3, CV_64FC1, rm);
	cv::Rodrigues(rvec, mat_rot);

	cv::Mat mat_cam_to_ground = (cv::Mat_<double>(4, 4) << rm[0], rm[1], rm[2], tvec.at<double>(0, 0),
								                          rm[3], rm[4], rm[5], tvec.at<double>(1, 0),
														  rm[6], rm[7], rm[8], tvec.at<double>(2, 0),
															0,     0,     0,     1);
	mat_ground_to_cam_ = mat_cam_to_ground.inv();

	return true;
}

bool _MultiView::triangulate(const cv::Mat &mat_ground_to_cam_0_,
							const cv::Mat &cam_mat_0_,
							const cv::Point2d &point_0_,
							const cv::Mat &mat_ground_to_cam_1_,
							const cv::Mat &cam_mat_1_,
							const cv::Point2d &point_1_,
							cv::Point3d &point_3d_)
{
	cv::Mat Rt0 = mat_ground_to_cam_0_.inv();
	Rt0 = Rt0.rowRange(0,3).colRange(0,4).clone();
	cv::Mat Rt1 = mat_ground_to_cam_1_.inv();
	Rt1 = Rt1.rowRange(0,3).colRange(0,4).clone();

	std::cout << "Rt0 " << Rt0 << std::endl;
	std::cout << "Rt1 " << Rt1 << std::endl;
	std::cout << "cam_mat_0_ " << cam_mat_0_ << std::endl;
	std::cout << "cam_mat_1_ " << cam_mat_1_ << std::endl;
	std::cout << "point_0_ " << point_0_ << std::endl;
	std::cout << "point_1_ " << point_1_ << std::endl;

	std::vector<cv::Point2d> points_0;
	points_0.push_back(point_0_);

	std::vector<cv::Point2d> points_1;
	points_1.push_back(point_1_);

	cv::Mat point3d_homo;
  	cv::triangulatePoints(cam_mat_0_ * Rt0, cam_mat_1_ * Rt1, 
                          points_0, points_1,
                          point3d_homo);

	cv::Mat x = point3d_homo.col(0);

	std::cout << "x " << x << std::endl;

	x /= (x.at<double>(3,0) + DBL_EPSILON); 
	cv::Point3d p (x.at<double>(0,0), 
				   x.at<double>(1,0), 
				   x.at<double>(2,0));
	point_3d_ = p;
	
	return true;
}
