#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <chrono>
#include "class_multiview.h"

int main()
{
    MultiView multiview;
    
    // // left
    // {
    //     //
    //     std::vector<cv::Mat> calib_imgs;
    //     for (int i = 0; i < 11; i++)
    //     {
    //         std::stringstream ss;
    //         ss << "../data/calib/left/" << i << ".bmp";
    //         cv::Mat img = cv::imread(ss.str());
    //         calib_imgs.push_back(img);
    //     }

    //     cv::Size board_size(8, 6);
    //     float    square_size = 28.57;
    //     cv::Mat  camera_matrix;
    //     cv::Mat  dist_coeffs;
    //     multiview.calib_cam(calib_imgs, board_size, square_size, camera_matrix, dist_coeffs);

    //     std::string calib_result_filepath = "../data/calib/left/result.yaml";
    //     cv::FileStorage fs(calib_result_filepath, cv::FileStorage::WRITE);
    //     fs << "camera_matrix" << camera_matrix;
    //     fs << "dist_coeffs" << dist_coeffs;

    //     //
    //     std::vector<cv::Point2f> corners_img;
    //     corners_img.push_back(cv::Point2f(56, 961));
    //     corners_img.push_back(cv::Point2f(10, 763));
    //     corners_img.push_back(cv::Point2f(403, 742));
    //     corners_img.push_back(cv::Point2f(711, 894));

	//     std::vector<cv::Point3f> corners_ground;
    //     corners_ground.push_back(cv::Point3f(0,0,0));
    //     corners_ground.push_back(cv::Point3f(0,2300,0));
    //     corners_ground.push_back(cv::Point3f(1620,2300,0));
    //     corners_ground.push_back(cv::Point3f(1620,0,0));

    //     cv::Mat  mat_ground_to_cam;
    //     multiview.calib_ground(camera_matrix, dist_coeffs, corners_img, corners_ground, mat_ground_to_cam);
    //     fs << "mat_ground_to_cam" << mat_ground_to_cam;

    // }

    // // right
    // {
    //     //
    //     std::vector<cv::Mat> calib_imgs;
    //     for (int i = 2; i < 11; i++)
    //     {
    //         std::stringstream ss;
    //         ss << "../data/calib/right/" << i << ".bmp";
    //         cv::Mat img = cv::imread(ss.str());
    //         calib_imgs.push_back(img);
    //     }

    //     cv::Size board_size(8, 6);
    //     float    square_size = 28.57;
    //     cv::Mat  camera_matrix;
    //     cv::Mat  dist_coeffs;
    //     multiview.calib_cam(calib_imgs, board_size, square_size, camera_matrix, dist_coeffs);

    //     std::string calib_result_filepath = "../data/calib/right/result.yaml";
    //     cv::FileStorage fs(calib_result_filepath, cv::FileStorage::WRITE);
    //     fs << "camera_matrix" << camera_matrix;
    //     fs << "dist_coeffs" << dist_coeffs;

    //     //
    //     std::vector<cv::Point2f> corners_img;
    //     corners_img.push_back(cv::Point2f(16, 849));
    //     corners_img.push_back(cv::Point2f(113, 665));
    //     corners_img.push_back(cv::Point2f(500, 662));
    //     corners_img.push_back(cv::Point2f(708, 827));

	//     std::vector<cv::Point3f> corners_ground;
    //     corners_ground.push_back(cv::Point3f(0,0,0));
    //     corners_ground.push_back(cv::Point3f(0,2300,0));
    //     corners_ground.push_back(cv::Point3f(1620,2300,0));
    //     corners_ground.push_back(cv::Point3f(1620,0,0));

    //     cv::Mat  mat_ground_to_cam;
    //     multiview.calib_ground(camera_matrix, dist_coeffs, corners_img, corners_ground, mat_ground_to_cam);
    //     fs << "mat_ground_to_cam" << mat_ground_to_cam;
    // }

    // human center
    {
        // left
        cv::Mat camera_matrix_left;
        cv::Mat dist_coeffs_left;
        cv::Mat  mat_ground_to_cam_left;

        std::string calib_result_filepath_left = "../data/calib/left/result.yaml";
        cv::FileStorage fs_left(calib_result_filepath_left, cv::FileStorage::READ);

        fs_left["camera_matrix"] >> camera_matrix_left;
        fs_left["dist_coeffs"] >> dist_coeffs_left;
        fs_left["mat_ground_to_cam"] >> mat_ground_to_cam_left;

        // right
        cv::Mat camera_matrix_right;
        cv::Mat dist_coeffs_right;
        cv::Mat  mat_ground_to_cam_right;

        std::string calib_result_filepath_right = "../data/calib/right/result.yaml";
        cv::FileStorage fs_right(calib_result_filepath_right, cv::FileStorage::READ);

        fs_right["camera_matrix"] >> camera_matrix_right;
        fs_right["dist_coeffs"] >> dist_coeffs_right;
        fs_right["mat_ground_to_cam"] >> mat_ground_to_cam_right;

        cv::Point3d point_3d;
        multiview.triangulate(mat_ground_to_cam_left, camera_matrix_left, cv::Point2d(281, 549),
                   mat_ground_to_cam_right, camera_matrix_right, cv::Point2d(324, 459), 
                   point_3d);
        
        std::cout << point_3d << std::endl;

    }

    std::cin.get();
    return 0;
}

