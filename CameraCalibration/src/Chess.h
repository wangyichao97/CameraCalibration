/*****************************************************************//**
 * \file   Chess.h
 * \brief  Use calibrateCamera and solvePnP to calibrate the camera and estimate the pose.
 *
 * \author wangyichao
 * \date   July 2022
 *********************************************************************/

#include <iostream>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d.hpp"

class chess
{
public:

    /**
     * @brief  Calibration using a Chessboard
     * @param vector_of_images  Vector containing the address of each picture
     *
    */
    void CameraCalibration(std::vector<cv::String> vector_of_images);

    /**
    * @brief  Calibration using a Chessboard
    * @param object_points_chess  World coordinate system
    * @param vector_of_images  Internal parameter matrix of camera
    * @param vector_of_images  Distortion matrix of camera
    *
    */
    void PnPCalibrateCamera(std::vector<std::vector<cv::Point3f>> object_points_chess, cv::Mat camera_matrix_chess,cv::Mat dist_coeffs_chess);

};

