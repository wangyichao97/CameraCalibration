#include "Chess.h"

void chess::CameraCalibration(std::vector<cv::String> vector_of_images)
{
    cv::Size image_size;                                    //Size of image
    cv::Size board_size;                                    //Number of corners of each row and column on the calibration board
    std::vector<cv::Point2f> image_points_buf;              //Cache the corner information detected in each images
    std::vector<std::vector<cv::Point2f>> image_points_seq; //Save all detected corners

    board_size = cv::Size(7, 5);

    //Find the corners of each images
    for (int i = 0; i < vector_of_images.size(); i++)
    {
        cv::Mat src_image, gray_Image;
        src_image = cv::imread(vector_of_images[i]);
        cv::cvtColor(src_image, gray_Image, cv::COLOR_RGB2GRAY);

        image_size.width = src_image.cols;
        image_size.height = src_image.rows;

        // Find corners
        if (0 == cv::findChessboardCorners(src_image,board_size, image_points_buf,cv::CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY))
        {
            std::cout << "can not find chessboard corners!\n";
            exit(1);
        }
        else
        {
            // find sub-pixel points to improve accuracy
            cv::cornerSubPix(gray_Image, image_points_buf,
                             cv::Size(5, 5),cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            image_points_seq.push_back(image_points_buf);

            //Show corners
            cv::drawChessboardCorners(src_image, board_size, image_points_buf, true);
            cv::namedWindow("Chessboard Corners", 0);
            cv::imshow("Chessboard Corners", src_image);

            cv::waitKey(0);
        }
    }

    cv::Size square_size;                                   //Size of each checkerboard on the calibration board
    cv::Mat camera_matrix;                                  //Camera internal parameter matrix
    cv::Mat dist_coeffs;                                    //Five distortion coefficients of the camera: k1,k2,p1,p2,k3
    std::vector<cv::Mat> r_vector;                          //Rotation vector of each images
    std::vector<cv::Mat> t_vector;                          //Translation vector of each images
    std::vector<std::vector<cv::Point3f>> object_points;    // Save the 3D coordinates of the corners on the calibration plate

    //Parameters setting
    square_size = cv::Size(15, 15);

    camera_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    dist_coeffs = cv::Mat(1, 5, CV_32FC1,cv::Scalar::all(0));

    /* Initialize the three-dimensional coordinates of the corners on the calibration plate */
    for (int t = 0; t < vector_of_images.size(); t++)
    {
        std::vector<cv::Point3f> temp_pointSet;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {
                cv::Point3f realPoint;
                realPoint.x = (i + 1) * square_size.width;
                realPoint.y = (j + 1) * square_size.height;
                realPoint.z = 0;
                temp_pointSet.push_back(realPoint);
            }
        }
        object_points.push_back(temp_pointSet);
    }

    //calibrate
    calibrateCamera(object_points, image_points_seq, image_size,
                    camera_matrix, dist_coeffs, r_vector, t_vector, 0);


    double each_error = 0.0;                          //Error of each image
    double total_error = 0.0;                         //Overall error of all graphs
    std::vector<cv::Point2f> image_points;            //Save the recalculated projection points

    std::cout<<"Calibration error of each image: "<<std::endl;

    for (int i = 0;i < vector_of_images.size(); i++)
    {
        std::vector<cv::Point3f> temp_PointSet = object_points[i];

        //Calculate the 2-D coordinates projected from the 3-D points in the world coordinate system to the pixel coordinate system
        cv::projectPoints(temp_PointSet, r_vector[i], t_vector[i], camera_matrix, dist_coeffs, image_points);
        std::vector<cv::Point2f> temp_image_point = image_points_seq[i];
        cv::Mat temp_image_pointMat = cv::Mat(1, temp_image_point.size(), CV_32FC2);
        cv::Mat image_points2Mat = cv::Mat(1, image_points.size(), CV_32FC2);

        for (int j = 0 ; j < temp_image_point.size(); j++)
        {
            image_points2Mat.at<cv::Vec2f>(0,j) = cv::Vec2f(image_points[j].x, image_points[j].y);
            temp_image_pointMat.at<cv::Vec2f>(0,j) = cv::Vec2f(temp_image_point[j].x, temp_image_point[j].y);
        }

        //calculation error
        each_error = norm(image_points2Mat, temp_image_pointMat, cv::NORM_L2);
        total_error += each_error;
        std::cout << "Average error of the " << i+1 << " image:  " << each_error<< "  pixel" << std::endl;
    }
    std::cout << "Total error of images:  " << total_error/vector_of_images.size() << "  pixel" << std::endl;

    PnPCalibrateCamera(object_points,camera_matrix,dist_coeffs);
}

void chess::PnPCalibrateCamera(std::vector<std::vector<cv::Point3f>> object_points_chess, cv::Mat camera_matrix_chess,cv::Mat dist_coeffs_chess)
{
    cv::Size image_size;                                        //Size of image
    cv::Size board_size;                                        //Number of corners of each row and column on the calibration board
    cv::Mat r_vector_PnP;                                       //Rotation of each images
    cv::Mat t_vector_PnP;                                       //Translation of each images
    std::string image_name_PnP;
    std::vector<cv::String> image_vector_PnP;
    std::vector<cv::Point2f> image_points_buf_PnP;              //Cache the corner information detected in each images
    std::vector<std::vector<cv::Point2f>> image_points_seq_PnP; //Save all detected corners in PnP method

    board_size = cv::Size(7, 5);

// Get an array of image addresses
    for (int i = 50; i <= 200 ; i += 25)
    {
        image_name_PnP = "../chessBoardImage\\src\\" + std::to_string(i) + ".bmp";
        image_vector_PnP.push_back(image_name_PnP);
    }

    for (int i = 0; i < image_vector_PnP.size(); i++)
    {
        cv::Mat srcImage, grayImage;
        srcImage = cv::imread(image_vector_PnP[i]);
        cvtColor(srcImage, grayImage, cv::COLOR_RGB2GRAY);
        image_size.width = srcImage.cols;
        image_size.height = srcImage.rows;

        //Traverse the file to find the corner information of all images
        if (0 == cv::findChessboardCorners(srcImage, board_size, image_points_buf_PnP))
        {
            std::cout << "can not find chessboard corners!\n";
            exit(1);
        }
        else
        {
            cv::cornerSubPix(grayImage, image_points_buf_PnP, cv::Size(5, 5), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            image_points_seq_PnP.push_back(image_points_buf_PnP);
        }
    }

    //Find the tVector of each picture
    for (int i = 0; i < image_vector_PnP.size(); i++)
    {
        solvePnP(object_points_chess[i], image_points_seq_PnP[i],
                 camera_matrix_chess, dist_coeffs_chess,
                 r_vector_PnP, t_vector_PnP,
                 false,cv::SOLVEPNP_ITERATIVE);
        std::cout << t_vector_PnP << std::endl;
    }
}


