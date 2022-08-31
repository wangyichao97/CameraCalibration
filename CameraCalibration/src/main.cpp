#include "Chess.h"

//using namespace cv;
//using namespace std;

int main()
{
    chess chessboard;

    // Get an array of image addresses
    std::string src_image_folder, str_image_name;
    std::vector<cv::String> image_vector;
    src_image_folder = "../chessBoardImage\\calibrate\\";
    str_image_name = src_image_folder + "*.bmp";
    cv::glob(str_image_name, image_vector);

    if (image_vector.size() == 0)
    {
        std::cout << "File is empty!" << std::endl;
        return -1;
    }

    chessboard.CameraCalibration(image_vector);

    return 0;
}
