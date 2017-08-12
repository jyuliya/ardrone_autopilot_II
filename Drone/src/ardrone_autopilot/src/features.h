#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

//Struct for frame on screen
struct Box {
    float left;
    float right;
    float top;
    float bottom;
};

//Struct for info for each circle on screen
struct CirclesMessage {
    std::vector<cv::RotatedRect> circles;
    Box box;
    std::vector<bool> inTheBox;
};

extern void processImage(cv::Mat&, CirclesMessage&);
