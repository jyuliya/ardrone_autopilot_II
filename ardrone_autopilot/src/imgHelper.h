
/*  
    imgHelper.h
  ---------------------------------------------------
  | Contains some utility structures and functions  |
  |  for imageHandler.cpp and compVision.cpp files. |
  ---------------------------------------------------

*/

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <std_msgs/Float32MultiArray.h>

struct Box {
    float left;
    float right;
    float top;
    float bottom;
};


struct CirclesMessage {
    std::vector<cv::RotatedRect> circles;
    Box box;
    std::vector<bool> inTheBox;
};

extern void processImage(cv::Mat&, CirclesMessage&);


// Main structure of imageHandler.cpp, that contains 
//  some image information, flags and publishers.

struct ImageHandler
{
    float imgRows;
    float imgCols;

    bool cv_enabled = false;

    ros::Publisher imgPublisher;
    ros::Publisher circlePublisher;

    std::vector<double> cameraDistortion;
    boost::array<double, 9> cameraMatrix;
};


// Convert CircleMessage to Float32MultiArray (only information about box).
// It provides the possibility of using a static box.

void cmsg2BoxMultiArray(CirclesMessage& cmsg, std_msgs::Float32MultiArray& boxToSend) {
        std::vector<float> vec1 = {
                                    cmsg.box.left, 
                                    cmsg.box.right, 
                                    cmsg.box.top, 
                                    cmsg.box.bottom,
                                    imgHandler.imgRows, imgHandler.imgCols 
                                  };

        boxToSend.data.insert(boxToSend.data.end(), vec1.begin(), vec1.end());
}


// Convert CircleMessage to Float32MultiArray (information about circles).

void cmsg2MultiArray(CirclesMessage& cmsg, std_msgs::Float32MultiArray& msg) {
        std::vector<float> vec1;
        for (size_t i = 0; i != cmsg.inTheBox.size(); ++i) {
            vec1.clear();
            vec1 = {
                    cmsg.circles[i].center.x,
                    cmsg.circles[i].center.y,
                    cmsg.circles[i].size.width,
                    cmsg.circles[i].size.height,
                    cmsg.inTheBox[i]
                    };
            
            msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());
        }
} 


