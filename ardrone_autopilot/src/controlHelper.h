
/*  
    controlHelper.h
  ----------------------------------------------------------------------------
  | Contains some utility classes and functions for the controller.cpp.      |
  ----------------------------------------------------------------------------

*/

class Circle {
 public:
    float x, y, width, height;
    bool inTheBox;
    
    Circle() {}

    Circle(float _x, float _y, float _width, float _height, bool b):
        x(_x), 
        y(_y), 
        width(_width), 
        height(_height), 
        inTheBox(b)
         {}
};

// Main class of the controller. Contains flags, target information, PID controller and other info.

class ControlCenter
{
    public:
	int counter;
        bool enabled, xInBox, yInBox;
        float imgRows, imgCols;
        float triCenterX, triCenterY;
        Box box;
        std::vector<Circle> targ;
        ros::Publisher cmdPublisher;
        PID pid;
        ros::Time lastLoop;
        ControlCenter(): pid(0.06, 0.07, 0, 0.02) {} // Started PID coefficients. Can be changed.
};



// Circle output
std::ostream &operator<<(std::ostream &os, Circle& c) { 
    std::string output;
    output = "X: " + std::to_string(c.x) + '\n';
    output += "Y: " + std::to_string(c.y) + '\n';
    output += "Width: " + std::to_string(c.width) + '\n';
    output += "Height: " + std::to_string(c.height) + '\n';
    output += "InTheBox: " + std::to_string(c.inTheBox) + '\n';
    return os << output;
}


