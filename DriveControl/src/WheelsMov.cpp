#include <ros/ros.h>


class wheels_mov
{
private:
    
    ros::NodeHandle handle;

    ros::Subscriber wheels_sub;

};

int main(int argc, char** argv)
{
    
    ros::init(argc,argv, "WheelsMov");

    return 0;
}
