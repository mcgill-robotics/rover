#include <ros/ros.h>
#include <std_msgs/String.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ArmControl");
    ros::NodeHandle nodeHandle;
    ros::Rate loopRate(60);
    
    int* ptr = new int[10];
    ptr[10] = 20;

    while (ros::ok())
    {
    }

    return 0;
}
