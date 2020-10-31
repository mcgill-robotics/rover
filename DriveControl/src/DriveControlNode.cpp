#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>


class Drive_Control
{
public:
    Drive_Control();

private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    

    ros::NodeHandle handle;

    //ros::Publisher drive_pub;
    ros::Subscriber drive_sub;

    int linear;
    int angular;

};


Drive_Control::Drive_Control():
    linear(1),
    angular(2)
{

    drive_sub =  handle.subscribe<sensor_msgs::Joy>("joy", 10, &Drive_Control::joyCallback, this);

}

void Drive_Control::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

}





int main(int argc, char** argv)
{
    ros::init(argc,argv, "DriveControlNode");
    Drive_Control drive;

    ros::spin();


    return 0;
}
