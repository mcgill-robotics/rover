#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <DriveControl/WheelSpeed.h>


class Drive_Control
{
public:
    Drive_Control();

private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    

    ros::NodeHandle handle;

    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;

    int linear_axis;
    int rotation_axis;
    double linear_vel;
    double rotation_vel;

};


Drive_Control::Drive_Control():
    //default parametes in case the parameters cannot could not be retrieved
    linear_axis{1}, 
    rotation_axis{0}
{

    handle.param("linear_axis", linear_axis, linear_axis);
    handle.param("rotation_axis", rotation_axis, rotation_axis);

    handle.setParam("linear_axis",1);
    handle.setParam("rotation_axis", 0);
    
    vel_pub = handle.advertise<DriveControl::WheelSpeed>("WheelSpeed", 1);

    joy_sub =  handle.subscribe<sensor_msgs::Joy>("joy", 10, &Drive_Control::joyCallback, this);

    
}

void Drive_Control::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    DriveControl::WheelSpeed speed;
    
    //speed.Wheel_Speed[0] = joy->axes[linear_axis];
    //speed.Wheel_Speed[1] = joy->axes[rotation_axis];

    vel_pub.publish(speed);



}





int main(int argc, char** argv)
{
    ros::init(argc,argv, "DriveControlNode");
    Drive_Control drive;

    ros::spin();


    return 0;
}
