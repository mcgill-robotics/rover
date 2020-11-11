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

    static constexpr int linear_axis = 1; //1 is the index controlling up/down axis on xbox 360 controller
    static constexpr int rotation_axis = 0; //0 is the index controlling up/down axis on xbox 360 controller
    
    static constexpr float linear_scale = 0.49;
    static constexpr float rotation_scale = 0.49;

    static constexpr float radius_wheel = 0.15; //radius of the wheel in meters
    static constexpr float length = 0.4; //wheel to center in meters

    float linear_veloctiy; //V / linear speed of rover
    float rotational_velocity; //omega / turning speed of the whole rover around axis at its location

    static constexpr float maxV = 2.42; //maximum linear velocity in m/s
    static constexpr float maxOmega = 6.048; //maximum omega in rad/s


};


Drive_Control::Drive_Control()
   
{


    //handle.setParam("linear_scale", 0.5);
    //handle.setParam("rotation_scale",0.5);



    
    vel_pub = handle.advertise<DriveControl::WheelSpeed>("WheelSpeed", 1);

    joy_sub =  handle.subscribe<sensor_msgs::Joy>("joy", 10, &Drive_Control::joyCallback, this);

    
}

void Drive_Control::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    DriveControl::WheelSpeed speed;

    
    linear_veloctiy = joy->axes[linear_axis] * maxV * linear_scale; //linear velocity received from controller input
    rotational_velocity = joy->axes[rotation_axis] * maxOmega * rotation_scale; //rotational velocity received from controller input

    speed.Wheel_Speed[0] = (linear_veloctiy + (rotational_velocity*length))/radius_wheel; //left wheel speed
    speed.Wheel_Speed[1] = (linear_veloctiy - (rotational_velocity*length))/radius_wheel; //right wheel speed



    vel_pub.publish(speed);  //publish wheelspeed message to WheelSpeed topic



}





int main(int argc, char** argv)
{
    ros::init(argc,argv, "DriveControlNode");
    Drive_Control rover_drive;

    ros::spin();


    return 0;
}
