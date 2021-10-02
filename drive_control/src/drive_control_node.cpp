#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <drive_control/WheelSpeed.h>


class drive_control
{
public:
    drive_control();

private:

    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
    

    ros::NodeHandle handle;

    ros::Publisher velPub;
    ros::Subscriber joySub;

    static constexpr int linearAxis = 1; //1 is the index controlling up/down axis on xbox 360 controller
    static constexpr int rotationAxis = 0; //0 is the index controlling up/down axis on xbox 360 controller
    
    static constexpr float kv = 0.5; //just a constant
    static constexpr float ko = 0.5; //just a constant

    static constexpr float radiusWheel = 0.15; //radius of the wheel in meters
    static constexpr float length = 0.4; //wheel to center in meters

    float linearVelocity; //V / linear speed of rover
    float rotationalVelocity; //omega / turning speed of the whole rover around axis at its location

    static constexpr float maxV = 2.41; //maximum linear velocity in m/s
    static constexpr float maxOmega = 6.047; //maximum omega in rad/s

    double x; //hold value of joystick horizontally
    double y; //hold value of joystick vertically
    
};


drive_control::drive_control()
   
{

    handle.setParam("dev", "/dev/input/js0"); //set the joystick device from which to receive joy messages

    velPub = handle.advertise<DriveControl::WheelSpeed>("wheel_speed", 1);

    joySub =  handle.subscribe<sensor_msgs::Joy>("joy", 10, &drive_control::joy_callback, this);

    
}

void drive_control::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    DriveControl::WheelSpeed speed;
    

     x = joy->axes[rotationAxis];
     y = joy->axes[linearAxis];
    
     linearVelocity = kv*0.5* maxV * y * (2-abs(x)); //linear velocity received from controller input
     rotationalVelocity = ko * 0.5 * maxOmega * x * (2-abs(y)); //rotational velocity received from controller input

     speed.wheel_speed[0] = (linearVelocity + (rotationalVelocity*length))/radiusWheel; //left wheel speed
     speed.wheel_speed[1] = (linearVelocity - (rotationalVelocity*length))/radiusWheel; //right wheel speed

    ROS_INFO("wheel_speed sent");

    velPub.publish(speed);  //publish wheelspeed message to WheelSpeed topic



}





int main(int argc, char** argv)
{
    ros::init(argc,argv, "drive_control_node");
    drive_control rover_drive;

    ros::spin();


    return 0;
}
