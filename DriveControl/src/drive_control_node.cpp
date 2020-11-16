#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <DriveControl/WheelSpeed.h>


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
    
    static constexpr float linearScale = 0.49;
    static constexpr float rotationScale = 0.49;

    static constexpr float radiusWheel = 0.15; //radius of the wheel in meters
    static constexpr float length = 0.4; //wheel to center in meters

    float linearVelocity; //V / linear speed of rover
    float rotationalVelocity; //omega / turning speed of the whole rover around axis at its location

    static constexpr float maxV = 2.42; //maximum linear velocity in m/s
    static constexpr float maxOmega = 6.048; //maximum omega in rad/s


};


drive_control::drive_control()
   
{


    velPub = handle.advertise<DriveControl::WheelSpeed>("wheel_speed", 1);

    joySub =  handle.subscribe<sensor_msgs::Joy>("joy", 10, &drive_control::joy_callback, this);

    
}

void drive_control::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    DriveControl::WheelSpeed speed;

    
    linearVelocity = joy->axes[linearAxis] * maxV * linearScale; //linear velocity received from controller input
    rotationalVelocity = joy->axes[rotationAxis] * maxOmega * rotationScale; //rotational velocity received from controller input

    speed.wheel_speed[0] = (linearVelocity + (rotationalVelocity*length))/radiusWheel; //left wheel speed
    speed.wheel_speed[1] = (linearVelocity - (rotationalVelocity*length))/radiusWheel; //right wheel speed



    velPub.publish(speed);  //publish wheelspeed message to WheelSpeed topic



}





int main(int argc, char** argv)
{
    ros::init(argc,argv, "drive_control_node");
    drive_control rover_drive;

    ros::spin();


    return 0;
}
