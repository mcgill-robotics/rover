#include "ArmControl/ArmMotorCommand.h"
#include "DirectArmControlManager.h"
#include <ros/ros.h>
#include <chrono>
#include <std_msgs/String.h>

constexpr int LOOP_RATE = 120;

int main(int argc, char **argv)
{
    using namespace Rover;
    ros::init(argc, argv, "ArmControl");
    ros::NodeHandle nodeHandle;
    ros::Rate loopRate(LOOP_RATE);

    DirectArmControlManager armControl(nodeHandle);
    auto time = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
    auto newTime = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
    while (ros::ok())
    {
        armControl.Update(std::chrono::duration_cast<std::chrono::duration<float, std::micro>>(newTime - time).count() / 1000000.0f);
        loopRate.sleep();
        time = newTime;
        newTime = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
    }

    return 0;
}
