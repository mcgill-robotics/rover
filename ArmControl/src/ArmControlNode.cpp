#include "ArmControl/ArmMotorCommand.h"
#include "DirectArmControlManager.h"
#include <ros/ros.h>
#include <chrono>
#include <std_msgs/String.h>

// change this if needed
constexpr int LOOP_RATE = 120;


///////////////////////////////
// Message to the next person
//
// The DirectArmControlManager class is my suggested way to interact with the underlying
// motors since it's an abstraction that has already been built for you. If you need
// additional abstractions, feel free to add to this class, but you don't need to 
// interface with the hardware yourself. When you get a chance to see the encoder feedback
// message format, add this to the DirectArmControlManager, and modify the Motor classes
// so that the Motor class can manage its own state. It's up to you to decide how to do
// that.
// For the Motor class, you will not need to instantiate them yourself. You can get
// an instance to any of the 6 motors by calling methods on DirectArmControlManager,
// either by index or by name.
// The DirectArmControlManager class owns these pointers!
// When the program is ready for inverse kinematics, call the DirectArmControl::DisconnectDirectInput
// method to "hijack" the class. Only after that can you safely subscribe to the processed
// joystick input and do IK. As said, you will be getting encoder data from the Motor
// class and IK output will be sent to the Motor class.
// In case IK fails, call DirectArmControl::ConnectDirectInput. You don't want to leave
// the rover in a state where it's arm is paralyzed.
// You might want to find a way to alert the operator in case IK fails.
// You might want to find a way to allow the operator to restart IK.
// You might also want to find a way for the operator to disable IK in case they believe it failed.
// Direct input is connected by default when the class is instantiated.
//
// About building this package:
// Debug and Release configurations have been setup in CMakeLists.txt for this package.
// The Debug config has a few sanitizers enabled, definitely take advantage of them.
//
// That's all I have for you. GOOD LUCK!
///////////////////////////////


int main(int argc, char **argv)
{
    using namespace Rover;
    ros::init(argc, argv, "ArmControl");
    ros::NodeHandle nodeHandle;
    ros::Rate loopRate(LOOP_RATE);

    DirectArmControlManager armControl(nodeHandle);
    // this code is used to calculate the timestep between ticks
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
