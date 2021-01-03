#include "DirectArmControlManager.h"
#include "ArmControl/ArmMotorCommand.h"
#include <cassert>

namespace Rover 
{
    using namespace ArmControl;

    DirectArmControlManager::DirectArmControlManager(ros::NodeHandle& handle)
        : nodeHandle(handle), arm_motor_publisher(handle.advertise<ArmMotorCommand>("arm_control_data", 16)),
          latestTimestep(0.0f)
    {
        ConnectDirectInput();
    }

    void DirectArmControlManager::update(float timestep)
    {
        ROS_INFO("Elapsed: %f", timestep);
        latestTimestep = timestep;

#ifdef _DEBUG
        for (size_t i = 0; i < 6; i++)
        {
            if (std::abs(motors[i].get_angular_velocity_relative() > 1.0f))
            {
                throw std::runtime_error("Motor speed out of range");
            }
        }
#endif
        // update motor state per tick
        std::for_each(std::begin(motors), std::end(motors), [=](Motor& motor) -> void
        {
            motor.update(timestep);
        });
        ArmMotorCommand command;
        for (size_t i = 0; i < 6; i++)
        {
            command.MotorVel[i] = static_cast<int8_t>(motors[i].get_angular_velocity_relative() * 100);
        }
        arm_motor_publisher.publish(command);
        ros::spinOnce(); // if applicable, call controller input callback
    }

    Motor* DirectArmControlManager::get_motor(size_t index)
    {
        assert(index >= 0 && index <= 5);
        return &motors[index];
    }
}

