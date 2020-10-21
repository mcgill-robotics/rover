#include "DirectArmControlManager.h"
#include "ArmControl/ArmMotorCommand.h"
#include <cassert>

namespace Rover 
{
    using namespace ArmControl;

    DirectArmControlManager::DirectArmControlManager(ros::NodeHandle& handle)
        : m_NodeHandle(handle), m_ArmMotorPublisher(handle.advertise<ArmMotorCommand>("arm_control_data", 16)),
          m_LatestTimestep(0.0f)
    {
        boost::function<void(const ProcessedControllerInput& input)> bindedFunc = [this](const ProcessedControllerInput& input) -> void
        {
            DirectControllCallback(input);
        };
        m_ProcessedInputSubscriber = handle.subscribe<ProcessedControllerInput>("ProcessedArmControllerInput", 16, bindedFunc);
    }

    void DirectArmControlManager::Update(float timestep)
    {
        ROS_INFO("Elapsed: %f", timestep);
        m_LatestTimestep = timestep;

#ifdef _DEBUG
        for (size_t i = 0; i < 6; i++)
        {
            if (std::abs(m_Motors[i].GetAngularVelocityRelative() > 1.0f))
            {
                throw std::runtime_error("Motor speed out of range");
            }
        }
#endif
        // update motor state per tick
        std::for_each(std::begin(m_Motors), std::end(m_Motors), [=](Motor& motor) -> void
        {
            motor.Update(timestep);
        });
        ArmMotorCommand command;
        for (size_t i = 0; i < 6; i++)
        {
            command.MotorVel[i] = static_cast<int8_t>(m_Motors[i].GetAngularVelocityRelative() * 100);
        }
        m_ArmMotorPublisher.publish(command);
        ros::spinOnce(); // if applicable, call controller input callback
    }

    Motor* DirectArmControlManager::GetMotor(size_t index)
    {
        assert(index >= 0 && index <= 5);
        return &m_Motors[index];
    }
}

