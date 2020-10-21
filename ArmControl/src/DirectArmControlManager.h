#pragma once
#include <stdlib.h>
#include "ArmControl/ArmMotorCommand.h"
#include "ArmControl/ProcessedControllerInput.h"
#include "Motor.h"
#include <ros/ros.h>

static_assert(std::is_trivially_copyable<ArmControl::ProcessedControllerInput>::value);

namespace Rover
{
    using namespace ArmControl;

    class DirectArmControlManager final
    {
    public:
        DirectArmControlManager(ros::NodeHandle& handle);

        void Update(float timestep);

        void DisconnectDirectInput()
        {
            m_ProcessedInputSubscriber.shutdown();
        }

        void ConnectDirectInput()
        {
            boost::function<void(const ProcessedControllerInput& input)> bindedFunc = [this](const ProcessedControllerInput& input) -> void
            {
                DirectControllCallback(input);
            };
            m_ProcessedInputSubscriber = m_NodeHandle.subscribe<ProcessedControllerInput>("ProcessedArmControllerInput", 16, bindedFunc);
        }

        Motor* GetMotor(size_t index);

        Motor* operator[](size_t index)
        {
            return GetMotor(index);
        }

        Motor* GetTurnTableMotor() { return &m_Motors[0]; };
        Motor* GetShoulderMotor() { return &m_Motors[1]; };
        Motor* GetElbowMotor() { return &m_Motors[2]; };
        Motor* GetWristXMotor() { return &m_Motors[3]; };
        Motor* GetWristYMotor() { return &m_Motors[4]; };
        Motor* GetClawMotor() { return &m_Motors[5]; };
        
    private:
        void DirectControllCallback(const ProcessedControllerInput& input)
        {
            for (size_t i = 0; i < 6; i++)
            {
                GetMotor(i)->SetAngularVelocityRelative(input.ControllerInput[i]);
            }
        }

        Motor m_Motors[6];

        ros::NodeHandle& m_NodeHandle;
        ros::Publisher m_ArmMotorPublisher;
        ros::Subscriber m_ProcessedInputSubscriber;
        size_t m_ActiveMotor;
        float m_LatestTimestep;
    };
}

