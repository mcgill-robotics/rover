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

        /**
         * called every single tick to update the internal states
         */
        void Update(float timestep);

        /**
         * call this method before doing any IK
         */
        void DisconnectDirectInput()
        {
            m_ProcessedInputSubscriber.shutdown();
        }

        /**
         * call this method to resume manual control (non-IK)
         */
        void ConnectDirectInput()
        {
            boost::function<void(const ProcessedControllerInput& input)> bindedFunc = [this](const ProcessedControllerInput& input) -> void
            {
                DirectControllCallback(input);
            };
            m_ProcessedInputSubscriber = m_NodeHandle.subscribe<ProcessedControllerInput>("processed_arm_controller_input", 16, bindedFunc);
        }

        /**
         * get a pointer to one of the motors
         * do not take ownership of this pointer
         * 0 <= index <= 5
         */
        Motor* GetMotor(size_t index);

        Motor* operator[](size_t index)
        {
            return GetMotor(index);
        }

        /**
         * do not take ownership of these pointers
         */
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
                m_Motors[i].SetAngularVelocityRelative(input.ControllerInput[i]);
            }
        }

        Motor m_Motors[6];

        ros::NodeHandle& m_NodeHandle;
        ros::Publisher m_ArmMotorPublisher;
        ros::Subscriber m_ProcessedInputSubscriber;
        float m_LatestTimestep;
    };
}

