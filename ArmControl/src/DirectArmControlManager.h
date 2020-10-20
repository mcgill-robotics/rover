#pragma once
#include <stdlib.h>
#include "ArmControl/ArmMotorData.h"
#include "ArmControl/ProcessedControllerInput.h"
#include <ros/ros.h>

namespace Rover
{
    using namespace ArmControl;

    class DirectArmControlManager final
    {
    public:
        struct MotorRange
        {
            float Min;
            float Max;
        };

        // TODO : to be populated later
        // TODO : refactor
        // In radians
        static constexpr MotorRange AXIS_0_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_1_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_2_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_3_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_4_RANGE = {0.0f, 0.0f};

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
        
    private:
        void DirectControllCallback(const ProcessedControllerInput& input)
        {
            ROS_INFO("Pitch: %f\nRoll: %f\nYall: %f\nBtnCycleForward: %d\nBtnCycleBackward: %d", 
                input.PitchAxis, 
                input.RollAxis,
                input.YawAxis,
                input.BtnCycleForward, 
                input.BtnCycleForward
            );
            
            
            // TODO wrist control
        }


        ArmMotorData m_ArmData;
        ros::NodeHandle& m_NodeHandle;
        ros::Publisher m_ArmMotorPublisher;
        ros::Subscriber m_ProcessedInputSubscriber;
        size_t m_ActiveMotor;
        float m_LatestTimestep;
    };
}

