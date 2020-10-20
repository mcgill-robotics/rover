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
        // In radians
        static constexpr MotorRange AXIS_0_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_1_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_2_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_3_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_4_RANGE = {0.0f, 0.0f};

        DirectArmControlManager(ros::NodeHandle& handle);

        void Update(float timestep);

        float& GetMotorData(size_t index)
        {
            ROS_ASSERT(index <= 4 && index >= 0);
            // TODO : optimize
            switch (index)
            {
                case 0:
                {
                    return m_ArmData.Axis0;
                }
                case 1:
                {
                    return m_ArmData.Axis1;
                }
                case 2:
                {
                    return m_ArmData.Axis2;
                }
                case 3:
                {
                    return m_ArmData.Axis3;
                }
                case 4:
                {
                    return m_ArmData.Axis4;
                }
                default:
                {
                    ROS_ASSERT(false);
                }
            }
        }

        float& operator[](size_t index)
        {
            return GetMotorData(index);
        }

        void SelectMotor(size_t index)
        {
            ROS_ASSERT(index <= 4 && index >= 0);
            m_ActiveMotor = index;
        }

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
                input.RollAxis, // ignore
                input.YawAxis, // ignore
                input.BtnCycleForward, 
                input.BtnCycleForward
            );
            if (input.BtnCycleForward)
            {
                CycleForward();
            }
            if (input.BtnCycleBackward)
            {
                CycleBackward();
            }
            GetMotorData(m_ActiveMotor) += input.PitchAxis * m_LatestTimestep;
        }

        void EnsureMotorIndexInRange()
        {
            if (m_ActiveMotor > 4)
            {
                m_ActiveMotor -= 5;
            }
            if (m_ActiveMotor < 0)
            {
                m_ActiveMotor += 5;
            }
        }

        void CycleForward()
        {
            m_ActiveMotor++;
            EnsureMotorIndexInRange();
        }

        void CycleBackward()
        {
            m_ActiveMotor--;
            EnsureMotorIndexInRange();
        }

        ArmMotorData m_ArmData;
        ros::NodeHandle& m_NodeHandle;
        ros::Publisher m_ArmMotorPublisher;
        ros::Subscriber m_ProcessedInputSubscriber;
        size_t m_ActiveMotor;
        float m_LatestTimestep;
    };
}

