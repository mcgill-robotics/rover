#pragma once
#include <stdlib.h>
#include "ArmControl/ArmMotorData.h"
#include <ros/ros.h>

namespace Rover
{
    class DirectArmControlManager final
    {
    public:
        struct MotorRange
        {
            float Max;
            float Min;
        };

        // TODO : to be populated later
        static constexpr MotorRange AXIS_0_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_1_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_2_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_3_RANGE = {0.0f, 0.0f};
        static constexpr MotorRange AXIS_4_RANGE = {0.0f, 0.0f};

        struct ArmData
        {
            ArmData()
            {
                memset(this, 0, sizeof(ArmData));
            }

            union
            {
                struct
                {
                    float Axis0;
                    float Axis1;
                    float Axis2;
                    float Axis3;
                    float Axis4;
                };
                float AxisData[5];
                ArmControl::ArmMotorData ArmDataMessageFormat;
            };

            // strict aliasing has been turned off so this is a non-issue
            operator ArmControl::ArmMotorData() const
            {
                return ArmDataMessageFormat;
            }
        };
        static_assert(sizeof(ArmControl::ArmMotorData) == 5 * sizeof(float));
        static_assert(sizeof(ArmData) == 5 * sizeof(float));
        static_assert(offsetof(ArmData, Axis4) == offsetof(ArmControl::ArmMotorData, axis4));

        DirectArmControlManager(ros::NodeHandle& handle);

        void Update(float timestep);
        
    private:
        ArmData m_ArmData;
        ros::NodeHandle& m_NodeHandle;
        ros::Publisher m_ArmMotorPublisher;
    };
}

