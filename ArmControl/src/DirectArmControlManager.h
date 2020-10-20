#pragma once
#include <stdlib.h>
#include "ArmControl/ArmMotorData.h"
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
        
    private:
        ArmMotorData m_ArmData;
        ros::NodeHandle& m_NodeHandle;
        ros::Publisher m_ArmMotorPublisher;
    };
}

