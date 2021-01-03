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
        void update(float timestep);

        /**
         * call this method before doing any IK
         */
        void disconnect_direct_input()
        {
            processedInputSubscriber.shutdown();
        }

        /**
         * call this method to resume manual control (non-IK)
         */
        void ConnectDirectInput()
        {
            boost::function<void(const ProcessedControllerInput& input)> bindedFunc = [this](const ProcessedControllerInput& input) -> void
            {
                direct_control_callback(input);
            };
            processedInputSubscriber = nodeHandle.subscribe<ProcessedControllerInput>("processed_arm_controller_input", 16, bindedFunc);
        }

        /**
         * get a pointer to one of the motors
         * do not take ownership of this pointer
         * 0 <= index <= 5
         */
        Motor* get_motor(size_t index);

        Motor* operator[](size_t index)
        {
            return get_motor(index);
        }

        /**
         * do not take ownership of these pointers
         */
        Motor* get_turn_table_motor() { return &motors[0]; };
        Motor* get_shoulder_motor() { return &motors[1]; };
        Motor* get_elbow_motor() { return &motors[2]; };
        Motor* get_wrist_x_motor() { return &motors[3]; };
        Motor* get_wrist_y_motor() { return &motors[4]; };
        Motor* get_claw_motor() { return &motors[5]; };
        
    private:
        void direct_control_callback(const ProcessedControllerInput& input)
        {
            for (size_t i = 0; i < 6; i++)
            {
                motors[i].set_angular_velocity_relative(input.ControllerInput[i]);
            }
        }

        Motor motors[6];

        ros::NodeHandle& nodeHandle;
        ros::Publisher arm_motor_publisher;
        ros::Subscriber processedInputSubscriber;
        float latestTimestep;
    };
}

