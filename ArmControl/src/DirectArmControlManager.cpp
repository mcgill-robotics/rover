#include "DirectArmControlManager.h"
#include "ArmControl/ArmMotorData.h"

namespace Rover 
{
    using namespace ArmControl;

    DirectArmControlManager::DirectArmControlManager(ros::NodeHandle& handle)
        : m_ArmData(), m_NodeHandle(handle), m_ArmMotorPublisher(handle.advertise<ArmControl::ArmMotorData>("arm_control_data", 16)),
          m_ActiveMotor(0), m_LatestTimestep(0.0f)
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
        m_ArmMotorPublisher.publish(m_ArmData);
        ros::spinOnce(); // if applicable, call controller input callback
    }
}

