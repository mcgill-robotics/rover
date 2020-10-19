#include "DirectArmControlManager.h"
#include "ArmControl/ArmMotorData.h"

namespace Rover 
{
    DirectArmControlManager::DirectArmControlManager(ros::NodeHandle& handle)
        : m_ArmData(), m_NodeHandle(handle), m_ArmMotorPublisher(handle.advertise<ArmControl::ArmMotorData>("arm_control_data", 30))
    {}

    void DirectArmControlManager::Update(float timestep)
    {
        ROS_INFO("Elapsed: %f", timestep);
        m_ArmMotorPublisher.publish(static_cast<ArmControl::ArmMotorData>(m_ArmData));
    }
}

