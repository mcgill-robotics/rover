#include "Motor.h"

#include <ros/ros.h>

namespace Rover
{
    Motor::Motor()
        : m_Min(0.0f), m_Max(0.0f)
    {

    }

    void Motor::Update(float timestep)
    {
        // TODO : manage the internal state of the motor
    }
}
