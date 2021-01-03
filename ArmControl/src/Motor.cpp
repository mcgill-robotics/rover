#include "Motor.h"

#include <ros/ros.h>

namespace Rover
{
    Motor::Motor()
        : min(0.0f), max(0.0f)
    {

    }

    void Motor::update(float timestep)
    {
        // TODO : manage the internal state of the motor
    }
}
