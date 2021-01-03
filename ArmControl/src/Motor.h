#pragma once

#include <stdexcept>

namespace Rover
{
    class Motor
    {
    public:
        Motor();

        /**
         * -1.0f <= vel <= 1.0f
         * reepresents the motor speed output as a percentage
         */
        void set_angular_velocity_relative(float vel)
        {
            angularVelocity = vel;
        }

        float get_angular_velocity_relative() const
        {
            return angularVelocity;
        }

        /**
         * update the internal state of the motor if needed
         * this method is called for every motor and every tick
         */
        void update(float timestep);

    private:
        // tentative
        // TODO : change this to fit your need
        float angularPosition; // in rad
        float angularVelocity; // in rad/sec
        float min; // ?
        float max;
    };
}
