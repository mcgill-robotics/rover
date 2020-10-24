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
        void SetAngularVelocityRelative(float vel)
        {
            m_AngularVelocity = vel;
        }

        float GetAngularVelocityRelative() const
        {
            return m_AngularVelocity;
        }

        /**
         * update the internal state of the motor if needed
         * this method is called for every motor and every tick
         */
        void Update(float timestep);

    private:
        // tentative
        // TODO : change this to fit your need
        float m_AngularPosition; // in rad
        float m_AngularVelocity; // in rad/sec
        float m_Min; // ?
        float m_Max;
    };
}
