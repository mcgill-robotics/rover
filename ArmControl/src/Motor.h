#pragma once

#include <stdexcept>

namespace Rover
{
    // tentative
    class Motor
    {
    public:
        Motor();

        void SetAngularVelocityRelative(float vel)
        {
            m_AngularVelocity = vel;
        }

        void SetAngularVelocityAbsolute(float vel)
        {
            throw std::runtime_error("Not implemented");
        }

        float GetAngularVelocityRelative() const
        {
            return m_AngularVelocity;
        }

        float GetAngularVelocityAbsolute() const
        {
            throw std::runtime_error("Not implemented");
        }

        void Update(float timestep);

    private:
        float m_NormalizedPosition;
        float m_AngularVelocity; // -1.0f to 1.0f
        float m_Min; // ?
        float m_Max;
    };
}
