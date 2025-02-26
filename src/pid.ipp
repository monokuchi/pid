
#pragma once



#include "pid.h"







template<typename T>
T PIDController<T>::update(const T& setpoint, const T& process_variable)
{
    // Set internal variables
    m_sp = setpoint;
    m_pv = process_variable;


    // Calculate the error term
    const T error = m_sp - m_pv;


    // Control Terms
    T p;
    T i;
    T d;

    // Porportional term
    if (m_params.controller_type == PIDControllerType::P ||
        m_params.controller_type == PIDControllerType::PI ||
        m_params.controller_type == PIDControllerType::PD ||
        m_params.controller_type == PIDControllerType::PID)
    { 
        p = m_params.k_p * error;
    }

    // Integral term (approximated with a Riemann sum)
    if (m_params.controller_type == PIDControllerType::I ||
        m_params.controller_type == PIDControllerType::IP ||
        m_params.controller_type == PIDControllerType::ID ||
        m_params.controller_type == PIDControllerType::PID)
    {
        m_params.integral += error * m_params.dt;
        i = m_params.k_i * m_params.integral;
    }

    // Derivative term (approximated with a difference quotient)
    if (m_params.controller_type == PIDControllerType::D ||
        m_params.controller_type == PIDControllerType::DP ||
        m_params.controller_type == PIDControllerType::DI ||
        m_params.controller_type == PIDControllerType::PID)
    {
        d = m_params.k_d * (error - m_e) / m_params.dt;
    }
        


    // Update state parameters
    switch (m_params.controller_type)
    {
        using enum PIDControllerType;

        case P:
            m_u = p + i + d;
            break;
        case I:
            m_u = i;
            break;
        case D:
            m_u = d;
            break;
        case PI:
            m_u = p + i;
            break;
        case ID:
            m_u = i + d;
            break;
        case PD:
            m_u = p + d;
            break;
        case PID:
            m_u = p + i + d;
            break;
        
        default:
            break;
    }
    m_e = error;

    return m_u;
}

