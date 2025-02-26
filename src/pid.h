
#pragma once


#include <iostream>
#include <vector>
#include <functional>



enum class PIDControllerType : uint8_t
{
    /*
     * Denotes selective use of control terms for the PID controller
     */
    P,
    I,
    D,
    PI,
    IP = PI,
    ID,
    DI = ID,
    PD,
    DP = PD,
    PID,
    PDI = PID,
    IDP = PID,
    IPD = PID,
    DPI = PID,
    DIP = PID
};


struct PIDControllerParams
{
    /*
     * Overloading of "<<" operator for printing
     */
    friend std::ostream& operator<<(std::ostream& stream, const PIDControllerParams& params)
    {
        // Convert controller_type to a std::string
        std::string controller_type;
        switch (params.controller_type)
        {
            using enum PIDControllerType;

            case P:
                controller_type = "P";
                break;
            case I:
                controller_type = "I";
                break;
            case D:
                controller_type = "D";
                break;
            case PI:
                controller_type = "PI";
                break;
            case ID:
                controller_type = "ID";
                break;
            case PD:
                controller_type = "PD";
                break;
            case PID:
                controller_type = "PID";
                break;

            default:
                controller_type = "Unknown PIDControllerType";
                break;
        }

        return stream << "controller_type: " << controller_type << "\n"
                      << "k_p: " << params.k_p << "\n"
                      << "k_i: " << params.k_i << "\n"
                      << "k_d: " << params.k_d << "\n"
                      << "dt: " << params.dt << "\n"
                      << "integral: " << params.integral;
    }


    /*
     * Controller type
     */
    PIDControllerType controller_type{PIDControllerType::PID};

    /*
     * Postive tuning constants for PID controller
     */
    double k_p{0.5};
    double k_i{0.5};
    double k_d{0.5};

    /*
     * Variables for differentation and integration
     */
    double dt{0.5};
    double integral{0.0};
};


template<typename T>
class PIDController
{
    public:
        PIDController() = default;
        PIDController(PIDController&&) = default;
        PIDController(const PIDControllerParams& params) : m_params{params} {};
        PIDController(const PIDControllerType& type, 
                      double p, 
                      double i, 
                      double d, 
                      double dt, 
                      double integral) : m_params{.controller_type=type, .k_p=p, .k_i=i, .k_d=d, .dt=dt, .integral=integral} {};
        
        
        /*
         * Overloading of "<<" operator for printing
         */
        friend std::ostream& operator<<(std::ostream& stream, const PIDController& controller)
        {
            return stream << "m_pv: " << controller.m_pv << "\n"
                          << "m_sp: " << controller.m_sp << "\n"
                          << "m_e: " << controller.m_e << "\n"
                          << "m_u: " << controller.m_u << "\n"
                          << "=============" << "\n" << "m_params" << "\n" << "=============" << "\n" << controller.m_params;
        }


        /*
         * Steps the controller
         */
        T update(const T& setpoint, const T& process_variable);

        /*
         * Updates the internal parameters
         */
        void set_params(const PIDControllerParams& params) { m_params = params; };


    private:
        T m_pv{}; // Process Variable
        T m_sp{}; // Set Point
        T m_e{}; // Error
        T m_u{}; // Control Variable

        PIDControllerParams m_params; // Holds parameters for the PID controller
};




template<typename T, typename... ArgTypes>
class PIDSystem
{
    public:
        PIDSystem(PIDController<T>& controller, const std::function<T(T, ArgTypes...)>& func) : m_controller{std::move(controller)}, m_function{func} {}

        /* 
         * Updates the system by one time step
         */
        T update(const T& setpoint, const ArgTypes&... args) 
        {
            // Calculate the new control variable
            const T u = m_controller.update(setpoint, m_pv);
            // Call the process function with the new control variable to get the process variable
            m_pv = std::invoke(m_function, u, args...);

            return m_pv;
        };

        /*
         * Updates the internal process function
         */
        void set_function(const std::function<T(T)>& func) { m_function = func; };

        /*
         * Prints the internal PID controller
         */
        void print_controller() { std::cout << m_controller << "\n"; };

    private:
        T m_pv{}; // Process Variable

        PIDController<T> m_controller; // PID Controller
        std::function<T(T, ArgTypes...)> m_function; // Process Function
};



#include "pid.ipp"
