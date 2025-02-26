

#include <fstream>

#include "pid.h"




double process_function(const double& x, const double& y)
{
    return x+y+1000;
}



int main()
{

    PIDControllerParams params{.controller_type=PIDControllerType::IDP, .k_p=.1, .k_i=.2, .k_d=.1, .dt=.5};
    PIDController<double> pid_controller(params);
    PIDSystem<double, double> pid_system(pid_controller, process_function);


    
    double process_variable{};
    double set_point = -500;
    constexpr size_t num_iterations = 1000;

    // std::ofstream file("data.dat");


    for (size_t i=0; i<num_iterations; ++i)
    {
        pid_system.print_controller();
        // process_variable += pid_controller.update(set_point, process_variable);
        process_variable = pid_system.update(set_point, 2500);
        std::cout << "Process Variable Output: " << process_variable << "\n\n";

        // file << process_variable << "\n";
    }

}

