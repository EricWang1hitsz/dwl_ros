#include <test/dwl_test.hpp>
#include <ocp/OptimalControl.h>
#include <solver/OptimizationSolver.h>
#include <solver/IpoptNLP.h>
#include <model/HS071DynamicalSystem.h>
#include <model/HS071Cost.h>


int main(int argc, char **argv)
{
    dwl::solver::OptimizationSolver *solver = new dwl::solver::IpoptNLP();
    dwl::ocp::OptimalControl optimal_control;
    solver->setOptimizationModel(&optimal_control);

    dwl::ocp::DynamicalSystem *dynamic_system = new dwl::model::HS071DynamicalSystem();
    dwl::ocp::Cost *cost = new dwl::model::HS071Cost();

    optimal_control.addDynamicalSystem(dynamic_system);
    optimal_control.addCost(cost);

    solver->setFromConfigFile("../ipopt_config.yaml");

    std::clock_t startcputime = std::clock();
    solver->init();
    solver->compute();
    Eigen::VectorXd solution = solver->getSolution();

    std::cout << solution << std::endl;

    return 0;
}
