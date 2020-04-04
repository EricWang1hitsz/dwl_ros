#ifndef DWL__SOLVER__OPTIMIZATION_SOLVER__H
#define DWL__SOLVER__OPTIMIZATION_SOLVER__H

#include <model/OptimizationModel.h>
#include <utils/YamlWrapper.h>
#include <utils/utils.h>


namespace dwl
{

namespace solver
{

/**
 * @class OptimizationSolver
 * @brief Abstract class for implementing different optimization solver such as Nonlinear
 * Optimization Programming (NLP), Quadratic Programming (QP), Sequential Quadratic Programming
 * (SQP), etc.
 */
class OptimizationSolver
{
	public:
		/** @brief Constructor function */
		OptimizationSolver();

		/** @brief Destructor function */
		virtual ~OptimizationSolver();

		/**
		 * @brief Sets the optimization model
		 * @param model::OptimizationModel* Pointer to the optimization model
		 */
		void setOptimizationModel(model::OptimizationModel* model);

		/**
		 * @brief Set the configuration parameters from a yaml file
		 * @param std::string Filename
		 */
		virtual void setFromConfigFile(std::string filename);

		/**
		 * @brief Abstract method for initialization of the solver
		 * @return True if was initialized
		 */
		virtual bool init();

		/**
		 * @brief Abstract method for computing a solution of an optimization problem
		 * @return True if it was computed a solution
		 */
		virtual bool compute(double computation_time = 2e19);

		/**
		 * @brief Gets the optimization model
		 * @return the object pointer of the optimization model
		 */
		model::OptimizationModel* getOptimizationModel();

		/**
		 * @brief Gets the solution computed by the optimizer
		 * @return const Eigen::VectorXd& The solution vector
		 */
		const Eigen::VectorXd& getSolution();

		/**
		 * @brief Gets the name of the solver
		 * @return The name of the solver
		 */
		std::string getName();


	protected:
		/** @brief Name of the solver */
		std::string name_;

		/** @brief Optimization model */
		model::OptimizationModel* model_;

		/** @brief The solution vector */
		Eigen::VectorXd solution_;
};

} //@namespace solver
} //@namespace dwl


#endif
