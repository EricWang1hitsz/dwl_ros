#ifndef DWL__OCP__INELASTIC_CONTACT_VELOCITY_CONSTRAINT__H
#define DWL__OCP__INELASTIC_CONTACT_VELOCITY_CONSTRAINT__H

#include <ocp/ComplementaryConstraint.h>


namespace dwl
{

namespace ocp
{

class InelasticContactVelocityConstraint : public ComplementaryConstraint
{
	public:
		/** @brief Constructor function */
		InelasticContactVelocityConstraint();

		/** @brief Destructor function */
		~InelasticContactVelocityConstraint();

		/**
		 * @brief Initializes the ineslatic contact velocity constraint given an URDF model (xml)
		 * @param Print model information
		 */
		void init(bool info);

		/**
		 * @brief Computes the first complement constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		void computeFirstComplement(Eigen::VectorXd& constraint,
									const WholeBodyState& state);

		/**
		 * @brief Computes the second complement constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		void computeSecondComplement(Eigen::VectorXd& constraint,
									 const WholeBodyState& state);


	private:
		/** @brief End-effector names */
		std::vector<std::string> end_effector_names_;
};

} //@namespace ocp
} //@namespace dwl

#endif
