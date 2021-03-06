#include <ocp/ConstrainedDynamicalSystem.h>


namespace dwl
{

namespace ocp
{

ConstrainedDynamicalSystem::ConstrainedDynamicalSystem() : num_actived_endeffectors_(0)
{
	// Setting the name of the constraint
	name_ = "constrained";

	// Setting the locomotion variables for this dynamical constraint
	system_variables_.position = true;
	system_variables_.velocity = true;
	system_variables_.effort = true;
}


ConstrainedDynamicalSystem::~ConstrainedDynamicalSystem()
{

}


void ConstrainedDynamicalSystem::setActiveEndEffectors(const rbd::BodySelector& active_set)
{
	active_endeffectors_ = active_set;
	num_actived_endeffectors_ = active_endeffectors_.size();
}


void ConstrainedDynamicalSystem::computeDynamicalConstraint(Eigen::VectorXd& constraint,
															const WholeBodyState& state)
{
	// Resizing the constraint vector
	constraint.resize(system_.getJointDoF() + 3 * num_actived_endeffectors_);

	// Computing the step time
	double step_time = state.time - state_buffer_[0].time;

	// Computing the constrained inverse dynamics to the defined active contacts
	Eigen::VectorXd estimated_joint_forces;
	Eigen::VectorXd base_acc = (state.base_vel - state_buffer_[0].base_vel) / step_time;
	Eigen::VectorXd joint_acc = (state.joint_vel - state_buffer_[0].joint_vel) / step_time;
	dynamics_.computeConstrainedFloatingBaseInverseDynamics(estimated_joint_forces,
															state.base_pos, state.joint_pos,
															state.base_vel, state.joint_vel,
															base_acc, joint_acc,
															active_endeffectors_);
	constraint.segment(0, system_.getJointDoF()) = estimated_joint_forces - state.joint_eff;


	// This constrained inverse dynamic algorithm could generate joint forces in cases where the
	// ground (or environment) is pulling or pushing the end-effector, which an unreal situation.
	// So, it's required to impose a velocity kinematic constraint in the active end-effectors
	rbd::BodyVectorXd endeffectors_vel;
	kinematics_.computeVelocity(endeffectors_vel,
								state.base_pos, state.joint_pos,
								state.base_vel, state.joint_vel,
								active_endeffectors_, rbd::Linear);

	for (rbd::BodyVectorXd::iterator endeffector_it = endeffectors_vel.begin();
			endeffector_it != endeffectors_vel.end(); endeffector_it++) {
		// Getting the end-effector index
		std::string name = endeffector_it->first;
		unsigned int id = system_.getEndEffectors().find(name)->second;

		constraint.segment<3>(system_.getJointDoF() + 3 * id) = endeffector_it->second;
	}
}


void ConstrainedDynamicalSystem::getDynamicalBounds(Eigen::VectorXd& lower_bound,
													Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(system_.getJointDoF() + 3 * num_actived_endeffectors_);
	upper_bound = Eigen::VectorXd::Zero(system_.getJointDoF() + 3 * num_actived_endeffectors_);
}

} //@namespace ocp
} //@namespace dwl
