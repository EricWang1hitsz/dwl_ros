#ifndef DWL__MODEL__WHOLE_BODY_DYNAMICS__H
#define DWL__MODEL__WHOLE_BODY_DYNAMICS__H

#include <model/WholeBodyKinematics.h>
#include <model/FloatingBaseSystem.h>
#include <utils/utils.h>


namespace dwl
{

namespace model
{

/**
 * @class WholeBodyDynamics
 * @brief WholeBodyDynamics class implements the dynamics methods for a
 * floating-base robot
 */
class WholeBodyDynamics
{
	public:
		/** @brief Constructor function */
		WholeBodyDynamics();

		/** @brief Destructor function */
		~WholeBodyDynamics();

		/**
		 * @brief Builds the model rigid-body system from an URDF file
		 * @param std::string URDF filename
		 * @param std::string Semantic system description filename
		 * @param Print model information
		 */
		void modelFromURDFFile(const std::string& urdf_file,
							   const std::string& system_file = std::string(),
							   bool info = false);

		/**
		 * @brief Builds the model rigid-body system from an URDF model (xml)
		 * @param std::string URDF model
		 * @param std::string Semantic system description filename
		 * @param Print model information
		 */
		void modelFromURDFModel(const std::string& urdf_model,
								const std::string& system_file = std::string(),
								bool info = false);

		/**
		 * @brief Computes the whole-body inverse dynamics, assuming a fully
		 * actuated robot, using the Recursive Newton-Euler Algorithm (RNEA).
		 * An applied external force is defined for a certain body, movable
		 * or fixed body, where a fixed body is considered a fixed point of a
		 * movable one. These forces are represented as Cartesian forces
		 * applied to the body, where the first three elements are the moments
		 * and the last three elements are the linear forces. In general a
		 * point only has linear forces, but with this representation we can
		 * model the forces applied by a surface of contact in the center of
		 * pressure of it.
		 * @param rbd::Vector6d& Base wrench
		 * @param Eigen::VectorXd& Joint forces
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration with respect to a
		 * gravity field
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::BodyWrench External force applied to a certain
		 * body of the robot
		 */
		void computeInverseDynamics(rbd::Vector6d& base_wrench,
									Eigen::VectorXd& joint_forces,
									const rbd::Vector6d& base_pos,
									const Eigen::VectorXd& joint_pos,
									const rbd::Vector6d& base_vel,
									const Eigen::VectorXd& joint_vel,
									const rbd::Vector6d& base_acc,
									const Eigen::VectorXd& joint_acc,
									const rbd::BodyVector6d& ext_force = rbd::BodyVector6d());

		/**
		 * @brief Computes the whole-body inverse dynamics using the Recursive
		 * Newton-Euler Algorithm (RNEA) for a floating-base robot
		 * (RX,RY,RZ,TX,TY,TZ). An applied external force is defined for a
		 * certain body, movable or fixed body, where a fixed body is
		 * considered a fixed point of a movable one. These forces are
		 * represented as Cartesian forces applied to the body, where the first
		 * three elements are the moments and the last three elements are the
		 * linear forces. In general a point only has linear forces, but with
		 * this representation we can model the forces applied by a surface of
		 * contact in the center of pressure of it.
		 * @param rbd::Vector6d& Base acceleration with respect to a gravity field
		 * @param Eigen::VectorXd& Joint forces
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::BodyWrench External force applied to a certain
		 * body of the robot
		 */
		void computeFloatingBaseInverseDynamics(rbd::Vector6d& base_acc,
												Eigen::VectorXd& joint_forces,
												const rbd::Vector6d& base_pos,
												const Eigen::VectorXd& joint_pos,
												const rbd::Vector6d& base_vel,
												const Eigen::VectorXd& joint_vel,
												const Eigen::VectorXd& joint_acc,
												const rbd::BodyVector6d& ext_force = rbd::BodyVector6d());

		/**
		 * @brief Computes the constrained whole-body inverse dynamics using
		 * the Recursive Newton-Euler Algorithm (RNEA). Constrained are defined
		 * by the contacts of the robot. Contacts could be defined for movable
		 * and fixed bodies, where a fixed body is considered a fixed point of
		 * a movable one. Thus, this approach allows us to compute the inverse
		 * dynamic when we have a predefined set of contacts, and without
		 * specific information of the contact forces of these contacts. Here
		 * we are assuming that the desired movement is realizable without base
		 * wrench (i.e. the hand's God).
		 * @param Eigen::VectorXd& Joint forces
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration with respect to a
		 * gravity field
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::BodyForce External force applied to a certain body
		 * of the robot
		 */
		void computeConstrainedFloatingBaseInverseDynamics(Eigen::VectorXd& joint_forces,
														   const rbd::Vector6d& base_pos,
														   const Eigen::VectorXd& joint_pos,
														   const rbd::Vector6d& base_vel,
														   const Eigen::VectorXd& joint_vel,
														   const rbd::Vector6d& base_acc,
														   const Eigen::VectorXd& joint_acc,
														   const rbd::BodySelector& contacts);

		/**
		 * @brief Computes the joint-space inertia matrix by using the
		 * Composite Rigid Body Algorithm
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @return Eigen::MatrixXd& The joint-space inertia matrix
		 */
		const Eigen::MatrixXd& computeJointSpaceInertiaMatrix(const rbd::Vector6d& base_pos,
															  const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Computes the centroidal inertia matrix
		 * @param const Eigen::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @return rbd::Matrix6d& The centroidal inertia matrix
		 */
		const rbd::Matrix6d& computeCentroidalInertiaMatrix(const rbd::Vector6d& base_pos,
															const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Computes the gravitational wrench in the CoM position
		 * @param const Eigen::Vector3d& CoM position expressed in the world frame
		 * @return rbd::Vector6d& Gravitational wrench
		 */
		const rbd::Vector6d& computeGravitoWrench(const Eigen::Vector3d& com_pos);

		/**
		 * @brief Computes the contact forces that generates the desired base
		 * wrench. This desired base wrench is computed by using robot state,
		 * i.e. position, velocity, acceleration and contacts. This function
		 * overwrite the base and joint acceleration in case that it isn't
		 * consistent with the constrained contacts
		 * @param rbd::BodyWrench& Contact forces applied to the defined set of
		 * contacts
		 * @param Eigen::VectorXd& Joint forces
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration with respect to a
		 * gravity field
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::BodySelector& Bodies that are constrained to be
		 * in contact
		 */
		void computeContactForces(rbd::BodyVector6d& contact_forces,
								  Eigen::VectorXd& joint_forces,
								  const rbd::Vector6d& base_pos,
								  const Eigen::VectorXd& joint_pos,
								  const rbd::Vector6d& base_vel,
								  const Eigen::VectorXd& joint_vel,
								  rbd::Vector6d& base_acc,
								  Eigen::VectorXd& joint_acc,
								  const rbd::BodySelector& contacts);

		/**
		 * @brief Computes the contact forces by comparing the estimated joint
		 * forces with the measured of the joint forces in a selected set of
		 * end-effectors
		 * @param rbd::BodyWrench& Contact forces applied to the defined set of
		 * contacts
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration with respect to a
		 * gravity field
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const Eigen::VectorXd& Joint forces
		 * @param const rbd::BodySelector& Selected set of end-effectors (bodies)
		 */
		void estimateContactForces(rbd::BodyVector6d& contact_forces,
								   const rbd::Vector6d& base_pos,
								   const Eigen::VectorXd& joint_pos,
								   const rbd::Vector6d& base_vel,
								   const Eigen::VectorXd& joint_vel,
								   const rbd::Vector6d& base_acc,
								   const Eigen::VectorXd& joint_acc,
								   const Eigen::VectorXd& joint_forces,
								   const rbd::BodySelector& contacts);

		/**
		 * @brief Computes the center of pressure position given the ground
		 * reactive forces and positions
		 * @param Eigen::Vector3d& Center of pressure position
		 * @param const rbd::BodyWrench& Contact forces
		 * @param const rbd::BodyVector& Contact position
		 */
		void computeCenterOfPressure(Eigen::Vector3d& cop_pos,
									 const rbd::BodyVector6d& contact_for,
									 const rbd::BodyVectorXd& contact_pos);

		/**
		 * @brief Computes the zero momento point using the inverted pendulum model
		 * @param const Eigen::Vector3d& Center of mass position
		 * @param const Eigen::Vector3d& Center of mass acceleration
		 * @param const double& Pendulum height
		 */
		void computeZeroMomentPoint(Eigen::Vector3d& zmp_pos,
									const Eigen::Vector3d& com_pos,
									const Eigen::Vector3d& com_acc,
									const double& height);

		/**
		 * @brief Computes the instantaneous capture point position assuming an
		 * linear inverted pendulum
		 * @param Eigen::Vector3d& Instantaneous capture point position
		 * @param const Eigen::Vector3d& CoM position
		 * @param const Eigen::Vector3d& CoM velocity
		 * @param const double& Pendulum height
		 */
		void computeInstantaneousCapturePoint(Eigen::Vector3d& icp_pos,
				                              const Eigen::Vector3d& com_pos,
											  const Eigen::Vector3d& com_vel,
											  const double& height);

		/**
		 * @brief Computes the centroidal moment pivot position given the
		 * contact forces (i.e. GRFs)
		 * @param Eigen::Vector3d& Centroidal moment pivot position
		 * @param const Eigen::Vector3d& CoM position
		 * @param const double& Pendulum height
		 * @param const rbd::BodyVector6d& Contact forces
		 */
		void computeCentroidalMomentPivot(Eigen::Vector3d& cmp_pos,
				                          const Eigen::Vector3d& com_pos,
				                          const double& height,
				                          const rbd::BodyVector6d& contact_for);

		/**
		 * @brief Computes the CoM torque given the CoP and CMP positions
		 * @param Eigen::Vector3d& CoM torque
		 * @param const Eigen::Vector3d& CoP position
		 * @param const Eigen::Vector3d& CMP position
		 * @param const rbd::BodyVector6d& Contact forces
		 */
		void computeCoMTorque(Eigen::Vector3d& torque,
							  const Eigen::Vector3d& cop_pos,
							  const Eigen::Vector3d& cmp_pos,
							  const rbd::BodyVector6d& contact_for);


		/**
		 * @brief Computes the equivalent contact forces from a center of
		 * pressure position
		 * @param rbd::BodyWrench& contact_for
		 * @param const Eigen::Vector3d& cop_pos
		 * @param const rbd::BodyPosition& contact_pos
		 * @param const rbd::BodySelector& Selected set of active contact
		 */
		void estimateGroundReactionForces(rbd::BodyVector6d& contact_for,
										  const Eigen::Vector3d& cop_pos,
										  const rbd::BodyVector3d& contact_pos,
										  const rbd::BodySelector& ground_contacts);

		/**
		 * @brief Estimates active contacts by comparing the estimated joint
		 * forces with the measured joint forces in a selected set of end-effectors
		 * @param rbd::BodySelector& Estimated active contacts
		 * @param rbd::BodyWrench& Contact forces
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration with respect to a
		 * gravity field
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const Eigen::VectorXd& Joint forces
		 * @param const rbd::BodySelector& Selected set of end-effectors (bodies)
		 * @param double Force threshold
		 */
		void estimateActiveContactsAndForces(rbd::BodySelector& active_contacts,
											 rbd::BodyVector6d& contact_forces,
											 const rbd::Vector6d& base_pos,
											 const Eigen::VectorXd& joint_pos,
											 const rbd::Vector6d& base_vel,
											 const Eigen::VectorXd& joint_vel,
											 const rbd::Vector6d& base_acc,
											 const Eigen::VectorXd& joint_acc,
											 const Eigen::VectorXd& joint_forces,
											 const rbd::BodySelector& contacts,
											 double force_threshold);

		/**
		 * @brief Estimates active contacts by comparing the estimated joint
		 * forces with the measured joint forces in a selected set of end-effectors
		 * @param rbd::BodySelector& Estimated active contacts
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration with respect to a
		 * gravity field
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const Eigen::VectorXd& Joint forces
		 * @param const rbd::BodySelector& Selected set of end-effectors (bodies)
		 * @param double Force threshold
		 */
		void estimateActiveContacts(rbd::BodySelector& active_contacts,
									const rbd::Vector6d& base_pos,
									const Eigen::VectorXd& joint_pos,
									const rbd::Vector6d& base_vel,
									const Eigen::VectorXd& joint_vel,
									const rbd::Vector6d& base_acc,
									const Eigen::VectorXd& joint_acc,
									const Eigen::VectorXd& joint_forces,
									const rbd::BodySelector& contacts,
									double force_threshold);

		/** @brief Gets the floating-base system information */
		const FloatingBaseSystem& getFloatingBaseSystem() const;

		/** @brief Gets the whole-body kinematics */
		const WholeBodyKinematics& getWholeBodyKinematics() const;

		/**
		 * @brief Detects the active contacts
		 * @param rbd::BodySelector& Detected active contacts
		 * @param rbd::BodyWrench& Contact forces
		 * @param double Force threshold
		 */
		void getActiveContacts(rbd::BodySelector& active_contacs,
							   const rbd::BodyVector6d& contact_forces,
							   double force_threshold);


	private:
		/**
		 * @brief Converts the applied external forces to RBDL format
		 * @param std::vector<RigidBodyDynamcis::Math::SpatialVector>& RBDL
		 * external forces format
		 * @param const rbd::BodyWrench& External forces
		 * @param const Eigen::VectorXd& Generalized joint position
		 */
		void convertAppliedExternalForces(std::vector<RigidBodyDynamics::Math::SpatialVector>& f_ext,
										  const rbd::BodyVector6d& ext_force,
										  const Eigen::VectorXd& generalized_joint_pos);

		/**
		 * @brief Computes a consistent acceleration for a defined constrained
		 * contact
		 * @param rbd::Vector6d& Consistent base acceleration
		 * @param Eigen::VectorXd& Consistent joint acceleration
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration with respect to a
		 * gravity field
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::BodySelector& Bodies that are constrained to be
		 * in contact
		 */
		void computeConstrainedConsistentAcceleration(rbd::Vector6d& base_feas_acc,
													  Eigen::VectorXd& joint_feas_acc,
													  const rbd::Vector6d& base_pos,
													  const Eigen::VectorXd& joint_pos,
													  const rbd::Vector6d& base_vel,
													  const Eigen::VectorXd& joint_vel,
													  const rbd::Vector6d& base_acc,
													  const Eigen::VectorXd& joint_acc,
													  const rbd::BodySelector& contacts);

		/* @brief Body ids */
		rbd::BodyID body_id_;

		/** @brief Kinematic model */
		WholeBodyKinematics kinematics_;

		/** @brief A floating-base system information */
		FloatingBaseSystem system_;

		/** @brief Gravitational wrench */
		rbd::Vector6d grav_wrench_;

		/** @brief The joint-space inertial matrix of the system */
		Eigen::MatrixXd joint_inertia_mat_;

		/** @brief The centroidal inertia matrix */
		rbd::Matrix6d com_inertia_mat_;
};

} //@namespace model
} //@namespace dwl

#endif
