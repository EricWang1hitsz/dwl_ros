#include <model/FloatingBaseSystem.h>


namespace dwl
{

namespace model
{

FloatingBaseSystem::FloatingBaseSystem(bool full, unsigned int _num_joints) :
		num_system_joints_(0), num_floating_joints_(6 * full),
		num_joints_(_num_joints), floating_ax_(full), floating_ay_(full),
		floating_az_(full), floating_lx_(full), floating_ly_(full),
		floating_lz_(full), type_of_system_(FixedBase), num_end_effectors_(0),
		num_feet_(0), grav_acc_(0.)
{

}


FloatingBaseSystem::~FloatingBaseSystem()
{

}


void FloatingBaseSystem::resetFromURDFFile(const std::string& urdf_file,
										   const std::string& system_file)
{
	resetFromURDFModel(urdf_model::fileToXml(urdf_file), system_file);
}


void FloatingBaseSystem::resetFromURDFModel(const std::string& urdf_model,
											const std::string& system_file)
{
	// Getting the RBDL model from URDF model
	RigidBodyDynamics::Model rbd;
	RigidBodyDynamics::Addons::URDFReadFromString(urdf_model.c_str(), &rbd, false);
	rbd_model_ = rbd;
	urdf_ = urdf_model;
	yarf_ = system_file;

	// Getting information about the floating-base joints
    urdf_model::JointID floating_joint_names; //typedef std::map<std::string,unsigned int> JointID;
    urdf_model::getJointNames(floating_joint_names, urdf_model, urdf_model::floating); //enum JointType {free = 0, fixed, floating, all};
	num_floating_joints_ = floating_joint_names.size();
    std::cout << "number of floating joints:" << num_floating_joints_ << std::endl;
	urdf_model::JointID floating_joint_motions;
	if (num_floating_joints_ > 0) {
		urdf_model::getFloatingBaseJointMotion(floating_joint_motions, urdf_model);
		for (urdf_model::JointID::iterator jnt_it = floating_joint_motions.begin();
				jnt_it != floating_joint_motions.end(); jnt_it++) {
			std::string joint_name = jnt_it->first;
			unsigned int joint_motion = jnt_it->second;
			unsigned int joint_id = floating_joint_names.find(joint_name)->second;

			// Setting the floating-base joint names
			floating_joint_names_.push_back(joint_name);

			// Setting the floating joint information
			FloatingBaseJoint joint(true, joint_id, joint_name);
			if (joint_motion == 6) {
				setFloatingBaseJoint(joint);
			} else {
				rbd::Coords6d joint_motion_coord = rbd::Coords6d(joint_motion);
				setFloatingBaseJoint(joint, joint_motion_coord);
			}
		}
	}

	// Getting the floating-base body name
	unsigned int base_id = 0;
	if (isFullyFloatingBase())
		base_id = 6;
	else
        base_id = getFloatingBaseDoF(); //return num_floating_joints_;
	floating_body_name_ = rbd_model_.GetBodyName(base_id);

	// Getting the information about the actuated joints
	urdf_model::JointID free_joint_names;
	urdf_model::getJointNames(free_joint_names, urdf_model, urdf_model::free);
	urdf_model::getJointLimits(joint_limits_, urdf_model);
	unsigned int num_free_joints = free_joint_names.size();
    std::cout << "number of free joints:" << num_free_joints << std::endl;
    // TODO(EricWang): Does quadruped have any floating joints?
	num_joints_ = num_free_joints - num_floating_joints_;
    std::cout << "number of joints:" << num_joints_ << std::endl;
	for (urdf_model::JointID::iterator jnt_it = free_joint_names.begin();
			jnt_it != free_joint_names.end(); jnt_it++) {
		std::string joint_name = jnt_it->first;
		unsigned int joint_id = jnt_it->second - num_floating_joints_;

		// Checking if it's a virtual floating-base joint
		if (num_floating_joints_ > 0) {
			if (floating_joint_names.find(joint_name) == floating_joint_names.end()) {
				// Setting the actuated joint information
				Joint joint(joint_id, joint_name);
				setJoint(joint);
			}
		} else if (num_floating_joints_ == 0) {
			// Setting the actuated joint information
			Joint joint(joint_id, joint_name);
			setJoint(joint);
		}
	}

	// Getting the joint name list
	dwl::urdf_model::JointID joints = getJoints();
	for (dwl::urdf_model::JointID::const_iterator joint_it = joints.begin();
			joint_it != joints.end(); joint_it++) {
		std::string joint_name = joint_it->first;
        std::cout << "joint name:" << joint_name << std::endl;
		joint_names_.push_back(joint_name);
	}

	// Getting the floating-base system information
	num_system_joints_ = num_floating_joints_ + num_joints_;
    if (isFullyFloatingBase()) {// real fully floating base systems!
        std::cout << "Fully Floating Base System!!!" << std::endl;
		num_system_joints_ = 6 + num_joints_;
        if (hasFloatingBaseConstraints()) // any of 6 DOF is constrained.
			type_of_system_ = ConstrainedFloatingBase;
		else
			type_of_system_ = FloatingBase;
	} else if (num_floating_joints_ > 0)
		type_of_system_ = VirtualFloatingBase;
	else
		type_of_system_ = FixedBase;

	// Getting the end-effectors information
	urdf_model::getEndEffectors(end_effectors_, urdf_model);

	// Getting the end-effector name list
	for (dwl::urdf_model::LinkID::const_iterator ee_it = end_effectors_.begin();
			ee_it != end_effectors_.end(); ee_it++) {
		std::string name = ee_it->first;
        std::cout << "end effector name:" << name << std::endl;
		end_effector_names_.push_back(name);
	}

	// Resetting the system description
	default_joint_pos_ = Eigen::VectorXd::Zero(num_joints_);
    // FIXME(EricWang): Segmentation fault (core dumped).
//	if (!system_file.empty())
//		resetSystemDescription(system_file);

	// Defining the number of end-effectors
	num_end_effectors_ = end_effectors_.size();
    std::cout << "number of end effectors:" << num_end_effectors_ << std::endl;

    if (num_feet_ == 0) {
        printf(YELLOW "Warning: setting up all the end-effectors are feet\n"
                COLOR_RESET);
        num_feet_ = num_end_effectors_;
        foot_names_ = end_effector_names_;
        feet_ = end_effectors_;
    }

	// Resizing the state vectors
	if (getTypeOfDynamicSystem() == FloatingBase ||
			getTypeOfDynamicSystem() == ConstrainedFloatingBase) {    
		full_state_.resize(6 + getJointDoF());
        std::cout << "FLOATING BASE SYSTEM" << std::endl;
	} else if (getTypeOfDynamicSystem() == VirtualFloatingBase) {
		unsigned int base_dof = getFloatingBaseDoF();
		full_state_.resize(base_dof + getJointDoF());
        std::cout << "VIRTUAL FLOATING BASE SYSTEM" << std::endl;
	} else {
		full_state_.resize(getJointDoF());
	}
	joint_state_.resize(getJointDoF());
    std::cout << "Testn" << std::endl;
	// Getting gravity information
	grav_acc_ = rbd_model_.gravity.norm();
	grav_dir_ = rbd_model_.gravity / grav_acc_;
}


void FloatingBaseSystem::resetSystemDescription(const std::string& filename)
{
	// Yaml reader
	YamlWrapper yaml_reader(filename);

	// Parsing the configuration file
	std::string robot = "robot";
	YamlNamespace robot_ns = {robot};
	YamlNamespace pose_ns = {robot, "default_pose"};

	// Reading and setting up the foot names
	if (yaml_reader.read(foot_names_, "feet", robot_ns)) {
		// Ordering the foot names
		using namespace std::placeholders;
		std::sort(foot_names_.begin(),
				  foot_names_.end(),
				  std::bind(&FloatingBaseSystem::compareString, this, _1, _2));

		// Getting the number of foot
		num_feet_ = foot_names_.size();

		// Adding to the feet to the end-effector lists if it doesn't exist
		for (unsigned int i = 0; i < foot_names_.size(); i++) {
			std::string name = foot_names_[i];
			if (end_effectors_.count(name) == 0) {
				unsigned int id = end_effectors_.size() + 1;
				end_effectors_[name] = id;
				end_effector_names_.push_back(name);
			}
		}
	}

	// Reading the default posture of the robot
	for (unsigned int j = 0; j < num_joints_; j++) {
		std::string name = joint_names_[j];

		double joint_pos = 0.;
		if (yaml_reader.read(joint_pos, name, pose_ns)) {
			default_joint_pos_(j) = joint_pos;
		}
	}
}


void FloatingBaseSystem::setFloatingBaseJoint(const FloatingBaseJoint& joint)
{
	FloatingBaseJoint new_joint = joint;
	new_joint.id = rbd::AX;
	floating_ax_ = new_joint;

	new_joint.id = rbd::AY;
	floating_ay_ = new_joint;

	new_joint.id = rbd::AZ;
	floating_az_ = new_joint;

	new_joint.id = rbd::LX;
	floating_lx_ = new_joint;

	new_joint.id = rbd::LY;
	floating_ly_ = new_joint;

	new_joint.id = rbd::LZ;
	floating_lz_ = new_joint;
}


void FloatingBaseSystem::setFloatingBaseJoint(const FloatingBaseJoint& joint,
											  rbd::Coords6d joint_coord)
{
	if (joint_coord == rbd::AX)
		floating_ax_ = joint;
	else if (joint_coord == rbd::AY)
		floating_ay_ = joint;
	else if (joint_coord == rbd::AZ)
		floating_az_ = joint;
	else if (joint_coord == rbd::LX)
		floating_lx_ = joint;
	else if (joint_coord == rbd::LY)
		floating_ly_ = joint;
	else
		floating_lz_ = joint;
}


void FloatingBaseSystem::setJoint(const Joint& joint)
{
	joints_[joint.name] = joint.id;
}


void FloatingBaseSystem::setFloatingBaseConstraint(rbd::Coords6d joint_id)
{
	if (joint_id == rbd::AX)
		floating_ax_.constrained = true;
	else if (joint_id == rbd::AY)
		floating_ay_.constrained = true;
	else if (joint_id == rbd::AZ)
		floating_az_.constrained = true;
	else if (joint_id == rbd::LX)
		floating_lx_.constrained = true;
	else if (joint_id == rbd::LY)
		floating_ly_.constrained = true;
	else
		floating_lz_.constrained = true;
}


void FloatingBaseSystem::setTypeOfDynamicSystem(enum TypeOfSystem type_of_system)
{
	type_of_system_ = type_of_system;
}


void FloatingBaseSystem::setSystemDoF(unsigned int num_dof)
{
	num_system_joints_ = num_dof;
}


void FloatingBaseSystem::setJointDoF(unsigned int num_joints)
{
	num_joints_ = num_joints;
}


const std::string& FloatingBaseSystem::getURDFModel() const
{
	return urdf_;
}


const std::string& FloatingBaseSystem::getYARFModel() const
{
	return yarf_;
}


RigidBodyDynamics::Model& FloatingBaseSystem::getRBDModel()
{
	return rbd_model_;
}


double FloatingBaseSystem::getTotalMass()
{
	double mass = 0.;
	unsigned int num_bodies = rbd_model_.mBodies.size();
	for (unsigned int i = 0; i < num_bodies; i++)
		mass += rbd_model_.mBodies[i].mMass;

	return mass;
}


const double& FloatingBaseSystem::getBodyMass(const std::string& body_name) const
{
	unsigned int body_id = rbd_model_.GetBodyId(body_name.c_str());
	return rbd_model_.mBodies[body_id].mMass;
}


const Eigen::Vector3d& FloatingBaseSystem::getGravityVector() const
{
	return rbd_model_.gravity;
}


const double& FloatingBaseSystem::getGravityAcceleration() const
{
	return grav_acc_;
}


const Eigen::Vector3d& FloatingBaseSystem::getGravityDirection() const
{
	return grav_dir_;
}

//Eric_wang: RBDL API Changed
//RBDL_DLLAPI void CalcCenterOfMass	(	Model & 	model,
//							const Math::VectorNd & 	q,
//							const Math::VectorNd & 	qdot,
//							const Math::VectorNd * 	qddot,
//							double & 	mass,
//							Math::Vector3d & 	com,
//							Math::Vector3d * 	com_velocity = NULL,
//							Math::Vector3d * 	com_acceleration = NULL,
//							Math::Vector3d * 	angular_momentum = NULL,
//							Math::Vector3d * 	change_of_angular_momentum = NULL,
//							bool 	update_kinematics = true
//							)

 const Eigen::Vector3d& FloatingBaseSystem::getSystemCoM(const rbd::Vector6d& base_pos,
 														const Eigen::VectorXd& joint_pos)
 {
 	Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);
 	Eigen::VectorXd qd = Eigen::VectorXd::Zero(num_system_joints_);

 	double mass;
 	RigidBodyDynamics::Utils::CalcCenterOfMass(rbd_model_,
 											   q, qd, NULL, mass,
 											   com_system_);

 	return com_system_;
 }


 const Eigen::Vector3d& FloatingBaseSystem::getSystemCoMRate(const rbd::Vector6d& base_pos,
 															const Eigen::VectorXd& joint_pos,
 															const rbd::Vector6d& base_vel,
 															const Eigen::VectorXd& joint_vel)
 {
 	Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);
 	Eigen::VectorXd qd = toGeneralizedJointState(base_vel, joint_vel);

 	double mass;
 	RigidBodyDynamics::Utils::CalcCenterOfMass(rbd_model_,
 											   q, qd, NULL, mass,
 											   com_system_, &comd_system_);

 	return comd_system_;
 }
/////////////////////////////////

const Eigen::Vector3d& FloatingBaseSystem::getFloatingBaseCoM() const
{
	unsigned int body_id = rbd_model_.GetBodyId(floating_body_name_.c_str());
	return rbd_model_.mBodies[body_id].mCenterOfMass;
}


const Eigen::Vector3d& FloatingBaseSystem::getBodyCoM(const std::string& body_name) const
{
	unsigned int body_id = rbd_model_.GetBodyId(body_name.c_str());
	return rbd_model_.mBodies[body_id].mCenterOfMass;
}


const unsigned int& FloatingBaseSystem::getSystemDoF() const
{
	return num_system_joints_;
}


const unsigned int& FloatingBaseSystem::getFloatingBaseDoF() const
{
	return num_floating_joints_;
}


const unsigned int& FloatingBaseSystem::getJointDoF() const
{
    return num_joints_; // 12 actual joint.
}


const FloatingBaseJoint& FloatingBaseSystem::getFloatingBaseJoint(rbd::Coords6d joint) const
{
	if (joint == rbd::AX)
		return floating_ax_;
	else if (joint == rbd::AY)
		return floating_ay_;
	else if (joint == rbd::AZ)
		return floating_az_;
	else if (joint == rbd::LX)
		return floating_lx_;
	else if (joint == rbd::LY)
		return floating_ly_;
	else
		return floating_lz_;
}


unsigned int FloatingBaseSystem::getFloatingBaseJointCoordinate(unsigned int id)
{
	if (floating_ax_.active && floating_ax_.id == id)
		return rbd::AX;
	else if (floating_ay_.active && floating_ay_.id == id)
		return rbd::AY;
	else if (floating_az_.active && floating_az_.id == id)
		return rbd::AZ;
	else if (floating_lx_.active && floating_lx_.id == id)
		return rbd::LX;
	else if (floating_ly_.active && floating_ly_.id == id)
		return rbd::LY;
	else if (floating_lz_.active && floating_lz_.id == id)
		return rbd::LZ;
	else {
		printf(RED "ERROR: the %i id doesn't bellow to floating-base joint\n"
				COLOR_RESET, id);
		return 0;
	}
}


const std::string& FloatingBaseSystem::getFloatingBaseName() const
{
	return floating_body_name_;
}


const unsigned int& FloatingBaseSystem::getJointId(const std::string& joint_name) const
{
	return joints_.find(joint_name)->second;
}


const urdf_model::JointID& FloatingBaseSystem::getJoints() const
{
	return joints_;
}


const urdf_model::JointLimits& FloatingBaseSystem::getJointLimits() const
{
	return joint_limits_;
}


const urdf::JointLimits& FloatingBaseSystem::getJointLimit(const std::string& name) const
{
	return joint_limits_.find(name)->second;
}


const double& FloatingBaseSystem::getLowerLimit(const std::string& name) const
{
	return getJointLimit(name).lower;
}


const double& FloatingBaseSystem::getLowerLimit(const urdf::JointLimits& joint) const
{
	return joint.lower;
}


const double& FloatingBaseSystem::getUpperLimit(const std::string& name) const
{
	return getJointLimit(name).upper;
}


const double& FloatingBaseSystem::getUpperLimit(const urdf::JointLimits& joint) const
{
	return joint.upper;
}


const double& FloatingBaseSystem::getVelocityLimit(const std::string& name) const
{
	return getJointLimit(name).velocity;
}


const double& FloatingBaseSystem::getVelocityLimit(const urdf::JointLimits& joint) const
{
	return joint.velocity;
}


const double& FloatingBaseSystem::getEffortLimit(const std::string& name) const
{
	return getJointLimit(name).effort;
}


const double& FloatingBaseSystem::getEffortLimit(const urdf::JointLimits& joint) const
{
	return joint.effort;
}


const rbd::BodySelector& FloatingBaseSystem::getFloatingJointNames() const
{
	return floating_joint_names_;
}


const rbd::BodySelector& FloatingBaseSystem::getJointNames() const
{
	return joint_names_;
}


const std::string& FloatingBaseSystem::getFloatingBaseBody() const
{
	return floating_body_name_;
}


const enum TypeOfSystem& FloatingBaseSystem::getTypeOfDynamicSystem() const
{
	return type_of_system_;
}


const unsigned int& FloatingBaseSystem::getNumberOfEndEffectors(enum TypeOfEndEffector type) const
{
	if (type == ALL)
		return num_end_effectors_;
	else
		return num_feet_;
}


const unsigned int& FloatingBaseSystem::getEndEffectorId(const std::string& contact_name) const
{
	return end_effectors_.find(contact_name)->second;
}


const urdf_model::LinkID& FloatingBaseSystem::getEndEffectors(enum TypeOfEndEffector type) const
{
	if (type == ALL)
		return end_effectors_;
	else
		return feet_;
}


const rbd::BodySelector& FloatingBaseSystem::getEndEffectorNames(enum TypeOfEndEffector type) const
{
	if (type == ALL)
		return end_effector_names_;
	else
		return foot_names_;
}


bool FloatingBaseSystem::isFullyFloatingBase()
{
	if (floating_ax_.active && floating_ay_.active &&
			floating_az_.active	&& floating_lx_.active &&
			floating_ly_.active && floating_lz_.active)
		return true;
	else
		return false;
}


bool FloatingBaseSystem::isVirtualFloatingBaseRobot()
{
	if (type_of_system_ == VirtualFloatingBase)
		return true;
	else
		return false;
}


bool FloatingBaseSystem::isConstrainedFloatingBaseRobot()
{
	if (type_of_system_ == ConstrainedFloatingBase)
		return true;
	else
		return false;
}


bool FloatingBaseSystem::hasFloatingBaseConstraints()
{
	if (floating_ax_.constrained || floating_ay_.constrained ||
			floating_az_.constrained ||	floating_lx_.constrained ||
			floating_ly_.constrained || floating_lz_.constrained)
		return true;
	else
		return false;
}


const Eigen::VectorXd& FloatingBaseSystem::toGeneralizedJointState(const rbd::Vector6d& base_state,
																   const Eigen::VectorXd& joint_state)
{
	// Getting the number of joints
	assert(joint_state.size() == getJointDoF());

	// Note that RBDL defines the floating base state as
	// [linear states, angular states]
	if (getTypeOfDynamicSystem() == FloatingBase ||
			getTypeOfDynamicSystem() == ConstrainedFloatingBase) {
		rbd::Vector6d _base_state = base_state;
		full_state_ << rbd::linearPart(_base_state),
					   rbd::angularPart(_base_state),
					   joint_state;
	} else if (getTypeOfDynamicSystem() == VirtualFloatingBase) {
		unsigned int base_dof = getFloatingBaseDoF();

		Eigen::VectorXd virtual_base(base_dof);
		if (floating_ax_.active)
			virtual_base(floating_ax_.id) = base_state(rbd::AX);
		if (floating_ay_.active)
			virtual_base(floating_ay_.id) = base_state(rbd::AY);
		if (floating_az_.active)
			virtual_base(floating_az_.id) = base_state(rbd::AZ);
		if (floating_lx_.active)
			virtual_base(floating_lx_.id) = base_state(rbd::LX);
		if (floating_ly_.active)
			virtual_base(floating_ly_.id) = base_state(rbd::LY);
		if (floating_lz_.active)
			virtual_base(floating_lz_.id) = base_state(rbd::LZ);

		full_state_ << virtual_base, joint_state;
	} else {
        //! eric_wang: Fixed base.
        full_state_ = joint_state;
	}

	return full_state_;
}


void FloatingBaseSystem::fromGeneralizedJointState(rbd::Vector6d& base_state,
												   Eigen::VectorXd& joint_state,
												   const Eigen::VectorXd& generalized_state)
{
	// Resizing the joint state
	joint_state.resize(getJointDoF());

	// Note that RBDL defines the floating base state as
	// [linear states, angular states]
	if (getTypeOfDynamicSystem() == FloatingBase ||
			getTypeOfDynamicSystem() == ConstrainedFloatingBase) {
		base_state << generalized_state.segment<3>(rbd::LX),
					  generalized_state.segment<3>(rbd::AX);
		joint_state = generalized_state.segment(6, getJointDoF());
	} else if (getTypeOfDynamicSystem() == VirtualFloatingBase) {
		for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
			rbd::Coords6d base_coord = rbd::Coords6d(base_idx);
			FloatingBaseJoint joint = getFloatingBaseJoint(base_coord);

			if (joint.active)
				base_state(base_coord) = generalized_state(joint.id);
		}

		joint_state = generalized_state.segment(getFloatingBaseDoF(),
												getJointDoF());
	} else {
		base_state = rbd::Vector6d::Zero();
		joint_state = generalized_state;
	}
}


void FloatingBaseSystem::setBranchState(Eigen::VectorXd& new_joint_state,// 12 actual joints
										const Eigen::VectorXd& branch_state,
										std::string body_name)
{
	// Getting the branch properties
	unsigned int q_index, num_dof;
	getBranch(q_index, num_dof, body_name);
    std::cout << "branch state size:" << branch_state.size()<< " " << "num dof:" << num_dof << std::endl;
    // TODO(EricWang): q_index has a error.
//    q_index = 36;
	// Removing the base index
//	if (isFullyFloatingBase())
//		q_index -= 6;
//	else
//		q_index -= getFloatingBaseDoF();

//    if (branch_state.size() != num_dof) {
//        printf(RED "FATAL: the branch state dimension is not consistent\n" COLOR_RESET);
//        exit(EXIT_FAILURE);
//    }
//    std::cout << "branch state size:" << branch_state.size()<< " " << "num dof:" << num_dof << std::endl;
    // FIXME(EricWang): Segmentation fault (core dumped).
    std::cout << "q index:" << q_index << std::endl;
    // FIXME(EricWang): q index:4294967290.
	new_joint_state.segment(q_index, num_dof) = branch_state;
    std::cout << "test6" << std::endl;
}


Eigen::VectorXd FloatingBaseSystem::getBranchState(Eigen::VectorXd& joint_state,
												   const std::string& body_name)
{
	// Getting the branch properties
	unsigned int q_index, num_dof;
	getBranch(q_index, num_dof, body_name);

	// Removing the base index
//	if (isFullyFloatingBase())
//		q_index -= 6;
//	else
//		q_index -= getFloatingBaseDoF();

	Eigen::VectorXd branch_state(num_dof);
	branch_state = joint_state.segment(q_index, num_dof);

	return branch_state;
}

void FloatingBaseSystem::getBranch(unsigned int &pos_idx, unsigned int &num_dof, const std::string &body_name)
{
    num_dof = 3;

    if(body_name == "lf_foot")
    {
        pos_idx = 0;
    }
    if(body_name == "lh_foot")
    {
        pos_idx = 3;
    }
    if(body_name == "rf_foot")
    {
        pos_idx = 6;
    }
    if(body_name == "rh_foot")
    {
        pos_idx = 9;
    }
}
//void FloatingBaseSystem::getBranch(unsigned int& pos_idx, //  the beginning joint.
//                                   unsigned int& num_dof, // very impartant!!!!!!!
//                                   const std::string& body_name)//contact name
//{
//	// Getting the body id
//	unsigned int body_id = rbd_model_.GetBodyId(body_name.c_str());
//    std::cout << "body id:" << body_id << std::endl; /// 2147483647

//	// Getting the base joint id. Note that the floating-base starts the
//	// kinematic-tree
//	unsigned int base_id = 0;
//	if (isFullyFloatingBase()) {
//		base_id = 6;
//	} else {
//		base_id = getFloatingBaseDoF();
//	}
//    std::cout << "base_id:" << base_id << std::endl; /// 6
//	// Setting the state values of a specific branch to the joint state
//	unsigned int parent_id = body_id;
//    if (rbd_model_.IsFixedBodyId(body_id)) //Checks whether the body is rigidly attached to another body.
//    {// fixed_body_discriminator - 1 moving body,
//        unsigned int fixed_idx = rbd_model_.fixed_body_discriminator; // Value that is used to discriminate between fixed and movable bodies
//        std::cout << "fixed idx:" << fixed_idx << std::endl; /// 2147483647
//        //! eric_wang: return the parent body id of this fixed body.
////        for(int i = 0; i < rbd_model_.mFixedBodies.size(); i++)
////        {
////            std::cout << "all parent id:" << rbd_model_.mFixedBodies[i].mMovableParent << std::endl;
////        }
//        std::cout << "Branch:fixedbody size:" << rbd_model_.mFixedBodies.size() << std::endl;
//        parent_id = rbd_model_.mFixedBodies[body_id - fixed_idx].mMovableParent;// Id of the movable body that this fixed body is attached to.
////        std::cout << "parent id:" << parent_id << std::endl; /// 5 8 11 14 are four legs' parent body id.
//	}

//	// Adding the branch state to the joint state. Two safety checking are done;
//	// checking that this branch has at least one joint, and checking the size
//	// of the new branch state
//	num_dof = 0;
////    for(int i = 0; i < rbd_model_.lambda.size(); i++)
////    {
////        std::cout << "all parent id:" << rbd_model_.lambda[i] << std::endl;
////    }
////    if (parent_id != base_id) {// The starts of the branch is detected when its parent frame is equals to the base frame.
//////    if (parent_id != pos_idx) {
////        do {
////            // FIXME(EricWang): loop all the time.
//////            std::cout << num_dof << std::endl;
////            pos_idx = rbd_model_.mJoints[parent_id].q_index;
////            std::cout << "position index:" << pos_idx << std::endl; /// parent_id + 3;
////            parent_id = rbd_model_.lambda[parent_id]; // The id of the parents body
////            std::cout << "parent id:" << parent_id << std::endl; ///
////            ++num_dof; // TODO(EricWang): should be 3.
////        } while (parent_id != base_id);
//////        } while (parent_id != pos_idx);
//////        std::cout << num_dof << std::endl;
////    }
//    /////////////
//    std::cout << "mjoint total:" << rbd_model_.mJoints.size() << std::endl;
//    for(int i = 0; i < rbd_model_.mJoints.size(); i ++)
//    {
//        std::cout << "mJoints[" << i << "] q_index:" << rbd_model_.mJoints[i].q_index << std::endl;
//    }
//    ////////////
//    pos_idx = rbd_model_.mJoints[parent_id].q_index;
//    std::cout << "position index:" << pos_idx << std::endl;
//    num_dof = 3;
//    std::cout << "num dof:" << num_dof << std::endl;
//    std::cout << body_name << " " << "check ended" << std::endl;
//}


const Eigen::VectorXd& FloatingBaseSystem::getDefaultPosture() const
{
	return default_joint_pos_;
}


bool FloatingBaseSystem::compareString(std::string a, std::string b)
{
	return a < b;
}

} //@namespace model
} //@namespace dwl
