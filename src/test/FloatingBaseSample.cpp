#include <model/FloatingBaseSystem.h>
#include <yaml-cpp/yaml.h>
#include <utils/YamlWrapper.h>
using namespace std;



int main(int argc, char **argv)
{

	// Floating-base system object
    dwl::model::FloatingBaseSystem sys;

//	// Resetting the system from the hyq urdf file
////	string model_file = DWL_SOURCE_DIR"/sample/hyq.urdf";
////	string robot_file = DWL_SOURCE_DIR"/config/hyq.yarf";
    string model_file = "/home/hit/catkin_ws_code/src/dwl_ros/src/test/hyq.urdf";
    string robot_file = "/home/hit/catkin_ws_code/src/dwl_ros/src/test/hyq.yarf";
    // FIXME(EricWang): Segmentation fault (core dumped).
    sys.resetFromURDFFile(model_file, robot_file);
    // Creates a human readable overview of the model.
    string model_hierarchy = RigidBodyDynamics::Utils::GetModelHierarchy(sys.getRBDModel());
    cout << "Model hierarchy: " << model_hierarchy << endl;
    // Creates a human readable overview of the Degrees of Freedom.
    string model_dof = RigidBodyDynamics::Utils::GetModelDOFOverview(sys.getRBDModel());
    cout << "Model dof overview: " << model_dof << endl;
    // Creates a human readable overview of the locations of all bodies that have names.
    string namedbody_origin = RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(sys.getRBDModel());
    cout << "Named body origin overview: " << namedbody_origin << endl;
    /*
     * Body Number 0: ROOT
     * Body Number 2147483647: lf_foot
     * Body Number 3: lf_hipassembly
     * Body Number 5: lf_lowerleg
     * Body Number 4: lf_upperleg
     * Body Number 2147483648: lh_foot
     * Body Number 6: lh_hipassembly
     * Body Number 8: lh_lowerleg
     * Body Number 7: lh_upperleg
     * Body Number 2147483649: rf_foot
     * Body Number 9: rf_hipassembly
     * Body Number 11: rf_lowerleg
     * Body Number 10: rf_upperleg
     * Body Number 2147483650: rh_foot
     * Body Number 12: rh_hipassembly
     * Body Number 14: rh_lowerleg
     * Body Number 13: rh_upperleg
     * Body Number 2: trunk
     * Body Number 1: trunk_Translate
    */
    // Getting body id and names.
    RigidBodyDynamics::Model rbdl_model = sys.getRBDModel();
    cout << "Model body size: " << rbdl_model.mBodies.size() << endl;
    for(auto it = rbdl_model.mBodyNameMap.begin(); it != rbdl_model.mBodyNameMap.end(); ++it)
    {
        cout << "Body Number " << it->second << ": " << it->first << endl;
    }

    // Getting the total mass of the system. Note that you could also get the mass of a specific
    // body (e.g. sys.getBodyMass(body_name))
    cout << "Total mass: " << sys.getTotalMass() << endl;
    cout << "Total body size: " << sys.getRBDModel().mBodies.size() << endl; // 15 (12 + 1 + 1 + trunk)
    // Getting every joint type from RBDL function, comparing with Floating system class.
    /*
     * joint type0: 0  added fixed base
     * joint type1: 10 floating base joint translation
     * joint type2: 8  floating base joint orientation
     * joint type3: 5  actuation joint 1
     * joint type4: 5  actuation joint 2
     * joint type5: 5  actuation joint 3
     * joint type6: 5  actuation joint 4
     * joint type7: 5  actuation joint 5
     * joint type8: 5  actuation joint 6
     * joint type9: 5  actuation joint 7
     * joint type10: 5 actuation joint 8
     * joint type11: 5 actuation joint 9
     * joint type12: 5 actuation joint 10
     * joint type13: 5 actuation joint 11
     * joint type14: 5 actuation joint 12
     * joint type15: 1 unknown
     */
    for(int i = 0; i <= sys.getRBDModel().mJoints.size(); i ++)
    {
        cout << "joint type" << i << ": " << sys.getRBDModel().mJoints[i].mJointType << endl;
    }
//    cout << "Joint type: " << sys.getRBDModel().mJoints[1].mJointType << endl;

    // Getting the CoM of the floating-base body. Note that you could also get the CoM of a
    // specific body (e.g. sys.getBodyCoM(body_name))
    cout << "Floating-base CoM: " << sys.getFloatingBaseCoM().transpose() << endl << endl;

    // Getting the number of system DoF, floating-base Dof, joints and end-effectors
    cout << "Total dof: " << sys.getSystemDoF() << endl; // 18
    cout << "Floating-base dof: " << sys.getFloatingBaseDoF() << endl; // 1
    cout << "Number of joint: " << sys.getJointDoF() << endl; // 12
    cout << "Number of end-effectors: " << sys.getNumberOfEndEffectors() << endl << endl; // 4

    // Getting the floating-base information
    for (unsigned int i = 0; i < 6; i++) {
        dwl::rbd::Coords6d base_id = dwl::rbd::Coords6d(i);
        dwl::model::FloatingBaseJoint jnt = sys.getFloatingBaseJoint(base_id);

        if (jnt.active) {
            cout << "Active base joint[" << jnt.id << "] = " << jnt.name << endl;
        }
    }
    cout << endl;

    // Getting the joint names and ids
    dwl::urdf_model::JointID joint_links = sys.getJoints();
    for (dwl::urdf_model::JointID::const_iterator joint_it = joint_links.begin();
            joint_it != joint_links.end(); ++joint_it) {
        string name = joint_it->first;
        unsigned int id = joint_it->second;

        cout << "Joint[" << id << "] = " << name << endl;
    }
    cout << endl;

    // Getting the end-effector names and ids
    dwl::urdf_model::LinkID contact_links = sys.getEndEffectors();
    for (dwl::urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
            contact_it != contact_links.end(); contact_it++) {
        string name = contact_it->first;
        unsigned int id = contact_it->second;

        cout << "End-effector[" << id << "] = " << name << endl;
    }
    cout << endl;

    // Getting the default posture
    Eigen::VectorXd joint_pos0 = sys.getDefaultPosture();

    // Getting the joint limits
    dwl::urdf_model::JointLimits joint_limits = sys.getJointLimits();
    dwl::urdf_model::JointID ids = sys.getJoints();
    for (dwl::urdf_model::JointLimits::iterator jnt_it = joint_limits.begin();
            jnt_it != joint_limits.end(); ++jnt_it) {
        string name = jnt_it->first;
        urdf::JointLimits limits = jnt_it->second;
        unsigned int id = sys.getJointId(name);

        cout << name << "[" << id << "].lower = " << limits.lower << endl;
        cout << name << "[" << id << "].upper = " << limits.upper << endl;
        cout << name << "[" << id << "].velocity = " << limits.velocity << endl;
        cout << name << "[" << id << "].effort = " << limits.effort << endl;
        cout << name << "[" << id << "].pos0 = " << joint_pos0(id) << endl << endl;
    }

    // Setting up the branch states
    dwl::rbd::Vector6d base_pos = dwl::rbd::Vector6d::Zero();
    Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(sys.getJointDoF());
    Eigen::Vector3d lf_branch_pos = Eigen::Vector3d(0.5, 0.75, 1.5);
    sys.setBranchState(joint_pos, lf_branch_pos, "rf_foot");
    cout << "Setting up lf_foot branch position = " << lf_branch_pos.transpose() << endl;
    cout << "Base position = " << base_pos.transpose() << endl;
    cout << "Joint position = " << joint_pos.transpose() << endl;
    cout << "Getting the lf_foot branch position = ";
    cout << sys.getBranchState(joint_pos, "rf_foot").transpose() << endl << endl;

    return 0;
}
