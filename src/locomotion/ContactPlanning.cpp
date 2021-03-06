#include <locomotion/ContactPlanning.h>


namespace dwl
{

namespace locomotion
{

ContactPlanning::ContactPlanning() : terrain_(NULL), robot_(NULL),
		computation_time_(std::numeric_limits<double>::max()), contact_horizon_(0)
{

}


ContactPlanning::~ContactPlanning()
{

}


void ContactPlanning::reset(robot::Robot* robot,
							environment::TerrainMap* terrain)
{
	printf(BLUE "Setting the robot properties in the %s contact planner \n"
			COLOR_RESET, name_.c_str());
	robot_ = robot;

	printf(BLUE "Setting the terrain information in the %s contact"
			" planner \n" COLOR_RESET, name_.c_str());
	terrain_ = terrain;

	for (int i = 0; i < (int) features_.size(); i++)
		features_[i]->reset(robot);
}


void ContactPlanning::addFeature(environment::Feature* feature)
{
	double weight;
	feature->getWeight(weight);
	printf(GREEN "Adding the %s feature with a weight of %f to the %s contact"
			" planner\n" COLOR_RESET, feature->getName().c_str(), weight,
			name_.c_str());
	features_.push_back(feature);
}


void ContactPlanning::setComputationTime(double computation_time)
{
	printf("Setting the allowed computation time of the contact solver"
			" to %f \n", computation_time);
	computation_time_ = computation_time;
}


void ContactPlanning::setContactHorizon(int horizon)
{
	contact_horizon_ = horizon;
}


std::vector<ContactSearchRegion> ContactPlanning::getContactSearchRegions()
{
	std::vector<ContactSearchRegion> contact_search_regions;
	contact_search_regions = contact_search_regions_;

	contact_search_regions_.clear();

	return contact_search_regions;
}

} //@namespace locomotion
} //@namespace dwl
