#include <environment/TerrainMap.h>


namespace dwl
{

namespace environment
{

TerrainMap::TerrainMap() :
		space_discretization_(0.04, 0.04, M_PI / 200),
		obstacle_discretization_(0.04, 0.04, M_PI / 200),
		average_cost_(0.), max_cost_(0.),
		min_height_(std::numeric_limits<double>::max()),
		terrain_information_(false), obstacle_information_(false),
		obstacle_resolution_(0.04)
{
	// Setting up the default values of the cell
	 // TODO compute the default height from robot state
	space_discretization_.coordToKey(default_cell_.key.z, 0., false);
	default_cell_.cost = std::numeric_limits<double>::max();
	default_cell_.normal = Eigen::Vector3d::UnitZ();
}


TerrainMap::~TerrainMap()
{

}


void TerrainMap::reset()
{
	terrain_map_.clear();
	terrain_heightmap_.clear();
}


void TerrainMap::setTerrainMap(const TerrainData& terrain_map)
{
	// Cleaning the old information
	TerrainDataMap empty_terrain_cost_map;
	terrain_map_.swap(empty_terrain_cost_map);
	average_cost_ = 0.;

	// Storing the terrain data according the vertex id
	Vertex vertex_2d;
	unsigned int num_cells = terrain_map.data.size();
	if (num_cells != 0) {
		// Setting the resolution
		setResolution(terrain_map.plane_size, true);
		setResolution(terrain_map.height_size, false);

		for (unsigned int i = 0; i < num_cells; i++) {
			// Building a cost-map for a every 3d vertex
			space_discretization_.keyToVertex(vertex_2d, terrain_map.data[i].key, true);
			double cost_value = terrain_map.data[i].cost;
			terrain_map_[vertex_2d] = terrain_map.data[i];

			// Setting up the maximum cost value
			if (cost_value > max_cost_)
				max_cost_ = cost_value;

			average_cost_ += cost_value;
		}

		// Computing the average cost of the terrain
		average_cost_ /= num_cells;

		// Setting up the values of the default cell. Note that these values
		// are used for unperceived cells
		default_cell_.cost = max_cost_;
		 // TODO compute the default height from robot state
		space_discretization_.coordToKey(default_cell_.key.z, 0., false);

		terrain_information_ = true;
	}
}


void TerrainMap::setTerrainMap(const TerrainDataMap& map)
{
	terrain_map_ = map;
}


void TerrainMap::setObstacleMap(const std::vector<Cell>& obstacle_map)
{
	// Cleaning the old information
	ObstacleMap empty_terrain_obstacle_map;
	obstaclemap_.swap(empty_terrain_obstacle_map);

	//Storing the obstacle-map data according the vertex id
	Vertex vertex_2d;
	if (obstacle_map.size() != 0) {
		// Setting the obstacle resolution
		obstacle_resolution_ = obstacle_map[0].plane_size;
		setObstacleResolution(obstacle_resolution_, true);
		setObstacleResolution(obstacle_resolution_, false);

		for (unsigned int i = 0; i < obstacle_map.size(); i++) {
			// Building a cost map for a every 3d vertex
			obstacle_discretization_.keyToVertex(vertex_2d, obstacle_map[i].key, true);
			obstaclemap_[vertex_2d] = true;
		}

		obstacle_information_ = true;
	}
}


void TerrainMap::setTerrainCell(TerrainCell& cell,
								double cost,
								double height,
								const Terrain& terrain_info)
{
	space_discretization_.coordToKeyChecked(cell.key, terrain_info.position);
	cell.cost = cost;
	cell.height = height;
	cell.normal = terrain_info.surface_normal;
}


void TerrainMap::addCellToTerrainMap(const TerrainCell& cell)
{
	Vertex vertex_id;
	space_discretization_.keyToVertex(vertex_id, cell.key, true);
	terrain_map_[vertex_id] = cell;
}


void TerrainMap::removeCellToTerrainMap(const Vertex& cell_vertex)
{
	terrain_map_.erase(cell_vertex);
}


void TerrainMap::addCellToTerrainHeightMap(const Vertex& cell_vertex,
										   double height)
{
	terrain_heightmap_[cell_vertex] = height;

	if (height < min_height_)
		min_height_ = height;
}


void TerrainMap::removeCellToTerrainHeightMap(const Vertex& cell_vertex)
{
	terrain_heightmap_.erase(cell_vertex);
}


double TerrainMap::getResolution(bool plane)
{
	return space_discretization_.getEnvironmentResolution(plane);
}


double TerrainMap::getObstacleResolution()
{
	return obstacle_resolution_;
}


void TerrainMap::setResolution(double resolution,
							   bool plane)
{
	space_discretization_.setEnvironmentResolution(resolution, plane);
}


void TerrainMap::setObstacleResolution(double resolution,
									   bool plane)
{
	obstacle_discretization_.setEnvironmentResolution(resolution, plane);
}


void TerrainMap::setStateResolution(double position_resolution,
									double angular_resolution)
{
	space_discretization_.setStateResolution(position_resolution,
											   angular_resolution);
	obstacle_discretization_.setStateResolution(position_resolution,
												angular_resolution);
}


const TerrainDataMap& TerrainMap::getTerrainDataMap() const
{
	return terrain_map_;
}


const HeightMap& TerrainMap::getTerrainHeightMap() const
{
	return terrain_heightmap_;
}


const ObstacleMap& TerrainMap::getObstacleMap() const
{
	return obstaclemap_;
}


const TerrainCell& TerrainMap::getTerrainData(const Vertex& vertex) const
{
	TerrainDataMap::const_iterator cell_it = terrain_map_.find(vertex);
	if (cell_it != terrain_map_.end())
		return cell_it->second;
	else
		return default_cell_;
}


const TerrainCell& TerrainMap::getTerrainData(const Eigen::Vector2d& position) const
{
	// Converting the position to a vertex
	Vertex vertex;
	space_discretization_.coordToVertex(vertex, position);

	return getTerrainData(vertex);
}


bool TerrainMap::getTerrainData(TerrainCell& cell,
								const Vertex& vertex) const
{
	TerrainDataMap::const_iterator cell_it = terrain_map_.find(vertex);
	if (cell_it != terrain_map_.end()) {
		cell = cell_it->second;

		space_discretization_.keyToCoord(cell.height, cell.key.z, false);
		return true;
	} else {
		cell = default_cell_;
		return false;
	}
}


bool TerrainMap::getTerrainData(TerrainCell& cell,
								const Eigen::Vector2d& position) const
{
	// Converting the position to a vertex
	Vertex vertex;
	space_discretization_.coordToVertex(vertex, position);

	return getTerrainData(cell, vertex);
}


double TerrainMap::getTerrainHeight(const Vertex& vertex) const
{
	double height;
	Key key = getTerrainData(vertex).key;
	space_discretization_.keyToCoord(height, key.z, false);

	return height;
}


double TerrainMap::getTerrainHeight(const Eigen::Vector2d& position) const
{
	// Converting the position to a vertex
	Vertex vertex;
	space_discretization_.coordToVertex(vertex, position);

	return getTerrainHeight(vertex);
}


bool TerrainMap::getTerrainHeight(double& height,
								  const Vertex& vertex) const
{
	TerrainCell cell;
	bool data = getTerrainData(cell, vertex);
	Key key = cell.key;
	space_discretization_.keyToCoord(height, key.z, false);

	return data;
}


bool TerrainMap::getTerrainHeight(double& height,
								  const Eigen::Vector2d& position) const
{
	// Converting the position to a vertex
	Vertex vertex;
	space_discretization_.coordToVertex(vertex, position);

	return getTerrainHeight(height, vertex);
}


const Weight& TerrainMap::getTerrainCost(const Vertex& vertex) const
{
	return getTerrainData(vertex).cost;
}


const Weight& TerrainMap::getTerrainCost(const Eigen::Vector2d& position) const
{
	// Converting the position to a vertex
	Vertex vertex;
	space_discretization_.coordToVertex(vertex, position);

	return getTerrainCost(vertex);
}


bool TerrainMap::getTerrainCost(Weight& cost,
								const Vertex& vertex) const
{
	TerrainCell cell;
	bool data = getTerrainData(cell, vertex);
	cost = cell.cost;

	return data;
}


bool TerrainMap::getTerrainCost(Weight& cost,
								const Eigen::Vector2d& position) const
{
	// Converting the position to a vertex
	Vertex vertex;
	space_discretization_.coordToVertex(vertex, position);

	return getTerrainCost(cost, vertex);
}


const Eigen::Vector3d& TerrainMap::getTerrainNormal(const Vertex& vertex) const
{
	return getTerrainData(vertex).normal;
}


const Eigen::Vector3d& TerrainMap::getTerrainNormal(const Eigen::Vector2d& position) const
{
	// Converting the position to a vertex
	Vertex vertex;
	space_discretization_.coordToVertex(vertex, position);

	return getTerrainNormal(vertex);
}


bool TerrainMap::getTerrainNormal(Eigen::Vector3d& normal,
								  const Vertex& vertex) const
{
	TerrainCell cell;
	bool data = getTerrainData(cell, vertex);
	normal = cell.normal;

	return data;
}


bool TerrainMap::getTerrainNormal(Eigen::Vector3d& normal,
								  const Eigen::Vector2d& position) const
{
	// Converting the position to a vertex
	Vertex vertex;
	space_discretization_.coordToVertex(vertex, position);

	return getTerrainNormal(normal, vertex);
}


const SpaceDiscretization& TerrainMap::getTerrainSpaceModel() const
{
	return space_discretization_;
}


const SpaceDiscretization& TerrainMap::getObstacleSpaceModel() const
{
	return obstacle_discretization_;
}


double TerrainMap::getAverageCostOfTerrain()
{
	return average_cost_;
}


bool TerrainMap::isTerrainInformation()
{
	return terrain_information_;
}


bool TerrainMap::isObstacleInformation()
{
	return obstacle_information_;
}

} //@namespace environment
} //@namespace dwl
