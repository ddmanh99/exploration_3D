/*
 * BestFrontier.cpp
 *
 *  Created on: July 14, 2020
 *      Author: Ana Batinovic
 */

#include <uav_frontier_exploration_3d/BestFrontier.h>

std::vector<double> start(3);
std::vector<double> goal(3);
struct Node
{
	octomap::point3d position;
	double g_cost, h_cost;
	Node *parent;

	Node(octomap::point3d pos, double g, double h, Node *p)
		: position(pos), g_cost(g), h_cost(h), parent(p) {}

	double f_cost() const { return g_cost + h_cost; }
};

struct CompareNode
{
	bool operator()(const Node *a, const Node *b)
	{
		return a->f_cost() > b->f_cost();
	}
};

struct ComparePoint3D
{
	bool operator()(const octomap::point3d &a, const octomap::point3d &b) const
	{
		if (a.x() != b.x())
			return a.x() < b.x();
		if (a.y() != b.y())
			return a.y() < b.y();
		return a.z() < b.z();
	}
};

double computePath(Node *goal_node)
{
	Node *current = goal_node;
	double total_distance = 0.0;
	while (current != nullptr)
	{
		if (current->parent != nullptr)
		{
			total_distance += (current->position - current->parent->position).norm();
		}
		current = current->parent;
	}
	return total_distance;
}

namespace best_frontier
{
	BestFrontier::BestFrontier()
	{
		ros::NodeHandle private_nh{ros::NodeHandle("~")};

		// m_logfile.open("/log_best_frontier.txt");
		// m_logfile << "This is a log file for BestFrontier" << endl;

		// Read from yaml file
		private_nh.param("exploration_config_filename", m_configFilename, m_configFilename);
		configureFromFile(m_configFilename);

		m_logPath = m_filePath + m_filename + "_BF_log.txt";
		m_logfile.open(m_logPath);
		m_logfile << "This is a log file for BestFrontier" << endl;

		m_potentialPub = m_nh.advertise<
			geometry_msgs::PoseArray>("exploration/potential", 1, false);

		m_candidateGoal = m_nh.advertise<
			visualization_msgs::MarkerArray>("/candidate_goal", 1, false);

		m_bestPotentialSub = m_nh.subscribe("/best_potential", 1, &BestFrontier::bestPotentialCB, this);
	}

	bool BestFrontier::configureFromFile(string config_filename)
	{
		cout << "BestFrontier - Configuring sensor specifications from file: " << endl;
		cout << "  " << config_filename << endl;

		// Open yaml file with configuration
		YAML::Node config = YAML::LoadFile(config_filename);

		// Get params
		m_resolution = config["octomap"]["resolution"].as<double>();
		m_boxInfGainSize = config["exploration"]["box_length"].as<double>();
		m_kGain = config["exploration"]["k_gain"].as<double>();
		m_lambda = config["exploration"]["lambda"].as<double>();
		m_mapFile = config["octomap"]["map_file"].as<string>();
		m_timeAStart = config["planner"]["time_aStar"].as<double>();
		m_filename = config["octomap"]["filename"].as<string>();
		m_filePath = config["octomap"]["file_path"].as<string>();
		m_waitTime = config["octomap"]["max_waitTime"].as<double>();
		m_maxCandidate = config["octomap"]["max_candidate"].as<int>();
		m_worldFrameId = config["exploration"]["global_frame"].as<string>();

		m_bestPotential_recieve = false;
		// octree_ptr = std::make_shared<octomap::OcTree>(m_mapFile);
		// if (octree_ptr)
		// {
		// 	size_t num_nodes = octree_ptr->calcNumNodes();
		// 	std::cout << "Số lượng node trong OctoMap: " << num_nodes << std::endl;
		// 	ROS_INFO("Successfully loaded OctoMap from file.");
		// }
		// else
		// {
		// 	ROS_ERROR("Failed to load OctoMap from file.");
		// }
		octree_rrts = new octomap::OcTree(m_mapFile);
		return true;
	}

	void BestFrontier::bestPotentialCB(geometry_msgs::PoseStamped msg)
	{
		m_bestPotential_recieve = true;
		bestPotential = msg;
	}

	double BestFrontier::AStarDistance(const octomap::OcTree *octree, const point3d &p1, const point3d &p2)
	{
		octomap::point3d start(round(p1.x()),
							   round(p1.y()),
							   round(p1.z()));
		octomap::point3d goal(round(p2.x()),
							  round(p2.y()),
							  round(p2.z()));

		return 999.0;
	}
	// raw
	point3d BestFrontier::bestFrontierInfGain(
		octomap::OcTree *octree, point3d currentPosition, KeySet &Cells)
	{
		// m_logfile << "bestFrontierInfGain" << endl;
		ros::WallTime startTime_frontier = ros::WallTime::now();
		if (Cells.size() == 0)
		{
			ROS_WARN("BestFrontier - Zero clustered frontiers!");
			return {};
		}
		vector<pair<point3d, point3d>> candidates;

		for (KeySet::iterator iter = Cells.begin(),
							  end = Cells.end();
			 iter != end; ++iter)
		{
			// Get cell position
			point3d tempCellPosition = octree->keyToCoord(*iter);
			double x = tempCellPosition.x();
			double y = tempCellPosition.y();
			double z = tempCellPosition.z();

			candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, 0.0)));
		}

		// If the cluster point is not in octree
		if (candidates.size() == 0)
		{
			ROS_WARN("BestFrontier - Zero candidates.");
			return {};
		}
		// Calculate information gain for every clustered candidate
		std::vector<double> InfGainVector(candidates.size());
		std::cout << "BF: Num of candidates is " << candidates.size() << std::endl;

		// Calculate information gain for every clustered candidate
		// std::vector<std::tuple<int, point3d, double, double>> InfGainData;

		for (int i = 0; i < candidates.size(); i++)
		{
			// Get candidate
			auto currCandidate = candidates[i];
			double unknownVolume = calcMIBox(octree, currCandidate.first);
			double tempDistance = calculateDistance(currentPosition, currCandidate.first);
			// double tempDistance = mahattanDistance(currentPosition, currCandidate.first);
			double kGain = m_kGain;
			InfGainVector[i] = kGain * unknownVolume * exp(-m_lambda * tempDistance);

			// Store the information as a tuple (point3d, unknownVolume, InfGain)
			// InfGainData.push_back(std::make_tuple(i, currCandidate.first, unknownVolume, InfGainVector[i]));

			std::cout << "BF: Candidate " << i << ": (" << currCandidate.first.x() << " " << currCandidate.first.y() << " " << currCandidate.first.z() << ") "
					  << tempDistance << " "
					  << unknownVolume << " "
					  << InfGainVector[i] << std::endl;
		}

		// Find max element index
		int maxElementIndex =
			max_element(InfGainVector.begin(), InfGainVector.end()) - InfGainVector.begin();

		// Define best frontier
		point3d bestFrontier = point3d(
			candidates[maxElementIndex].first.x(),
			candidates[maxElementIndex].first.y(),
			candidates[maxElementIndex].first.z());
		std::cout << "Best frontier point " << maxElementIndex << ": " << bestFrontier << endl;
		// m_logfile << "Best frontier point " << maxElementIndex << ": " << bestFrontier << endl;
		m_logfile << bestFrontier.x() << " " << bestFrontier.y() << " " << bestFrontier.z() << endl;
		double total_time_frontier = (ros::WallTime::now() - startTime_frontier).toSec();
		// m_logfile << "Best frontier used total: " << total_time_frontier << " sec" << endl;

		return bestFrontier;
	}

	// min distance
	point3d BestFrontier::bestFrontierInDis(
		octomap::OcTree *octree, point3d currentPosition, KeySet &Cells)
	{
		// m_logfile << "bestFrontierInfGain" << endl;
		ros::WallTime startTime_frontier = ros::WallTime::now();
		if (Cells.size() == 0)
		{
			ROS_WARN("BestFrontier - Zero clustered frontiers!");
			return {};
		}
		vector<pair<point3d, point3d>> candidates;

		for (KeySet::iterator iter = Cells.begin(),
							  end = Cells.end();
			 iter != end; ++iter)
		{
			// Get cell position
			point3d tempCellPosition = octree->keyToCoord(*iter);
			double x = tempCellPosition.x();
			double y = tempCellPosition.y();
			double z = tempCellPosition.z();

			candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, 0.0)));
		}

		// If the cluster point is not in octree
		if (candidates.size() == 0)
		{
			ROS_WARN("BestFrontier - Zero candidates.");
			return {};
		}
		// Calculate information gain for every clustered candidate
		std::vector<std::pair<point3d, double>> candidateDistances;
		std::vector<double> InfGainVector(candidates.size());
		// std::cout << "BF: Num of candidates is " << candidates.size() << std::endl;

		// Calculate information gain for every clustered candidate
		// std::vector<std::tuple<int, point3d, double, double>> InfGainData;

		for (int i = 0; i < candidates.size(); i++)
		{
			// Get candidate
			auto currCandidate = candidates[i];
			double unknownVolume = calcMIBox(octree, currCandidate.first);
			double tempDistance = calculateDistance(currentPosition, currCandidate.first);
			double kGain = m_kGain;
			InfGainVector[i] = kGain * unknownVolume * exp(-m_lambda * tempDistance);
			// double tempDistance = mahattanDistance(currentPosition, currCandidate.first);
			candidateDistances.push_back(std::make_pair(currCandidate.first, tempDistance));
		}

		// Find max element index
		int maxElementIndex =
			max_element(InfGainVector.begin(), InfGainVector.end()) - InfGainVector.begin();
		// Define best frontier
		point3d bestFrontier_raw = point3d(
			candidates[maxElementIndex].first.x(),
			candidates[maxElementIndex].first.y(),
			candidates[maxElementIndex].first.z());
		m_logfile << bestFrontier_raw.x() << " " << bestFrontier_raw.y() << " " << bestFrontier_raw.z() << endl;

		// std::cout << "BF: Candidate " << i << ": (" << currCandidate.first.x() << " " << currCandidate.first.y() << " " << currCandidate.first.z() << ") "
		// 		  << tempDistance << " "
		// 		  << unknownVolume << " "
		// 		  << InfGainVector[i] << std::endl;
		std::sort(candidateDistances.begin(), candidateDistances.end(),
				  [](const std::pair<point3d, double> &a, const std::pair<point3d, double> &b)
				  {
					  return a.second < b.second;
				  });

		geometry_msgs::PoseArray poseArray;
		poseArray.header.stamp = ros::Time::now();
		poseArray.header.frame_id = "world";
		for (int i = 0; i < std::min(m_maxCandidate, (int)candidateDistances.size()); i++)
		{
			point3d bestFrontier = candidateDistances[i].first;
			double distance = candidateDistances[i].second;
			std::cout << "Candidate " << i << ": (" << bestFrontier.x() << ", " << bestFrontier.y() << ", " << bestFrontier.z() << ") with distance: " << distance << std::endl;
			// m_logfile << "Closest candidate " << i << ": " << bestFrontier.x() << " " << bestFrontier.y() << " " << bestFrontier.z() << " Distance: " << distance << std::endl;
			geometry_msgs::Pose pose;
			pose.position.x = bestFrontier.x();
			pose.position.y = bestFrontier.y();
			pose.position.z = bestFrontier.z();
			// pose.orientation.x = volume;
			pose.orientation.w = 1.0; // Orientation mặc định

			// Thêm điểm vào PoseArray
			poseArray.poses.push_back(pose);
		}

		m_potentialPub.publish(poseArray);
		visualCandidateGoal(poseArray);

		ros::Time startWait = ros::Time::now();
		ros::Duration timeout(m_waitTime);
		while (ros::Time::now() - startWait < timeout)
		{
			ros::spinOnce();
			if (m_bestPotential_recieve)
			{
				point3d bestPotentialFrontier = point3d(bestPotential.pose.position.x, bestPotential.pose.position.y, bestPotential.pose.position.z);
				m_bestPotential_recieve = false;
				m_logfile << bestPotentialFrontier.x() << " " << bestPotentialFrontier.y() << " " << bestPotentialFrontier.z() << endl;
				return bestPotentialFrontier;
			}
		}
		// return bestFrontier;
	}

	point3d BestFrontier::bestFrontierInfGainRRTS(
		octomap::OcTree *octree, point3d currentPosition, KeySet &Cells)
	{
		// m_logfile << "bestFrontierInfGain" << endl;
		ros::WallTime startTime_frontier = ros::WallTime::now();
		if (Cells.size() == 0)
		{
			ROS_WARN("BestFrontier - Zero clustered frontiers!");
			return {};
		}
		vector<pair<point3d, point3d>> candidates;

		for (KeySet::iterator iter = Cells.begin(),
							  end = Cells.end();
			 iter != end; ++iter)
		{
			// Get cell position
			point3d tempCellPosition = octree->keyToCoord(*iter);
			double x = tempCellPosition.x();
			double y = tempCellPosition.y();
			double z = tempCellPosition.z();

			candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, 0.0)));
		}

		// If the cluster point is not in octree
		if (candidates.size() == 0)
		{
			ROS_WARN("BestFrontier - Zero candidates.");
			return {};
		}
		// Calculate information gain for every clustered candidate
		std::vector<double> InfGainVector(candidates.size());
		// std::cout << "BF: Num of candidates is " << candidates.size() << std::endl;

		// Calculate information gain for every clustered candidate
		std::vector<std::tuple<int, point3d, double, double>> InfGainData;

		for (int i = 0; i < candidates.size(); i++)
		{
			// Get candidate
			auto currCandidate = candidates[i];
			double unknownVolume = calcMIBox(octree, currCandidate.first);
			if (unknownVolume < 0.1)
			{
				unknownVolume = 0.0;
			}
			double tempDistance = calculateDistance(currentPosition, currCandidate.first);
			// double tempDistance = mahattanDistance(currentPosition, currCandidate.first);
			double kGain = m_kGain;
			InfGainVector[i] = kGain * unknownVolume * exp(-m_lambda * tempDistance);

			// Store the information as a tuple (point3d, unknownVolume, InfGain)
			InfGainData.push_back(std::make_tuple(i, currCandidate.first, unknownVolume, InfGainVector[i]));

			// std::cout << "BF: Candidate " << i << ": (" << currCandidate.first.x() << " " << currCandidate.first.y() << " " << currCandidate.first.z() << ") "
			// 		  << tempDistance << " "
			// 		  << unknownVolume << " "
			// 		  << InfGainVector[i] << std::endl;
		}

		// Find max element index
		int maxElementIndex =
			max_element(InfGainVector.begin(), InfGainVector.end()) - InfGainVector.begin();

		// Define best frontier
		point3d bestFrontier = point3d(
			candidates[maxElementIndex].first.x(),
			candidates[maxElementIndex].first.y(),
			candidates[maxElementIndex].first.z());
		std::cout << "Best frontier point " << maxElementIndex << ": " << bestFrontier << endl;
		// m_logfile << "Best frontier point " << maxElementIndex << ": " << bestFrontier << endl;
		m_logfile << bestFrontier.x() << " " << bestFrontier.y() << " " << bestFrontier.z() << endl;
		// double total_time_frontier = (ros::WallTime::now() - startTime_frontier).toSec();
		// m_logfile << "Best frontier used total: " << total_time_frontier << " sec" << endl;

		// Sort the vector based on InfGain in descending order
		if (InfGainData.size() == 1)
		{
			point3d bestPotentialFrontier = std::get<1>(InfGainData[0]);
			return bestPotentialFrontier;
		}
		std::sort(InfGainData.begin(), InfGainData.end(), [](const auto &a, const auto &b)
				  {
					  return std::get<3>(a) > std::get<3>(b); // Sort by InfGain (third element in tuple)
				  });
		geometry_msgs::PoseArray poseArray;
		poseArray.header.stamp = ros::Time::now();
		poseArray.header.frame_id = "world";
		for (int i = 0; i < std::min(m_maxCandidate, (int)InfGainData.size()); i++)
		{
			int id = std::get<0>(InfGainData[i]);
			point3d frontier = std::get<1>(InfGainData[i]);
			double volume = std::get<2>(InfGainData[i]);
			double gain = std::get<3>(InfGainData[i]);
			std::cout << "Potential frontier " << id << ": (" << frontier.x() << ", " << frontier.y() << ", " << frontier.z() << ") "
					  << volume << " " << gain << std::endl;
			// Tạo một Pose cho từng điểm
			geometry_msgs::Pose pose;
			pose.position.x = frontier.x();
			pose.position.y = frontier.y();
			pose.position.z = frontier.z();
			pose.orientation.x = volume;
			pose.orientation.w = 1.0; // Orientation mặc định

			// Thêm điểm vào PoseArray
			poseArray.poses.push_back(pose);
		}
		// Publish PoseArray chứa 3 điểm
		m_potentialPub.publish(poseArray);
		visualCandidateGoal(poseArray);

		ros::Time startWait = ros::Time::now();
		ros::Duration timeout(m_waitTime);
		while (ros::Time::now() - startWait < timeout)
		{
			ros::spinOnce();
			if (m_bestPotential_recieve)
			{
				point3d bestPotentialFrontier = point3d(bestPotential.pose.position.x, bestPotential.pose.position.y, bestPotential.pose.position.z);
				m_bestPotential_recieve = false;
				m_logfile << bestPotentialFrontier.x() << " " << bestPotentialFrontier.y() << " " << bestPotentialFrontier.z() << endl;
				return bestPotentialFrontier;
			}
		}
		return bestFrontier;
	}

	//
	point3d BestFrontier::bestFrontierMaxInfGain(
		octomap::OcTree *octree, point3d currentPosition, KeySet &Cells)
	{
		// Start timing
		ros::WallTime startTime_frontier = ros::WallTime::now();

		if (Cells.size() == 0)
		{
			ROS_WARN("BestFrontierMaxInfGain - Zero clustered frontiers!");
			return {};
		}

		vector<pair<point3d, point3d>> candidates;

		// Convert Cells to candidate positions
		for (KeySet::iterator iter = Cells.begin(), end = Cells.end(); iter != end; ++iter)
		{
			point3d tempCellPosition = octree->keyToCoord(*iter);
			candidates.push_back(make_pair(tempCellPosition, point3d(0.0, 0.0, 0.0)));
		}

		if (candidates.size() == 0)
		{
			ROS_WARN("BestFrontierMaxInfGain - Zero candidates.");
			return {};
		}

		std::vector<double> InfGainVector(candidates.size());
		std::cout << "BFMax: Num of candidates is " << candidates.size() << std::endl;

		// Calculate only the information gain (unknownVolume) for each candidate
		for (int i = 0; i < candidates.size(); i++)
		{
			point3d currCandidate = candidates[i].first;
			double unknownVolume = calcMIBox(octree, currCandidate);
			InfGainVector[i] = unknownVolume;

			std::cout << "BFMax: Candidate " << i << ": (" << currCandidate.x() << " "
					  << currCandidate.y() << " " << currCandidate.z() << ") "
					  << unknownVolume << std::endl;
		}

		// Find the index of the candidate with the maximum unknownVolume
		int maxElementIndex = max_element(InfGainVector.begin(), InfGainVector.end()) - InfGainVector.begin();

		// Define the best frontier with the largest unknownVolume
		point3d bestFrontier = candidates[maxElementIndex].first;
		std::cout << "Best frontier point with max unknownVolume " << maxElementIndex << ": "
				  << bestFrontier << std::endl;

		// Log best frontier point
		m_logfile << bestFrontier.x() << " " << bestFrontier.y() << " " << bestFrontier.z() << std::endl;

		double total_time_frontier = (ros::WallTime::now() - startTime_frontier).toSec();
		// Log total time
		m_logfile << "Best frontier (max unknownVolume) used total: " << total_time_frontier << " sec" << std::endl;

		return bestFrontier;
	}

	double BestFrontier::calcMIBox(const octomap::OcTree *octree, const point3d &sensorOrigin)
	{
		ros::WallTime startTime_frontier = ros::WallTime::now();
		// Calculate number of unchanged cells inside a box around candidate point
		// Propotional to number of the unknown cells inside a box

		// Set bounding box
		point3d minPoint, maxPoint;
		double a = m_boxInfGainSize;
		minPoint.x() = sensorOrigin.x() - (a / 2);
		minPoint.y() = sensorOrigin.y() - (a / 2);
		minPoint.z() = sensorOrigin.z() - (a / 2);

		maxPoint.x() = sensorOrigin.x() + (a / 2);
		maxPoint.y() = sensorOrigin.y() + (a / 2);
		maxPoint.z() = sensorOrigin.z() + (a / 2);

		int unknownNum{0};
		int allNum{0};
		for (double ix = minPoint.x(); ix < maxPoint.x(); ix += m_resolution)
		{
			for (double iy = minPoint.y(); iy < maxPoint.y(); iy += m_resolution)
			{
				for (double iz = minPoint.z(); iz < maxPoint.z(); iz += m_resolution)
				{
					allNum++;
					if (!octree->search(ix, iy, iz))
						unknownNum++;
				}
			}
		}
		return (double)unknownNum / (double)allNum;
	}
	bool isSafe(const octomap::OcTree *octree, const octomap::point3d &point, double min_distance)
	{
		for (double x = -min_distance; x <= min_distance; x += octree->getResolution())
		{
			for (double y = -min_distance; y <= min_distance; y += octree->getResolution())
			{
				for (double z = -min_distance; z <= min_distance; z += octree->getResolution())
				{
					octomap::point3d neighbor(point.x() + x, point.y() + y, point.z() + z);
					if (octree->search(neighbor) && octree->isNodeOccupied(octree->search(neighbor)))
					{
						return false; // The point is too close to an obstacle
					}
				}
			}
		}
		return true;
	}

	void BestFrontier::visualCandidateGoal(const geometry_msgs::PoseArray array)
	{
		visualization_msgs::MarkerArray markerArray;
		int id = 0;
		for (const auto &pose : array.poses)
		{
			visualization_msgs::Marker marker;
			marker.header.frame_id = m_worldFrameId;
			marker.header.stamp = ros::Time::now();
			marker.ns = "candidate_goals";
			marker.id = id++;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = pose.position.x;
			marker.pose.position.y = pose.position.y;
			marker.pose.position.z = pose.position.z;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 1.0;
			marker.scale.y = marker.scale.x;
			marker.scale.z = marker.scale.x;
			marker.color.r = 1.0;
			marker.color.g = 0.5;
			marker.color.b = 0.0;
			marker.color.a = 0.9;
			markerArray.markers.push_back(marker);
		}
		m_candidateGoal.publish(markerArray);
	}
}
