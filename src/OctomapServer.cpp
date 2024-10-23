/*
 * OctomapServer.cpp
 *
 *  Created on: July 14, 2020
 *      Author: Ana Batinovic
 */

#include <uav_frontier_exploration_3d/OctomapServer.h>

namespace octomap_server
{
	OctomapServer::OctomapServer()
	{
		ros::NodeHandle private_nh{ros::NodeHandle("~")};

		// Read from yaml file
		private_nh.param("exploration_config_filename", m_configFilename, m_configFilename);
		configureFromFile(m_configFilename);

		m_logPath = m_filePath + m_filename + "_O_log.txt";
		m_logfile.open(m_logPath);

		m_odomPath = m_filePath + m_filename + "_odom_log.txt";
		m_odomfile.open(m_odomPath);

		m_cellfilePath = m_filePath + m_filename + "_O_Cells_log.txt";
		m_cellfile.open(m_cellfilePath);

		// m_logfile.open("log_octomap.txt");
		m_logfile
			<< "This is a log file for OctomapServer." << endl;
		m_startTimeFlag = ros::Time::now();
		// m_odomfile << "This is a log file for odom." << endl;

		// Initialize octomap object and params
		m_octree = new OcTree(m_resolution);
		m_octree->setProbHit(m_probHit);
		m_octree->setProbMiss(m_probMiss);
		m_octree->setClampingThresMin(m_thresMin);
		m_octree->setClampingThresMax(m_thresMax);
		m_treeDepth = m_octree->getTreeDepth();
		m_maxTreeDepth = m_treeDepth;
		// std::cout << "m_maxTreeDepth :" << m_maxTreeDepth << std::endl;

		// Initialize publishers
		m_uavPose = m_nh.advertise<
			visualization_msgs::Marker>("uav_pose", 10, false);
		m_markerOccPub = m_nh.advertise<
			visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, false);
		m_markerFreePub = m_nh.advertise<
			visualization_msgs::MarkerArray>("free_cells_vis_array", 1, false);
		m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, false);
		m_pubVolumes = m_nh.advertise<
			std_msgs::Float64MultiArray>("octomap_volume", 1);

		// Initialize subscribers
		m_pointCloudSub = m_nh.subscribe("cloud_in", 1,
										 &OctomapServer::pointCloudCallback, this);
		m_uavGlobalPoseSub = m_nh.subscribe("odometry", 10,
											&OctomapServer::globalPoseCallback, this);
		m_saveOctomapServer = m_nh.advertiseService(
			"exploration/save_octomap", &OctomapServer::saveOctomapServiceCb, this);
	}

	OctomapServer::~OctomapServer()
	{
		if (m_octree)
		{
			delete m_octree;
			m_octree = NULL;
		}
	}

	bool OctomapServer::configureFromFile(string config_filename)
	{
		cout << "OctomapServer - Configuring uav exploration from file: " << endl;
		cout << "  " << config_filename << endl;

		// Open yaml file with configuration
		YAML::Node config = YAML::LoadFile(config_filename);

		// Get params
		m_worldFrameId = config["exploration"]["global_frame"].as<string>();
		m_baseFrameId = config["exploration"]["base_link_frame"].as<string>();
		m_totalVol = config["exploration"]["volume"].as<double>();

		m_maxRange = config["sensor_model"]["max_range"].as<double>();
		m_probHit = config["sensor_model"]["probability_hit"].as<double>();
		m_probMiss = config["sensor_model"]["probability_miss"].as<double>();
		m_thresMin = config["sensor_model"]["clamping_thres_min"].as<double>();
		m_thresMax = config["sensor_model"]["clamping_thres_max"].as<double>();

		m_resolution = config["octomap"]["resolution"].as<double>();
		m_treeDepth = config["octomap"]["octree_depth"].as<unsigned>();
		m_filename = config["octomap"]["filename"].as<string>();
		m_filePath = config["octomap"]["file_path"].as<string>();

		return true;
	}

	void OctomapServer::pointCloudCallback(
		const sensor_msgs::PointCloud2::ConstPtr &cloud)
	{
		// std::cout << "-----" << std::endl;
		m_currPointCloud = *cloud;
		m_pointCloudReceivedFlag = true;
		runDefault();
		publishVolume();
		// std::cout << "m_pointCloudReceivedFlag " << m_pointCloudReceivedFlag << std::endl;
	}

	void OctomapServer::globalPoseCallback(
		const nav_msgs::Odometry::ConstPtr &msg)
	{
		m_uavCurrentPose = msg->pose.pose;
		m_time = ros::Time::now();
		m_odomfile << ros::Time::now() << " " << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << endl;
		// m_logfile << msg->header.stamp << endl;
		// static tf::TransformBroadcaster br;
		// tf::Transform t_odom_to_world = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
		// 											  tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
		// tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, lidarFrame);
		// br.sendTransform(trans_odom_to_lidar);
		// Tạo marker để hiển thị vị trí robot
		visualization_msgs::Marker marker;
		marker.header.frame_id = "world"; // Hoặc bất kỳ frame nào bạn đang sử dụng
		marker.header.stamp = ros::Time::now();
		marker.ns = "robot_marker";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE; // Hoặc CUBE, ARROW, CYLINDER tùy bạn
		marker.action = visualization_msgs::Marker::ADD;

		// Đặt vị trí marker theo vị trí robot
		marker.pose.position.x = msg->pose.pose.position.x;
		marker.pose.position.y = msg->pose.pose.position.y;
		marker.pose.position.z = msg->pose.pose.position.z;
		marker.pose.orientation = msg->pose.pose.orientation;

		// Đặt kích thước của marker
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;

		// Đặt màu cho marker (RGBA)
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;

		// Xuất marker
		m_uavPose.publish(marker);
	}

	bool OctomapServer::saveOctomapServiceCb(
		std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		string filePath = m_filePath + m_filename + ".binvox.bt";
		cout << "OctomapServer - saving octomap to file: " << filePath << endl;

		if (m_octree == nullptr)
			return false;

		m_octree->writeBinary(filePath);
		ros::Time m_endTimeFlag = ros::Time::now();
		ros::Duration m_totalTimeRun = m_endTimeFlag - m_startTimeFlag;
		std::cout << "-----------------------------------" << std::endl;
		std::cout << "| Total run time is " << m_totalTimeRun.toSec() / 60.0
				  << " |" << std::endl;
		std::cout << "-----------------------------------" << std::endl;
		m_logfile << "--- Total run time is " << m_totalTimeRun << " ---" << endl;
		return true;
	}

	// Function used to get all neighbours of a specific cell x, y, z
	void OctomapServer::genNeighborCoord(float x, float y, float z)
	{
		point3d point(x, y, z);
		// The keys count the number of cells (voxels)
		// from the origin as discrete address of a voxel.
		OcTreeKey key;
		if (!m_octree->coordToKeyChecked(point, key))
		{
			OCTOMAP_ERROR_STR("Error in search: ["
							  << point << "] is out of OcTree bounds!");
			return;
		}
		std::vector<octomap::point3d> neighbor;
		genNeighborCoord(key, neighbor);
	}

	void OctomapServer::genNeighborCoord(
		OcTreeKey start_key, std::vector<octomap::point3d> &occupiedNeighbor)
	{
		occupiedNeighbor.clear();
		OcTreeKey neighbor_key;
		// 26 neighbours
		for (int i = 0; i < 26; i++)
		{
			// Implements a lookup table that allows to
			// computer keys of neighbor cells directly
			m_lut.genNeighborKey(start_key, i, neighbor_key);
			point3d query = m_octree->keyToCoord(neighbor_key);
			occupiedNeighbor.push_back(query);
		}
	}

	void OctomapServer::trackChanges(PCLPointCloudI &changedCells)
	{
		m_logfile << "OctomapServer - trackChanges" << endl;
		KeyBoolMap::const_iterator startPnt = m_octree->changedKeysBegin();
		KeyBoolMap::const_iterator endPnt = m_octree->changedKeysEnd();

		int c = 0;
		for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter)
		{
			c++;
			OcTreeNode *node = m_octree->search(iter->first);

			bool occupied = m_octree->isNodeOccupied(node);

			pcl::PointXYZI pnt;

			pnt.x = m_octree->keyToCoord(iter->first.k[0]);
			pnt.y = m_octree->keyToCoord(iter->first.k[1]);
			pnt.z = m_octree->keyToCoord(iter->first.k[2]);

			if (occupied)
				pnt.intensity = 1000;
			else
				pnt.intensity = -1000;

			changedCells.push_back(pnt);
		}
		m_octree->resetChangeDetection();
	}

	void OctomapServer::insertScan(const PCLPointCloud &cloud)
	{
		ros::WallTime startTime_insert = ros::WallTime::now();
		m_logfile << "OctomapServer - insertScan" << endl;

		// Set sensorOriginAsCurrentUAVPosition
		point3d sensorOrigin(m_uavCurrentPose.position.x,
							 m_uavCurrentPose.position.y, m_uavCurrentPose.position.z);

		if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) ||
			!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
		{
			ROS_ERROR_STREAM("OctomapServer - Could not generate Key for origin " << sensorOrigin);
		}

		KeySet free_cells, occupied_cells;

		for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
		{
			point3d point(it->x, it->y, it->z);
			// Maxrange check
			if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange))
			{
				// Free cells
				if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
				{
					free_cells.insert(m_keyRay.begin(), m_keyRay.end());
				}
				// Occupied endpoint
				OcTreeKey key;
				if (m_octree->coordToKeyChecked(point, key))
				{
					occupied_cells.insert(key);

					updateMinKey(key, m_updateBBXMin);
					updateMaxKey(key, m_updateBBXMax);
				}
			}
			else
			{ // Ray longer than maxrange:;
				point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
				if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay))
				{
					free_cells.insert(m_keyRay.begin(), m_keyRay.end());

					octomap::OcTreeKey endKey;
					if (m_octree->coordToKeyChecked(new_end, endKey))
					{
						updateMinKey(endKey, m_updateBBXMin);
						updateMaxKey(endKey, m_updateBBXMax);
					}
					else
					{
						ROS_ERROR_STREAM("OctomapServer - Could not generate Key for endpoint " << new_end);
					}
				}
			}
		}

		// Mark free cells only if not seen occupied in this cloud
		for (KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
		{
			if (occupied_cells.find(*it) == occupied_cells.end())
				m_octree->updateNode(*it, false);
		}

		// Mark all occupied cells:
		for (KeySet::iterator it = occupied_cells.begin(), end = free_cells.end(); it != end; it++)
		{
			m_octree->updateNode(*it, true);
		}

		octomap::point3d minPt, maxPt;

		minPt = m_octree->keyToCoord(m_updateBBXMin);
		maxPt = m_octree->keyToCoord(m_updateBBXMax);

		double total_time_insert = (ros::WallTime::now() - startTime_insert).toSec();
		m_logfile << "OctomapServer -InsertScan used total " << total_time_insert << " sec" << endl;

		if (m_compressMap)
			m_octree->prune();
		ros::WallTime startTime_evaluation = ros::WallTime::now();
		// Extract frontiers
		// Get changed cells in the current iteration
		m_octree->enableChangeDetection(true);
		m_changedCells.clear();
		trackChanges(m_changedCells);
		// cout << "OctomapServer - changed cells: " << m_changedCells.size() << endl;
	}

	void OctomapServer::runDefault()
	{
		ros::spinOnce();
		tf::StampedTransform sensorToWorldTf;
		PCLPointCloud pc;

		transformAndFilterPointCloud(sensorToWorldTf, pc);
		insertScan(pc);
		publishOccAndFree();
		publishBinaryOctoMap();
	}

	void OctomapServer::transformAndFilterPointCloud(
		tf::StampedTransform &sensorToWorldTf, PCLPointCloud &pc)
	{
		while (!m_pointCloudReceivedFlag)
		{
			ros::spinOnce();
			ros::Duration(0.2).sleep();
		}
		// Input cloud for filtering and ground-detection
		pcl::fromROSMsg(m_currPointCloud, pc);
		// std::cout << "0" << std::endl;

		try
		{
			m_tfListener.lookupTransform(
				m_worldFrameId, m_currPointCloud.header.frame_id,
				m_currPointCloud.header.stamp, sensorToWorldTf);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR_STREAM("Transform error of sensor data: "
							 << ex.what() << ", quitting callback");
			return;
		}
		// std::cout << "Size of m_currPointCloud: " << m_currPointCloud.width * m_currPointCloud.height << std::endl;
		// std::cout << "1" << std::endl;
		Eigen::Matrix4f sensorToWorld;
		pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

		// Set up filter for height range, also removes NANs
		// TODO: set params!
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
		// std::cout << "2" << std::endl;

		// Directly transform to map frame
		pcl::transformPointCloud(pc, pc, sensorToWorld);

		// Filter height range
		pass.setInputCloud(pc.makeShared());
		pass.filter(pc);
		// std::cout << "Size of pc " << pc.size0() << std::endl;
	}

	bool OctomapServer::isSpeckleNode(const OcTreeKey &nKey) const
	{
		OcTreeKey key;
		bool neighborFound = false;
		for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2])
		{
			for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1])
			{
				for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0])
				{
					if (key != nKey)
					{
						OcTreeNode *node = m_octree->search(key);
						if (node && m_octree->isNodeOccupied(node))
						{
							// We have a neighbor => break!
							neighborFound = true;
						}
					}
				}
			}
		}
		return neighborFound;
	}

	void OctomapServer::publishVolume()
	{
		double occVol, freeVol{0};
		double totalVol, leftVol;

		for (OcTree::leaf_iterator it = m_octree->begin_leafs(),
								   end = m_octree->end_leafs();
			 it != end; ++it)
		{
			double voxelSize = m_octree->getNodeSize(m_maxTreeDepth);
			if (m_octree->isNodeOccupied(*it))
				occVol += pow(voxelSize, 3);
			else
				freeVol += pow(voxelSize, 3);
		}
		totalVol = m_totalVol;
		leftVol = totalVol - (occVol + freeVol);

		std_msgs::Float64MultiArray allVolumes;
		allVolumes.data.resize(5);
		allVolumes.data[0] = occVol;
		allVolumes.data[1] = freeVol;
		allVolumes.data[2] = totalVol;
		allVolumes.data[3] = leftVol;
		allVolumes.data[4] = ros::Time::now().toSec();

		m_pubVolumes.publish(allVolumes);
	}

	void OctomapServer::publishOccAndFree()
	{
		int sum_occ = 0;
		int cout = 0;

		ros::WallTime startTime = ros::WallTime::now();
		size_t octomapSize = m_octree->size();
		if (octomapSize <= 1)
		{
			ROS_WARN("Nothing to publish, octree is empty");
			return;
		}

		// Init markers for free space:
		visualization_msgs::MarkerArray freeNodesVis;
		// Each array stores all cubes of a different size, one for each depth level:
		freeNodesVis.markers.resize(m_treeDepth + 1);

		// Init markers:
		visualization_msgs::MarkerArray occupiedNodesVis;
		// Each array stores all cubes of a different size, one for each depth level:
		occupiedNodesVis.markers.resize(m_treeDepth + 1);

		// Traverse all leafs in the tree:
		for (OcTree::iterator it = m_octree->begin(m_maxTreeDepth),
							  end = m_octree->end();
			 it != end; ++it)
		{
			// if (it.getDepth() == 14)
			// {
			// 	cout++;
			// }
			if (m_octree->isNodeOccupied(*it))
			{
				double z = it.getZ();
				if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
				{
					double size = it.getSize();
					double x = it.getX();
					double y = it.getY();

					// Ignore speckles in the map:
					if (m_filterSpeckles && (it.getDepth() == m_treeDepth + 1) &&
						isSpeckleNode(it.getKey()))
					{
						ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
						continue;
					} // else: current octree node is no speckle, send it out

					// Create marker:
					unsigned idx = it.getDepth();
					assert(idx < occupiedNodesVis.markers.size());

					geometry_msgs::Point cubeCenter;
					cubeCenter.x = x;
					cubeCenter.y = y;
					cubeCenter.z = z;
					// sum_occ_cells++;

					occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

					double minX, minY, minZ, maxX, maxY, maxZ;
					m_octree->getMetricMin(minX, minY, minZ);
					m_octree->getMetricMax(maxX, maxY, maxZ);

					double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
					occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
				}
			}
			else
			{
				// Node not occupied => mark as free in 2D map if unknown so far
				double z = it.getZ();
				if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
				{

					double x = it.getX();
					double y = it.getY();

					// Create marker for free space:
					unsigned idx = it.getDepth();
					assert(idx < freeNodesVis.markers.size());

					geometry_msgs::Point cubeCenter;
					cubeCenter.x = x;
					cubeCenter.y = y;
					cubeCenter.z = z;

					freeNodesVis.markers[idx].points.push_back(cubeCenter);
					// sum_free_cells++;
				}
			}
		}
		// m_logfile << "Number of free cells:" << sum_free_cells << endl;
		// m_logfile << "Number of occ cells:" << sum_occ_cells << endl;
		// m_cellfile << m_time << " " << sum_free_cells << " " << sum_occ_cells << endl;
		// std::cout << "Num of depth node 14: " << cout << std::endl;
		// for (size_t idx = 0; idx < occupiedNodesVis.markers.size(); ++idx)
		// {
		// 	size_t pointsCount = occupiedNodesVis.markers[idx].points.size();
		// 	if (pointsCount > 0)
		// 	{
		// 		std::cout << "Marker in occupiedNodesVis [" << idx << "] has " << pointsCount << " points" << std::endl;
		// 	}
		// }

		// for (size_t idx = 0; idx < freeNodesVis.markers.size(); ++idx)
		// {
		// 	size_t pointsCount = freeNodesVis.markers[idx].points.size();
		// 	if (pointsCount > 0)
		// 	{
		// 		std::cout << "Marker in freeNodesVis [" << idx << "] has " << pointsCount << " points" << std::endl;
		// 	}
		// }

		// Finish MarkerArray:
		std_msgs::ColorRGBA colorOcc, colorFree;
		colorOcc.r = 0.0;
		colorOcc.g = 0.0;
		colorOcc.b = 1.0;
		colorOcc.a = 1.0;

		colorFree.r = 0.0;
		colorFree.g = 1.0;
		colorFree.b = 0.0;
		colorFree.a = 0.5;

		unsigned int sum_free_cells = 0;
		unsigned int sum_occ_cells = 0;
		// std::cout << occupiedNodesVis.markers.size() << std::endl;

		for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i)
		{
			// std::cout << "---" << std::endl;
			sum_occ_cells += occupiedNodesVis.markers[i].points.size() * pow(8, (m_treeDepth - i));
			double size = m_octree->getNodeSize(i);
			// std::cout << i << " " << size << std::endl;

			// std::cout << occupiedNodesVis.markers[i].points.size() << " " << occupiedNodesVis.markers[i].points.size() * pow(8, (m_treeDepth - i))
			// 		  << std::endl;

			occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
			occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
			occupiedNodesVis.markers[i].ns = "red";
			occupiedNodesVis.markers[i].id = i;
			occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			occupiedNodesVis.markers[i].scale.x = size;
			occupiedNodesVis.markers[i].scale.y = size;
			occupiedNodesVis.markers[i].scale.z = size;
			// occupiedNodesVis.markers[i].color = colorOcc;

			if (occupiedNodesVis.markers[i].points.size() > 0)
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}
		m_markerOccPub.publish(occupiedNodesVis);

		// Finish FreeMarkerArray:
		for (unsigned i = 0; i < freeNodesVis.markers.size(); ++i)
		{

			sum_free_cells += freeNodesVis.markers[i].points.size() * pow(8, (m_treeDepth - i));
			double size = m_octree->getNodeSize(i);

			freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
			freeNodesVis.markers[i].header.stamp = ros::Time::now();
			freeNodesVis.markers[i].ns = "red";
			freeNodesVis.markers[i].id = i;
			freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			freeNodesVis.markers[i].scale.x = size;
			freeNodesVis.markers[i].scale.y = size;
			freeNodesVis.markers[i].scale.z = size;
			freeNodesVis.markers[i].color = colorFree;

			if (freeNodesVis.markers[i].points.size() > 0)
				freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}
		m_markerFreePub.publish(freeNodesVis);

		// std::cout << sum_free_cells << " " << sum_occ_cells << std::endl;
		m_cellfile << m_time << " " << sum_free_cells << " " << sum_occ_cells << " " << sum_free_cells + sum_occ_cells << std::endl;
	}

	std_msgs::ColorRGBA OctomapServer::heightMapColor(double h)
	{
		std_msgs::ColorRGBA color;
		color.a = 1;
		// blend over HSV-values (more colors)

		double s = 1.0;
		double v = 1.0;

		h -= floor(h);
		h *= 6;
		int i;
		double m, n, f;

		i = floor(h);
		f = h - i;
		if (!(i & 1))
			f = 1 - f; // if i is even
		m = v * (1 - s);
		n = v * (1 - s * f);

		switch (i)
		{
		case 6:
		case 0:
			color.r = v;
			color.g = n;
			color.b = m;
			break;
		case 1:
			color.r = n;
			color.g = v;
			color.b = m;
			break;
		case 2:
			color.r = m;
			color.g = v;
			color.b = n;
			break;
		case 3:
			color.r = m;
			color.g = n;
			color.b = v;
			break;
		case 4:
			color.r = n;
			color.g = m;
			color.b = v;
			break;
		case 5:
			color.r = v;
			color.g = m;
			color.b = n;
			break;
		default:
			color.r = 1;
			color.g = 0.5;
			color.b = 0.5;
			break;
		}

		return color;
	}

	void OctomapServer::publishBinaryOctoMap()
	{
		Octomap map;
		map.header.frame_id = m_worldFrameId;
		map.header.stamp = ros::Time::now();

		if (octomap_msgs::binaryMapToMsg(*m_octree, map))
			m_binaryMapPub.publish(map);
		else
			ROS_ERROR("Error serializing OctoMap");
	}
}