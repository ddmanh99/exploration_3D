#ifndef BEST_FRONTIER_H_
#define BEST_FRONTIER_H_

#include <uav_frontier_exploration_3d/OctomapServer.h>
#include <queue>         // for priority_queue
#include <unordered_map> // for unordered_map to track visited nodes
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <queue>
#include <vector>
#include <chrono>

// #include <global_planner/rrtStarOctomap.h>
// #include <trajectory_planner/polyTrajOctomap.h>

namespace best_frontier
{
  class BestFrontier
  {
  public:
    BestFrontier();
    bool configureFromFile(string config_filename);
    point3d bestFrontierInfGain(
        octomap::OcTree *octree, point3d currentPosition, KeySet &Cells);
    point3d bestFrontierMaxInfGain(
        octomap::OcTree *octree, point3d currentPosition, KeySet &Cells);
    point3d bestFrontierInfGainRRTS(
        octomap::OcTree *octree, point3d currentPosition, KeySet &Cells);
    point3d bestFrontierInDis(
        octomap::OcTree *octree, point3d currentPosition, KeySet &Cells);

    double AStarDistance(const octomap::OcTree *octree, const point3d &p1, const point3d &p2);

  protected:
    double calculateDistance(const point3d p1, const point3d p2)
    {
      double distance = sqrt(
          pow(p2.x() - p1.x(), 2) +
          pow(p2.y() - p1.y(), 2) +
          pow(p2.z() - p1.z(), 2));
      return distance;
    }

    double mahattanDistance(const point3d p1, const point3d p2)
    {
      double distance = std::abs(p1.x() - p2.x()) + std::abs(p1.y() - p2.y()) + std::abs(p1.z() - p2.z());
      return distance;
    }
    double calcMIBox(const octomap::OcTree *octree, const point3d &sensorOrigin);
    void bestPotentialCB(const geometry_msgs::PoseStamped msg);
    void visualCandidateGoal(const geometry_msgs::PoseArray array);

    ros::NodeHandle m_nh;
    ros::Publisher m_potentialPub;
    ros::Publisher m_candidateGoal;
    ros::Subscriber m_bestPotentialSub;

    string m_configFilename;
    string m_mapFile; // for a star
    std::string m_worldFrameId;
    double m_resolution, m_kGain, m_lambda, m_boxInfGainSize;
    ofstream m_logfile;
    string m_filename, m_filePath, m_logPath;
    std::shared_ptr<octomap::OcTree> octree_ptr;
    double m_step = 1.0;
    double m_limit_findPath = m_step;
    double m_timeAStart;
    double m_waitTime;
    octomap::OcTree *octree_rrts;
    bool m_bestPotential_recieve;
    geometry_msgs::PoseStamped bestPotential;
    int m_maxCandidate;
  };
}
#endif