#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OccupancyOcTree.h>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <functional>

// Node structure for A* algorithm
struct Node
{
    int x, y, z;
    float g_cost, h_cost, f_cost;
    Node *parent;

    Node(int x, int y, int z) : x(x), y(y), z(z), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}

    float calculateHCost(Node *end)
    {
        return std::sqrt(std::pow(x - end->x, 2) + std::pow(y - end->y, 2) + std::pow(z - end->z, 2));
    }

    bool operator>(const Node &other) const
    {
        return f_cost > other.f_cost;
    }

    bool operator==(const Node &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    struct Hash
    {
        size_t operator()(const Node *n) const
        {
            return std::hash<int>()(n->x) ^ std::hash<int>()(n->y) ^ std::hash<int>()(n->z);
        }
    };
};

// Octomap to grid conversion
void convertOctomapToGrid(const octomap::OcTree *ocTree, std::vector<std::vector<std::vector<bool>>> &grid)
{
    for (octomap::OcTree::iterator it = ocTree->begin_tree(), end = ocTree->end_tree(); it != end; ++it)
    {
        if (ocTree->isNodeOccupied(*it))
        {
            octomap::point3d p = it.getCoordinate();
            int x = static_cast<int>(p.x());
            int y = static_cast<int>(p.y());
            int z = static_cast<int>(p.z());
            if (x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && z >= 0 && z < grid[0][0].size())
            {
                grid[x][y][z] = true;
            }
        }
    }
}

// A* algorithm to find path
std::vector<Node *> findPath(Node *start, Node *end, const std::vector<std::vector<std::vector<bool>>> &grid)
{
    std::priority_queue<Node *, std::vector<Node *>, std::greater<>> open_list;
    std::unordered_set<Node *, Node::Hash> closed_set;
    std::unordered_set<Node *, Node::Hash> open_set;

    open_list.push(start);
    open_set.insert(start);

    while (!open_list.empty())
    {
        Node *current = open_list.top();
        open_list.pop();
        open_set.erase(current);

        if (*current == *end)
        {
            std::vector<Node *> path;
            Node *temp = current;
            while (temp)
            {
                path.push_back(temp);
                temp = temp->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        closed_set.insert(current);

        // Add neighboring nodes (in 3D)
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    int nx = current->x + dx;
                    int ny = current->y + dy;
                    int nz = current->z + dz;

                    if (nx >= 0 && nx < grid.size() && ny >= 0 && ny < grid[0].size() && nz >= 0 && nz < grid[0][0].size() && !grid[nx][ny][nz])
                    {
                        Node *neighbor = new Node(nx, ny, nz);
                        if (closed_set.find(neighbor) == closed_set.end())
                        {
                            neighbor->g_cost = current->g_cost + 1.0f;
                            neighbor->h_cost = neighbor->calculateHCost(end);
                            neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                            neighbor->parent = current;
                            if (open_set.find(neighbor) == open_set.end())
                            {
                                open_list.push(neighbor);
                                open_set.insert(neighbor);
                            }
                        }
                    }
                }
            }
        }
    }

    return std::vector<Node *>();
}

// Main class to handle Octomap and path planning
class OctomapPlanner
{
public:
    OctomapPlanner()
    {
        sub_ = nh_.subscribe("octomap_binary", 1, &OctomapPlanner::octomapCallback, this);
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
        octomap::OcTree *ocTree = dynamic_cast<octomap::OcTree *>(tree);

        if (ocTree)
        {
            std::vector<std::vector<std::vector<bool>>> grid(100, std::vector<std::vector<bool>>(100, std::vector<bool>(100, false)));
            convertOctomapToGrid(ocTree, grid);

            Node *start = new Node(0.0, 0.0, 3.0); // Example start position
            Node *end = new Node(5.0, 0.0, 5.0);   // Example end position

            std::vector<Node *> path = findPath(start, end, grid);

            for (Node *node : path)
            {
                std::cout << "Path node: (" << node->x << ", " << node->y << ", " << node->z << ")\n";
            }

            for (Node *node : path)
            {
                delete node;
            }
        }
        else
        {
            ROS_ERROR("Failed to cast to OcTree.");
        }
        delete tree; // Don't forget to delete the tree to avoid memory leaks
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_planner");
    OctomapPlanner planner;
    ros::spin();
    return 0;
}
