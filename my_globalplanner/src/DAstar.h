
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <vector>

#define MAX_INIT 1.0e10

namespace my_planner {

    class Node {
    public:
        Node(int a, int b){
            index = a;
            cost = b;
        }
    int index;
    float cost;
    };

    struct greater1
    {
        bool operator()(const Node& a, const Node& b){
            return a.cost > b.cost;
        }
    };

    class DAstar : public nav_core::BaseGlobalPlanner
    {
    public:
        DAstar(/* args */);
        DAstar(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
       * @brief  Initialization function for the DAstarPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
        bool makePlan(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        /**
       * @brief Given double paths in the map, get a plan
       * @param camefrom Store the parent index in the positive direction
       * @param decamefrom Store the parent index in the opposite direction 
       * @param start_i The index that start pose transfromed in map
       * @param goal_i The index that goal pose transfromed in map
       * @param plan The plan... filled by the planner
       * @param index The coincident index in the double paths
       */
        void getPlan1(std::vector<int> camefrom, std::vector<int> decamefrom, int start_i, int goal_i,
            std::vector<geometry_msgs::PoseStamped>& plan, int index);

        /**
       * @brief Given a path in the map, get a plan
       * @param camefrom Store the parent index in the positive direction
       * @param decamefrom Store the parent index in the opposite direction 
       * @param start_i The index that start pose transfromed in map
       * @param goal_i The index that goal pose transfromed in map
       * @param plan The plan... filled by the planner
       */
        void getPlan(std::vector<int> camefrom, std::vector<int> decamefrom, int start_i, int goal_i,
            std::vector<geometry_msgs::PoseStamped>& plan);

        /**
       * @brief Given a node in the map, get its surrounding nodes
       * @param camefrom Store the parent index in someone direction
       * @param i The index of current node
       * @param goal_i The index that goal pose transfromed in map
       * @param judge 1-positive direction; 0-opposite direction
       */
        void addNeigbors(std::vector<int>& camefrom, std::vector<float>& gcost, int i, int goal_i, int judge);

	float getPre_min(std::vector<float> gcost, int i, std::vector<int>& camefrom);

        double getGcost(int i, int next_i);

        double getHcost(int next_i, int goal_i);

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

    public:
        std::vector<Node> queue_, dequeue_; 
        unsigned char lethal_cost_; 
        int width; 
        int height;
        int map_size;
        bool initialized_;
        ros::Publisher plan_pub_;
        std::string frame_id_;
        unsigned char *costs;
        costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
    };


  
}

