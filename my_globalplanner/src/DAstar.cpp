#include <DAstar.h>
#include <pluginlib/class_list_macros.h>


// register as a plugin
PLUGINLIB_EXPORT_CLASS(my_planner::DAstar, nav_core::BaseGlobalPlanner)

namespace my_planner{
    DAstar::DAstar(){};

    DAstar::DAstar(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros); 
    }

    void DAstar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            costs = costmap_->getCharMap();
            

            frame_id_ = costmap_ros->getGlobalFrameID();
            ros::NodeHandle private_nh("~/" + name);

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            initialized_ = true;
        }
        else
        {
            ROS_WARN("This planner has already been initialized.. doing noting");

        }
    }

    bool DAstar::makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        if(!initialized_){
            ROS_ERROR("The plannner has not been initialized...");
            return false;
            }
        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, 
            goal.pose.position.x, goal.pose.position.y);
        
        // the coordinates of the starting point in the world
        double wx = start.pose.position.x;
        double wy = start.pose.position.y;

        // Convert to coordinates in the map
        unsigned int start_x, start_y;
        costmap_->worldToMap(wx, wy, start_x, start_y);
        int start_index = costmap_->getIndex(start_x, start_y);

        // the coordinates of the ending point in the world
        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        // Convert to coordinates in the map
        unsigned int goal_x, goal_y;
        costmap_->worldToMap(wx, wy, goal_x, goal_y);
        int goal_index = costmap_->getIndex(goal_x, goal_y);

        // queue_ store the grid to be searched in the positive direction
        // dequeue_ store the grid to be searched in the opposite direction
        queue_.clear();
        dequeue_.clear();

        // add starting/ending point to the tail
        queue_.push_back(Node(start_index, 0));
        dequeue_.push_back(Node(goal_index, 0));

        // to store the cost of the current grid to the respective starting point 
        // when searching in someone direction
        std::vector<float> gCost(map_size, MAX_INIT);
        std::vector<float> degCost(map_size, MAX_INIT);

        // Initialized the starting point's cost to 0
        gCost[start_index] = 0;
        degCost[goal_index] = 0;

        // to store the parent point of the current point
        std::vector<int> cameFrom(map_size, -1);
        std::vector<int> decameFrom(map_size, -1);

        plan.clear();
        
        // Loop, search grids in the map
        while(queue_.size() > 0 && dequeue_.size() > 0){
            Node top = queue_[0], detop = dequeue_[0]; // get the minimum cost of point
            std::pop_heap(queue_.begin(), queue_.end(), greater1());
            std::pop_heap(dequeue_.begin(), dequeue_.end(), greater1());
            queue_.pop_back();
            dequeue_.pop_back();

            int i = top.index, de_i = detop.index;
            if(gCost[de_i] < MAX_INIT){ 
                // in the positive direction 
                // searched the grid that has been traveled in the opposite direction
                getPlan1(cameFrom, decameFrom, start_index, goal_index, plan, de_i);
                return true;
            }

            if(degCost[i] < MAX_INIT){
                // in the opposite direction 
                // searched the grid that has been traveled in the positive direction
                getPlan1(cameFrom, decameFrom, start_index, goal_index, plan, i);
                return true;
            }

            if(i == goal_index){
                // in the positive direction, reach the ending
                getPlan(cameFrom, decameFrom, start_index, goal_index, plan);
                return true;
            }

            if(de_i == start_index){
                // in the opposite direction, reach the ending
                getPlan(cameFrom, decameFrom, start_index, goal_index, plan);
                return true;
            }
            // search grid in the positive direction
            addNeigbors(cameFrom, gCost, i, goal_index, 1);

            // search grid in the opposite direction
            addNeigbors(decameFrom, degCost, de_i, start_index, 0);

        }
        return false;
    }

    void DAstar::getPlan1(std::vector<int> camefrom, std::vector<int> decamefrom, int start_i, int goal_i,
            std::vector<geometry_msgs::PoseStamped>& plan, int index){
        
        std::vector<int> camePath;
        int came = index, decame = index;
        while(came != start_i){

            //get indexes of that path has been finded, positive direction
            camePath.push_back(camefrom[came]);
            came = camefrom[came];
        }

        std::reverse(camePath.begin(), camePath.end());
        camePath.push_back(index); // add the coincident index

        while(decame != goal_i){

            // get indexes of that path has been finded, opposite direction
            camePath.push_back(decamefrom[decame]);
            decame = decamefrom[decame];
        }

        plan.clear();
        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < camePath.size(); i++){
            unsigned mx, my;
            costmap_->indexToCells(camePath[i], mx, my);
            double wx, wy;
            costmap_->mapToWorld(mx, my, wx, wy);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            plan.push_back(pose);
        }
        publishPlan(plan);

    }

    void DAstar::getPlan(std::vector<int> camefrom, std::vector<int> decamefrom, int start_i, int goal_i,
            std::vector<geometry_msgs::PoseStamped>& plan){

        std::vector<int> camePath;

        // in positive direction, if it has not reached the ending
        if(camefrom[goal_i] == -1){
            int came = start_i;
            camePath.push_back(came);
            while (came != goal_i)
            {
                // get indexes of that path has been finded, opposite direction
                camePath.push_back(decamefrom[came]);
                came = decamefrom[came];
            } 
        }
        else{
            int came = goal_i;
            while(came != start_i){

                // get indexes of that path has been finded, opposite direction
                camePath.push_back(camefrom[came]);
                came = camefrom[came];
            }
            std::reverse(camePath.begin(), camePath.end());
            // add the index of goal point
            camePath.push_back(goal_i);
        }

        plan.clear();
        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < camePath.size(); i++){
            unsigned mx, my;
            costmap_->indexToCells(camePath[i], mx, my);
            double wx, wy;
            costmap_->mapToWorld(mx, my, wx, wy);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            plan.push_back(pose);
        }
        publishPlan(plan);

    }

    void DAstar::addNeigbors(std::vector<int>& camefrom, std::vector<float>& gcost, int i, int goal_i, int judge){

        for(int x = -1; x <= 1; x++){
            for(int y = -1; y <= 1; y++){

                if(x == 0 && y == 0) continue;

                unsigned xi, yi;
                costmap_->indexToCells(i, xi, yi);
                int next_x = xi + x;
                int next_y = yi + y;
                int next_i = costmap_->getIndex(next_x, next_y);

                // not in map
                if(next_i < 0 || next_i > map_size) continue;

                // has been searched
                if(gcost[next_i] < MAX_INIT) continue;

                // obstacle
	            int n = (int)costs[next_i];
                if(n >= 243) continue;

                // cost = gcost + hcost
                gcost[next_i] = gcost[i] + getGcost(i, next_i);
                float distance = getHcost(next_i, goal_i);

                if(judge == 1){
                    // in positive
                    queue_.push_back(Node(next_i, gcost[next_i] + distance * 20));
                    std::push_heap(queue_.begin(), queue_.end(), greater1());
                }
                else{ 
                    // in opposite
                    dequeue_.push_back(Node(next_i, gcost[next_i] + distance * 20));
                    std::push_heap(dequeue_.begin(), dequeue_.end(), greater1());
                }
                camefrom[next_i] = i; // add the index of its parent

            }
        }        
    }

    double DAstar::getGcost(int i, int next_i){
        int diff = abs(next_i - i);
        // up、down、left、right points
        if(diff == width || diff == 1) return 1.0;
        return 1.4; 
    }

    double DAstar::getHcost(int next_i, int goal_i){
        int next_x = next_i % width;
        int next_y = next_i / width;
        int goal_x = goal_i % width;
        int goal_y = goal_i / width;
        return abs(goal_x - next_x) + abs(goal_y - goal_x); 
    }

    void DAstar::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized yet, but it is being used");
            return;
        }

        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        for(unsigned int i = 0; i < path.size(); i++){
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }


}
