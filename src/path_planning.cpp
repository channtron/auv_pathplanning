// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <auv_pathPlanning/pose_node.hpp>
#include <auv_pathPlanning/a_star.h>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
// #include <octomap_msgs/msg/octomap_binary.h>
#include <conversions.h>

using namespace std::chrono_literals;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;

using octomap_msgs::msg::Octomap;
    

class PathPlanning : public rclcpp::Node
{
public:
    PathPlanning(rclcpp::NodeOptions options) : Node("path_planner", options)
    {
        // init whatever is needed for your node


        // init subscribers
        subscriber = create_subscription<PoseStamped>(
            "path_planning_goal",    // which topic
            10,         // QoS            
            // std::bind(&PathPlanning::path_planning,this, std::placeholders::_1));
            [this](PoseStamped::UniquePtr msg)    // callback are perfect for lambdas
            {
                last_goal = *msg;
                last_goal.pose.position.set__z(-5.0);
                path_planning();
            });

        // init subscribers
        odomSubscriber = create_subscription<Pose>(
            "/bluerov2/pose_gt",    // which topic
            10,         // QoS            
            [this](Pose::UniquePtr msg)    // callback are perfect for lambdas
            {
                last_odom = *msg;
                current = auvNode(last_odom);   
            });

        
        octoSubscriber = create_subscription<Octomap>(
            "/bluerov2/octomap_full",    // which topic
            10,         // QoS            
            [this](Octomap::UniquePtr msg)    // callback are perfect for lambdas
            {
                // change for a function update octo in node
                last_octo = *msg; 

                octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
                if (tree) {
                    goal.updateOctree( dynamic_cast<octomap::OcTree *>(tree));
                    std::cout<<"octree updated"<<std::endl;
                    // if (goal.isFree(11.,0.,-5.)) std::cout<<"is free"<<std::endl;
                    // else std::cout<<"is occupied"<<std::endl;
                    // if (goal.imFree()) std::cout<<"is free"<<std::endl;
                    // else std::cout<<"is occupied"<<std::endl;
                    std::cout<<"Occupancy: "<< goal.imFree() <<std::endl;

                } else {
                    RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message");
                } 
            });
            
        // init publishers
        pathPublisher = create_publisher<Path>("computed_path", 10);   // topic + QoS
        
    }   
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<PoseStamped>::SharedPtr subscriber;
    rclcpp::Subscription<Pose>::SharedPtr odomSubscriber;
    PoseStamped last_goal{};
    Pose last_odom{};

    rclcpp::Subscription<Octomap>::SharedPtr octoSubscriber;
    Octomap last_octo; 

    rclcpp::Publisher<Path>::SharedPtr pathPublisher;
    //geometry_msgs::msg::Pose2D pose;
    PoseStamped cmd{};  
    
    std::vector<auvNode> path{};
    auvNode current{};
    auvNode goal{};
    
    void path_planning() // PoseStamped::UniquePtr msg
    {
        // use last_msg to build and publish command
        goal = auvNode(last_goal);
        std::cout << "A* computation starting" << std::endl;
        std::cout << "new path being procesed" << std::endl;
        path = duels::Astar(current, goal);
        path.push_back(goal);

        std::cout << "A* computation finished" << std::endl;
        std::cout << "first waypoint: " << path[1].pose.position.z << std::endl;
        std::cout << "Goal: " << goal.pose.position.z << std::endl;
        std::cout << "last waypoint: " << path.back().pose.position.z << std::endl;

        Path pathCmd{};
        pathCmd.poses[path.size()];
        pathCmd.header.frame_id = "world";
        pathCmd.header.stamp = this->now();
        for (auto const & waypoint : path) pathCmd.poses.push_back(waypoint);
        pathPublisher->publish(pathCmd);
    }
    
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<PathPlanning>(options));
  rclcpp::shutdown();
  return 0;
}
// register this plugin
/*
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PathPlanning)
*/