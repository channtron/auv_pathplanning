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

using namespace std::chrono_literals;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;
    

class PathPlanning : public rclcpp::Node
{
public:
    PathPlanning(rclcpp::NodeOptions options) : Node("node_name", options)
    {
        // init whatever is needed for your node
        cmd.header.frame_id = "world";
        cmd.pose.position.x = 0.;
        cmd.pose.position.y = 0.;
        cmd.pose.position.z = -5.;
        cmd.pose.orientation.x = 0.;
        cmd.pose.orientation.y = 0.;
        cmd.pose.orientation.z = 0.;
        cmd.pose.orientation.w = 1.;


        // init subscribers
        subscriber = create_subscription<PoseStamped>(
            "/goal_pose",    // which topic
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
            
        // init publishers
        publisher = create_publisher<PoseStamped>("/bluerov2/cmd_pose", 10);   // topic + QoS
        pathPublisher = create_publisher<Path>("Astar_path", 10);   // topic + QoS
      
        // init timer - the function will be called with the given rate
        publish_timer = create_wall_timer(100ms,    // rate
                                          [&]()
                                          { publish_waypoints(); });
        
    }   
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<PoseStamped>::SharedPtr subscriber;
    rclcpp::Subscription<Pose>::SharedPtr odomSubscriber;
    PoseStamped last_goal{};
    Pose last_odom{};
    Pose init_pose{};

    rclcpp::Publisher<PoseStamped>::SharedPtr publisher;
    rclcpp::Publisher<Path>::SharedPtr pathPublisher;
    //geometry_msgs::msg::Pose2D pose;
    PoseStamped cmd{};
    
    rclcpp::TimerBase::SharedPtr publish_timer;    
    
    std::vector<auvNode> path{};
    auvNode current{};
    auvNode goal{};
    
    void path_planning() // PoseStamped::UniquePtr msg
    {


        // use last_msg to build and publish command
        std::cout << "A* computation starting" << std::endl;
        auvNode new_goal{};
        if (last_goal.header.frame_id=="") new_goal = auvNode(0., 0., -5., 0.);
        else new_goal = auvNode(last_goal.pose);
        if (!new_goal.isGoal(goal)) 
        {
            std::cout << "new path being procesed" << std::endl;
            goal = new_goal;
            path = duels::Astar(current, goal);
        }
        
        std::cout << "A* computation finished" << std::endl;
        std::cout << "first waypoint: " << path[1].pose.position.z << std::endl;
        std::cout << "Goal: " << goal.pose.position.z << std::endl;
        std::cout << "last waypoint: " << path.back().pose.position.z << std::endl;

    }

    void publish_waypoints()
    {
        // auto current = auvNode(last_odom);
        if (path.size() > 1) {
            if (current.isGoal(path[1]) && path.size() > 2) 
            {
                path.erase(path.begin());
                std::cout << "Waypoint reached" << std::endl;
            }
            cmd = path[1];

            Path pathCmd{};
            pathCmd.poses[path.size()];
            pathCmd.header.frame_id = "world";
            pathCmd.header.stamp = this->now();
            for (auto const & waypoint : path) pathCmd.poses.push_back(waypoint);
            pathPublisher->publish(pathCmd);
        } else {
            RCLCPP_WARN(this->get_logger(), "Path planning failed or path is too short");
        }
        publisher->publish(cmd);
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