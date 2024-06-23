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
    

class Navigator : public rclcpp::Node
{
public:
    Navigator(rclcpp::NodeOptions options) : Node("navigation", options)
    {
        // init whatever is needed for your node
        double displacement = declare_parameter("displacement", 2.);
        double collisionRadius = declare_parameter("radius", 2.);

        goal.setDisplacement(displacement);
        goal.setRadius(collisionRadius);

        cmd.header.frame_id = "world";

        goal = auvNode(0., 0., -5., 0.);
        path.poses.push_back(current);
        path.poses.push_back(goal);

        cmd.pose = goal.pose;


        // init subscribers
        subscriber = create_subscription<PoseStamped>(
            "/goal_pose",    // which topic
            10,         // QoS            
            // std::bind(&Navigator::path_planning,this, std::placeholders::_1));
            [this](PoseStamped::UniquePtr msg)    // callback are perfect for lambdas
            {
                last_goal = *msg;
                last_goal.pose.position.set__z(-5.0);

                auvNode new_goal = auvNode(last_goal.pose);
                if (!new_goal.isGoal(goal)) 
                {
                    RCLCPP_INFO(this->get_logger(), "New goal, path planning computing");
                    goalPublisher->publish(last_goal);
                    pathIsNew = false;
                    goal = auvNode(last_goal.pose);
                }
            });

            octoSubscriber = create_subscription<Octomap>(
            "/bluerov2/octomap_full",    // which topic
            10,         // QoS            
            [this](Octomap::UniquePtr msg)    // callback are perfect for lambdas
            {
                octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
                if (tree) {
                    goal.updateOctree( dynamic_cast<octomap::OcTree *>(tree));

                } else {
                    RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message");
                } 
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

        
        pathSubscriber = create_subscription<Path>(
            "computed_path", 
            10,         // QoS            
            [this](Path::UniquePtr msg)    // callback are perfect for lambdas
            {
                path = *msg;
                pathIsNew = true;
            });
            
        // init publishers
        cmdPublisher = create_publisher<PoseStamped>("/bluerov2/cmd_pose", 10);   // topic + QoS
        goalPublisher = create_publisher<PoseStamped>("path_planning_goal", 10);   // topic + QoS
      
        // init timer - the function will be called with the given rate
        publish_timer = create_wall_timer(100ms,    // rate
                                          [&]()
                                          { publish_waypoints(); });
        
    }   
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<PoseStamped>::SharedPtr subscriber;
    rclcpp::Subscription<Pose>::SharedPtr odomSubscriber;
    rclcpp::Subscription<Path>::SharedPtr pathSubscriber;
    rclcpp::Subscription<Octomap>::SharedPtr octoSubscriber;
    PoseStamped last_goal{};
    Pose last_odom{};
    Path path{};
    bool pathIsNew{false};

    rclcpp::Publisher<PoseStamped>::SharedPtr cmdPublisher;
    rclcpp::Publisher<PoseStamped>::SharedPtr goalPublisher;
    //geometry_msgs::msg::Pose2D pose;
    PoseStamped cmd{};
    
    rclcpp::TimerBase::SharedPtr publish_timer;

    auvNode current{};
    auvNode goal{};

    void publish_waypoints()
    {
        if (path.poses.size() > 1) {
            // std::cout << "path > 1" << std::endl;
            if (path.poses.size() > 2) 
            {
                auvNode nextNode(path.poses[1]);
                auvNode afterNextNode(path.poses[2]);
                // std::cout << "path > 2" << std::endl;
                if (
                (!nextNode.imFree() && !nextNode.isGoal(goal)) 
                || (!afterNextNode.imFree() && !afterNextNode.isGoal(goal))
                ) 
                {
                    RCLCPP_INFO(this->get_logger(), "Next possition collision detected, replanning path");
                    if (pathIsNew)
                    {
                        goalPublisher->publish(last_goal);
                        pathIsNew = false;
                    }
                }
                // std::cout << "checked if replan" << std::endl;

                if (current.isGoal(nextNode) && afterNextNode.imFree() && pathIsNew)
                {
                    path.poses.erase(path.poses.begin());
                    RCLCPP_INFO(this->get_logger(), "Waypoint reached");
                }
                // std::cout << "checked if waypoint" << std::endl;
            }
            cmd = path.poses[1];
        } else {
            RCLCPP_WARN(this->get_logger(), "Path planning failed or path is too short");
        }
        cmdPublisher->publish(cmd);
    }
    
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Navigator>(options));
  rclcpp::shutdown();
  return 0;
}
// register this plugin
/*
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PathPlanning)
*/