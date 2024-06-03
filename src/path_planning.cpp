// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <auv_pathPlanning/pose_node.hpp>
#include <auv_pathPlanning/a_star.h>

using namespace std::chrono_literals;
using geometry_msgs::msg::PoseStamped;


class PathPlanning : public rclcpp::Node
{
public:
    PathPlanning(rclcpp::NodeOptions options) : Node("node_name", options)
    {
        // init whatever is needed for your node
        
        // init subscribers
        subscriber = create_subscription<PoseStamped>(
            "/goal_pose",    // which topic
            10,         // QoS            
            [this](PoseStamped::UniquePtr msg)    // callback are perfect for lambdas
            {
                last_goal = *msg;
                last_goal.pose.position.set__z(-5.0);
            });
            
        // init publishers
        publisher = create_publisher<PoseStamped>("/bluerov2/cmd_pose", 10);   // topic + QoS
      
        // init timer - the function will be called with the given rate
        publish_timer = create_wall_timer(100ms,    // rate
                                          [&](){publisher->publish(last_goal);});
    }   
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<PoseStamped>::SharedPtr subscriber;
    PoseStamped last_goal;

    rclcpp::Publisher<PoseStamped>::SharedPtr publisher;
    //geometry_msgs::msg::Pose2D pose;
    
    rclcpp::TimerBase::SharedPtr publish_timer;    
    
    
    void duplicateArm()
    {
        // use last_msg to build and publish command

        
        
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