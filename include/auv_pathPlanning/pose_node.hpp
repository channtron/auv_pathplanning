#ifndef BOAT_NODE_H
#define BOAT_NODE_H

#include <rclcpp/clock.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>


class auvNode : public geometry_msgs::msg::PoseStamped
{
  using auvNodePtr = std::unique_ptr<auvNode>;
public:

  auvNode(double _x = 0., double _y = 0., double _z = 0., double _t = 0.)
  {
    header.frame_id = "world";
    rclcpp::Clock Clock{};
    header.stamp = Clock.now();
    pose.position.x = _x;
    pose.position.y = _y;
    pose.position.z = _z;
    pose.orientation.z = 1;
    pose.orientation.w = _t;
  }

  auvNode(const geometry_msgs::msg::Pose &_pose)
  {
    header.frame_id = "world";
    rclcpp::Clock Clock{};
    header.stamp = Clock.now(); 
    pose.position = _pose.position;
    pose.orientation = _pose.orientation;
  }

  auvNode(const geometry_msgs::msg::PoseStamped &_pose)
  {
    header = _pose.header;
    pose.position = _pose.pose.position;
    pose.orientation = _pose.pose.orientation;
  }
/*
  inline void set(const Pose2D &pose)
  {
    x = pose.x;
    y = pose.y;
    pose.orientation = pose.pose.orientation;
  }
*/
  inline bool operator==(const auvNode &other) const
  {
    return pose.position==other.pose.position;
  }

  
  float h(const auvNode &goal) const
  {
    auto dx = goal.pose.position.x - pose.position.x;
    auto dy = goal.pose.position.y - pose.position.y;
    auto dz = goal.pose.position.z - pose.position.z;
    return std::sqrt(std::pow(dx,2)+std::pow(dy,2)+std::pow(dz,2));
  }

  virtual float distToParent() const {return 1.;}

  bool isGoal(const auvNode &goal) const
  {
    const double tolerance = 0.5; // Define a tolerance for goal checking
    return std::abs(pose.position.x - goal.pose.position.x) < tolerance &&
            std::abs(pose.position.y - goal.pose.position.y) < tolerance &&
            std::abs(pose.position.z - goal.pose.position.z) < tolerance;
  }

  std::vector<auvNodePtr> children() const
  {
    
    std::vector<auvNodePtr> out;
    const double displacement = 1.0;
    const double diagDisp1 = displacement * 0.7071;
    // const double diagDisp2 = diagDisp1 * 0.7071;
    std::vector<std::tuple<double,double,double>> movements {
                              std::make_tuple(displacement, 0, 0),
                              std::make_tuple(-displacement, 0, 0),
                              std::make_tuple(0, displacement, 0),
                              std::make_tuple(0, -displacement, 0),
                              std::make_tuple(0, 0, displacement),
                              std::make_tuple(0, 0, -displacement),
                              
                              std::make_tuple(diagDisp1, diagDisp1, 0),
                              std::make_tuple(-diagDisp1, diagDisp1, 0),
                              std::make_tuple(diagDisp1, -diagDisp1, 0),
                              std::make_tuple(-diagDisp1, -diagDisp1, 0),
                              
                              std::make_tuple(diagDisp1, 0, diagDisp1),
                              std::make_tuple(-diagDisp1, 0, diagDisp1),
                              std::make_tuple(diagDisp1, 0, -diagDisp1),
                              std::make_tuple(-diagDisp1, 0, -diagDisp1),
                              
                              std::make_tuple(0, diagDisp1, diagDisp1),
                              std::make_tuple(0, -diagDisp1, diagDisp1),
                              std::make_tuple(0, diagDisp1, -diagDisp1),
                              std::make_tuple(0, -diagDisp1, -diagDisp1)/*,
                              
                              std::make_tuple(diagDisp2, diagDisp2, diagDisp2),
                              std::make_tuple(diagDisp2, -diagDisp2, diagDisp2),
                              std::make_tuple(diagDisp2, diagDisp2, -diagDisp2),
                              std::make_tuple(diagDisp2, -diagDisp2, -diagDisp2),

                              std::make_tuple(-diagDisp2, diagDisp2, diagDisp2),
                              std::make_tuple(-diagDisp2, -diagDisp2, diagDisp2),
                              std::make_tuple(-diagDisp2, -diagDisp2, -diagDisp2),
                              std::make_tuple(-diagDisp2, diagDisp2, -diagDisp2)*/};
  
    // Create child nodes for each direction
    for (const auto &delta : movements)
    {
        auto child = std::make_unique<auvNode>(*this);
        child->pose.position.x += std::get<0>(delta);
        child->pose.position.y += std::get<1>(delta);
        child->pose.position.z += std::get<2>(delta);
        
        tf2::Quaternion q_orig, q_rot, q_new;
        q_orig.setRPY(0.0, 0.0, 0.0);
        q_rot.setRPY(0.0, 0.0, atan2(std::get<1>(delta),std::get<0>(delta))); // -M_PI/2
        q_new = q_orig * q_new;
        // std::cout <<"x: " << q_rot.getX() << " y: " << q_rot.getY() << 
        //           " z: " << q_rot.getZ() << " w: " << q_rot.getW() << std::endl;
        child->pose.orientation.x = q_rot.getX();
        child->pose.orientation.y = q_rot.getY();
        child->pose.orientation.z = q_rot.getZ();
        child->pose.orientation.w = q_rot.getW();
        out.push_back(std::move(child));
    }
    // std::cout << "children generated" << std::endl;
    return out;
  }

  double h_cost;
  double g_cost;
};


#endif // BOAT_NODE_H
