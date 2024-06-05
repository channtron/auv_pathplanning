#ifndef BOAT_NODE_H
#define BOAT_NODE_H

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>


class auvNode : public geometry_msgs::msg::Pose
{
  using auvNodePtr = std::unique_ptr<auvNode>;
public:

  auvNode(double _x = 0., double _y = 0., double _z = 0., double _t = 0.)
  {
    position.x = _x;
    position.y = _y;
    position.z = _z;
    orientation.z = 1;
    orientation.w = _t;
  }

  auvNode(const geometry_msgs::msg::Pose &pose)
  {
    position = pose.position;
    orientation = pose.orientation;
  }
/*
  inline void set(const Pose2D &pose)
  {
    x = pose.x;
    y = pose.y;
    orientation = pose.orientation;
  }
*/
  inline bool operator==(const auvNode &other) const
  {
    return position==other.position;
  }

  
  float h(const auvNode &goal) const
  {
    auto dx = goal.position.x - position.x;
    auto dy = goal.position.y - position.y;
    auto dz = goal.position.z - position.z;
    return std::sqrt(std::pow(dx,2)+std::pow(dy,2)+std::pow(dz,2));
  }

  virtual float distToParent() const {return 1.;}

  bool isGoal(const auvNode &goal) const
  {
    const double tolerance = 0.5; // Define a tolerance for goal checking
    return std::abs(position.x - goal.position.x) < tolerance &&
            std::abs(position.y - goal.position.y) < tolerance &&
            std::abs(position.z - goal.position.z) < tolerance;
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
        child->position.x += std::get<0>(delta);
        child->position.y += std::get<1>(delta);
        child->position.z += std::get<2>(delta);
        
        tf2::Quaternion q_orig, q_rot, q_new;
        q_orig.setRPY(0.0, 0.0, 0.0);
        q_rot.setRPY(0.0, 0.0, atan2(std::get<1>(delta),std::get<0>(delta))); // -M_PI/2
        q_new = q_orig * q_new;
        // std::cout <<"x: " << q_rot.getX() << " y: " << q_rot.getY() << 
        //           " z: " << q_rot.getZ() << " w: " << q_rot.getW() << std::endl;
        child->orientation.x = q_rot.getX();
        child->orientation.y = q_rot.getY();
        child->orientation.z = q_rot.getZ();
        child->orientation.w = q_rot.getW();
        out.push_back(std::move(child));
    }
    // std::cout << "children generated" << std::endl;
    return out;
  }

  double h_cost;
  double g_cost;
};


#endif // BOAT_NODE_H
