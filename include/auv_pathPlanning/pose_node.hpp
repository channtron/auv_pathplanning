#ifndef BOAT_NODE_H
#define BOAT_NODE_H

#include <rclcpp/clock.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <fcl/fcl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/narrowphase/collision.h>



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
    // return pose.position==other.pose.position;
    return std::abs(pose.position.x - other.pose.position.x) < tolerance &&
            std::abs(pose.position.y - other.pose.position.y) < tolerance &&
            std::abs(pose.position.z - other.pose.position.z) < tolerance;
  }

  
  float h(const auvNode &goal) const
  {
    auto dx = goal.pose.position.x - pose.position.x;
    auto dy = goal.pose.position.y - pose.position.y;
    auto dz = goal.pose.position.z - pose.position.z;
    return std::sqrt(std::pow(dx,2)+std::pow(dy,2)+std::pow(dz,2));
  }

  virtual float distToParent() const {return 2.;}

  bool isGoal(const auvNode &goal) const
  {
    return std::abs(pose.position.x - goal.pose.position.x) < tolerance &&
            std::abs(pose.position.y - goal.pose.position.y) < tolerance &&
            std::abs(pose.position.z - goal.pose.position.z) < tolerance;
  }

  std::vector<auvNodePtr> children() const
  {
    
    std::vector<auvNodePtr> out;
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
        octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);
        octomap::point3d direction(std::get<0>(delta), std::get<1>(delta), std::get<2>(delta));
        octomap::point3d end;
        if (!octree_->castRay(origin, direction, end, true, displacement))
        {
          auto child = std::make_unique<auvNode>(*this);
          child->pose.position.x += std::get<0>(delta);
          child->pose.position.y += std::get<1>(delta);
          child->pose.position.z += std::get<2>(delta);
          
          tf2::Quaternion q_rot;
          q_rot.setRPY(0.0, 0.0, atan2(std::get<1>(delta),std::get<0>(delta))); // -M_PI/21
          // std::cout <<"x: " << q_rot.getX() << " y: " << q_rot.getY() << 
          //           " z: " << q_rot.getZ() << " w: " << q_rot.getW() << std::endl;
          child->pose.orientation.x = q_rot.getX();
          child->pose.orientation.y = q_rot.getY();
          child->pose.orientation.z = q_rot.getZ();
          child->pose.orientation.w = q_rot.getW();
          if (child->pose.position.z <= -1.0 && child->imFree()) out.push_back(std::move(child));

        }
    }
    // std::cout << "children generated" << std::endl;
    return out;
  }

  bool isFree (double x, double y, double z)
  { /* octree_->isNodeOccupied (
    auto node = octree_->search(x, y, z);
    if (node != NULL) return node->getOccupancy()<=0.5;
    else return false;
    */
    if (treeCollisionObj)
    {
      // // we model a sample robot with box and sphere collision  geometry
      std::shared_ptr<fcl::CollisionGeometry<float>> robot(new fcl::Sphere<float>(collisionRadius));
      auto robotCollisionObj = std::make_shared<fcl::CollisionObject<float>>(robot);

      // // perform collision checking between collision object tree and collision object robot
      
      fcl::Vector3f translation(x, y, z);
      robotCollisionObj->setTranslation(translation);
      fcl::CollisionRequest<float> requestType(1, false, 1, false);
      fcl::CollisionResult<float> collisionResult;
      fcl::collide(robotCollisionObj.get(), treeCollisionObj.get(), requestType, collisionResult);
    
      if (collisionResult.isCollision()) {
          std::cout << "Collision detected!" << std::endl;
          return false;
      } else {
          std::cout << "No collision." << std::endl;
      }
    }
    return true;
  }

  bool imFree ()
  {
    return isFree(
      pose.position.x,
      pose.position.y,
      pose.position.z
    );
  }

  void updateOctree(octomap::OcTree* _tree) 
  {
    if (_tree) {
            octree_ = std::shared_ptr<octomap::OcTree>((_tree));
            fcl::OcTree<float>* tree(new fcl::OcTree<float>(octree_));
            treeCollisionObj = std::make_shared<fcl::CollisionObject<float>>((std::shared_ptr<fcl::CollisionGeometry<float>>(tree)));
        } else {
            // Handle the case where _tree is nullptr, perhaps initializing an empty or default tree
            fcl::OcTree<float>* tree(new fcl::OcTree<float>(0.1)); // Default resolution
            treeCollisionObj = std::make_shared<fcl::CollisionObject<float>>((std::shared_ptr<fcl::CollisionGeometry<float>>(tree)));
        }
    /*
    octree_ = std::shared_ptr<octomap::OcTree>((_tree));
    std::shared_ptr<const octomap::OcTree> sharedTree = std::make_shared<const octomap::OcTree>(*_tree);
	  fcl::OcTree<double>* tree = new fcl::OcTree<double>(sharedTree);
	  treeCollisionObj = new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry<double>>(tree));
    //
    std::shared_ptr<const fcl::OcTree> tree(new fcl::OcTree(_tree));
	  treeCollisionObj = new fcl::CollisionObject(tree);*/
	}

  void setDisplacement (double disp) 
  {
    displacement=disp;
    tolerance=displacement/2;
  }
  void setRadius (double rad) {collisionRadius=rad;}


private:

    static std::shared_ptr<octomap::OcTree> octree_;
    static std::shared_ptr<fcl::CollisionObject<float>> treeCollisionObj;
    
    static double displacement; // Define displacement between nodes
    static double tolerance; // Define a tolerance for goal/other checking
    static double collisionRadius; // Collision sphere radius

    double h_cost;
    double g_cost;
};

// Define static members
std::shared_ptr<fcl::CollisionObject<float>> auvNode::treeCollisionObj = nullptr;
std::shared_ptr<octomap::OcTree> auvNode::octree_ = nullptr;


double auvNode::displacement{2.};
double auvNode::tolerance{1.}; 
double auvNode::collisionRadius{2.}; 


#endif // BOAT_NODE_H
