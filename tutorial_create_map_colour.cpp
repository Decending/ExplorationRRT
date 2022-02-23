// We will subscribe to colored UFOMaps in this tutorial
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
// UFOMap ROS msg
#include <ufomap_msgs/UFOMapStamped.h>
// To convert between UFO and ROS
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <ros/ros.h>
#include <list>
#include <iostream>
#include <chrono>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h>

using namespace std::chrono;
using namespace std;

struct node{
   public:
      ufo::math::Vector3* point;
      node* myParent;
      double distanceToParent;
      std::list<struct node*> myChilds{};
      std::list<struct node*> myPath{};
      double distanceToGoal;
      node(float x, float y, float z){
         point = new ufo::math::Vector3(x, y, z);
      }
      ufo::math::Vector3* getNodePointer(){
         return point;
      }
      void addChild(node* newChild){
        myChilds.push_back(newChild);
      }
      void addParent(node* newParent){
        myParent = newParent;
      }
      void changeDistanceToGoal(double newDistance){
        distanceToGoal = newDistance;
      }
      void changeDistanceToParent(){
        if(myParent != nullptr){
          distanceToParent = sqrt(pow(point->x() - myParent->point->x(), 2) + pow(point->y() - myParent->point->y(), 2) + pow(point->z() - myParent->point->z(), 2));
        }
        else{
          distanceToParent = 0;
        }
      }
      double sumDistance(){
        changeDistanceToParent();
        double myDistance = 0;
        if(myParent != nullptr){
          myDistance = myParent->sumDistance();
          changeDistanceToParent();
        }
        if(myParent == nullptr){
          return 0;
        }
        return (myDistance + distanceToParent);
      }
      void getPath(std::list<struct node*>* givenPath){
        if(myParent != nullptr){
          myParent->getPath(givenPath);
          givenPath->push_back(myParent);
        }
        if(myParent == nullptr){
          return;
        }
        return;
      }
};

// Variables of interest
int NUMBER_OF_NODES = 3000;
int NUMBER_OF_GOALS = 3;
int NUMBER_OF_ITTERATIONS = 3000;
bool RUN_BY_NODES = true;
double SENSOR_RANGE = 2;
int itterations;
int STEP_LENGTH = 1;
float SCALER_AABB = 10;
float SCALER_X;
float SCALER_Y;
float SCALER_Z;
double radius = 0.5;
bool map_received = false;
bool RRT_created = false;
bool position_received = false;
bool fetched_path = false;
float position_x = 0;
float position_y = 0;
float position_z = 0;
double arbitraryDistance = 0.5;
struct node* goalNode;
float lowest_x;
float lowest_y;
float lowest_z;
float highest_x;
float highest_y;
float highest_z;


// Create a colored UFOMap
ufo::map::OccupancyMapColor myMap(0.1);
std::list<struct node> RRT_TREE{};
std::list<struct node*> PATH{};
std::list<ufo::math::Vector3> myGoals{};
std::list<struct node> goalNodes{};

void tuneGeneration(ufo::map::OccupancyMapColor const& map, bool occupied_space, bool free_space, bool unknown_space, ufo::map::DepthType min_depth = 0){
  highest_x = std::numeric_limits<float>::min();
  highest_y = std::numeric_limits<float>::min();
  highest_z = std::numeric_limits<float>::min();
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();
  ufo::math::Vector3 minPoint(position_x - 1 * SCALER_AABB, position_y - 1 * SCALER_AABB, position_z - 1 * SCALER_AABB);
  ufo::math::Vector3 maxPoint(position_x - 1 + SCALER_AABB, position_y + 1 * SCALER_AABB, position_z + 1 * SCALER_AABB);
  ufo::geometry::AABB aabb(minPoint, maxPoint);
  for (auto it = map.beginLeaves(aabb, occupied_space, free_space, unknown_space, false, min_depth), it_end = map.endLeaves(); it != it_end; ++it) {
    if(it.getX() > highest_x){
      highest_x = it.getX();
    }if(it.getX() < lowest_x){
      lowest_x = it.getX();
    }
    if(it.getY() > highest_y){
      highest_y = it.getX();
    }if(it.getY() < lowest_y){
      lowest_y = it.getY();
    }
    if(it.getZ() > highest_z){
      highest_z = it.getX();
    }if(it.getZ() < lowest_z){
      lowest_z = it.getZ();
    }
  }
  SCALER_X = highest_x - lowest_x;
  SCALER_Y = highest_y - lowest_y;
  SCALER_Z = highest_z - lowest_z;
}

bool isInCollision(ufo::map::OccupancyMapColor const& map, 
                   ufo::geometry::BoundingVar const& bounding_volume, 
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = 0)
{
  // Iterate through all leaf nodes that intersects the bounding volume
  for (auto it = map.beginLeaves(bounding_volume, occupied_space, 
                                 free_space, unknown_space, false, min_depth), 
        it_end = map.endLeaves(); it != it_end; ++it) {
    // Is in collision since a leaf node intersects the bounding volume.
    return true;
  }
  // No leaf node intersects the bounding volume.
  return false;
}

void generateGoals(){
  // Generate goals, check for occupancy status
  ufo::math::Vector3 goal;
  srand(time(0));
  while((myGoals.size() < NUMBER_OF_GOALS) and (itterations < 10000)){
    float x = lowest_x + 1024 * rand () / (RAND_MAX + 1.0) * SCALER_X;
    float y = lowest_y + 1024 * rand () / (RAND_MAX + 1.0) * SCALER_Y;
    float z = lowest_z + 1024 * rand () / (RAND_MAX + 1.0) * SCALER_Z;
    ufo::math::Vector3 goal(position_x + x, position_y + y, position_z + z);
    ufo::geometry::Sphere goal_sphere(goal, radius);
    if(!isInCollision(myMap, goal_sphere, true, false, true, 0) and isInCollision(myMap, goal_sphere, false, true, false, 0)){
      myGoals.push_back(goal);
    };
      itterations++;
  };
  std::cout << "Goals generated successfully\n" << std::endl;
};

void setPath(){
  float distanceToTravel = std::numeric_limits<float>::max();
  for(std::list<node>::iterator it_goal = goalNodes.begin(); it_goal != goalNodes.end(); it_goal++){
    if((it_goal->sumDistance() < distanceToTravel)){
      distanceToTravel = it_goal->sumDistance();
      //std::cout << "distance to travel: " << distanceToTravel << std::endl;
      goalNode = &*it_goal;
      //std::cout << "complete answer: " << sqrt(pow(goalNode->point->x() - goalNode->myParent->point->x(), 2) + pow(goalNode->point->y() - goalNode->myParent->point->y(), 2) + pow(goalNode->point->z() - goalNode->myParent->point->z(), 2)) << std::endl;
    }
  }
}

void generateRRT(){
  RRT_TREE.clear();
  std::cout << "Building RRT-tree" << std::endl;
  float step_length = 1;
  float sensor_range = 2;
  node origin(position_x, position_y, position_z);
  origin.addParent(nullptr);
  origin.changeDistanceToGoal(std::numeric_limits<double>::max());
  RRT_TREE.push_back(origin);
  srand(time(0));
  while(((RRT_TREE.size() <= NUMBER_OF_NODES and RUN_BY_NODES) or (itterations <= NUMBER_OF_ITTERATIONS and !RUN_BY_NODES)) and itterations < 100000){
    // Generate a random point
    float x = lowest_x + 1024 * rand () / (RAND_MAX + 1.0) * SCALER_X;
    float y = lowest_y + 1024 * rand () / (RAND_MAX + 1.0) * SCALER_Y;
    float z = lowest_z + 1024 * rand () / (RAND_MAX + 1.0) * SCALER_Z;
    ufo::math::Vector3 random_point(position_x + x, position_y + y, position_z + z);
    // If the point is not occupied, continue
    if(!isInCollision(myMap, random_point, true, false, true, 0) and isInCollision(myMap, random_point, false, true, false, 0)){
      // TO DO: check for unknown within sensor_range
      //Find closest node in the RRT-TREE, set up the relation and add to tree
      float distance = std::numeric_limits<float>::max();
      node* parent;
      std::list<node>::iterator it_node;
      for(it_node = RRT_TREE.begin(); it_node != RRT_TREE.end(); it_node++){
        ufo::math::Vector3 direction = random_point - *(it_node->point);
        double new_distance = abs(direction.norm());
        if(new_distance < distance){
          distance = new_distance;
          parent = &*it_node;
        }
      };
      ufo::math::Vector3 start_point(parent->point->x(), parent->point->y(), parent->point->z());
      ufo::geometry::LineSegment myLine(random_point, start_point);

      // Check if the oriented bounding box collides with either occupied or unknown space,
      // at depth 3 (16 cm)
      if(!isInCollision(myMap, myLine, true, false, true, 0)) {
        node new_node(position_x + x, position_y + y, position_z + z);
        new_node.addParent(parent);
        new_node.changeDistanceToGoal(std::numeric_limits<double>::max());
        parent->addChild(&new_node);
        RRT_TREE.push_back(new_node);
        for(std::list<ufo::math::Vector3>::iterator it_goals = myGoals.begin(); it_goals != myGoals.end(); it_goals++){
        
          float new_distance = sqrt(pow(new_node.point->x() - it_goals->x(), 2) + pow(new_node.point->y() - it_goals->y(), 2) + pow(new_node.point->z() - it_goals->z(), 2));
          if(new_distance < new_node.distanceToGoal){
            new_node.changeDistanceToGoal(new_distance);
          }
        }
        if(new_node.distanceToGoal < arbitraryDistance){
          goalNodes.push_back(new_node);
        }
      }
    };
    // find closest node in the RRT-TREE
    itterations++;
  };
  std::cout << "RRT-tree built successfully" << std::endl;
  
  if(RUN_BY_NODES){
    std::cout << "Verifying tree" << std::endl;
    int total_childs = 0;
    int total_parents = 0;
    std::list<node>::iterator it_comeon;
    for(it_comeon = RRT_TREE.begin(); it_comeon != RRT_TREE.end(); it_comeon++){
      total_childs = total_childs + it_comeon->myChilds.size();
      if(it_comeon->myParent != nullptr){
        total_parents = total_parents + 1;
      }
    };
    if(total_childs == NUMBER_OF_NODES){
      std::cout << "All children accounter for" << std::endl;
    }else{
      std::cout << "Expected " << NUMBER_OF_NODES << " children, but " << total_childs << " was found." << std::endl;
    };
    if(total_parents == NUMBER_OF_NODES){
      std::cout << "All parents accounter for" << std::endl;
    }else{
      std::cout << "Expected " << NUMBER_OF_NODES << " parents, but " << total_parents << " was found." << std::endl;
    };
  }else{
    std::cout << "Running by itterations, so the amount of nodes are unknown and hence can't be verified" << std::endl;
  }
};

void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const& msg)
{
  // Convert ROS message to UFOMap
  if (ufomap_msgs::msgToUfo(msg->map, myMap)) {
    // Conversion was successful
    //std::cout << "Conversion sucessful" << std::endl;
    map_received = true;
  } else {
    std::cout << "Conversion failed" << std::endl;
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  position_received = true;
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  position_z = msg->pose.pose.position.z;
}

int main(int argc, char *argv[])
{ 
  ros::init(argc, argv, "RRT_TREE");
  ros::NodeHandle nh;
  ros::Publisher points_pub = nh.advertise<visualization_msgs::Marker>("RRT_NODES", 10);
  ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("RRT_PATH", 10);
  ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>("RRT_GOALS", 10);
  ros::Publisher map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("goe_map", 10);
  ros::Subscriber map_sub = nh.subscribe("ufomap_mapping_server_node/map_depth_4", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("odometry/imu", 1, odomCallback);
  ros::Rate rate(10);
  
  /*auto start = high_resolution_clock::now();
  createRRT();
  auto stop = high_resolution_clock::now();
  */
  
  //auto duration = duration_cast<microseconds>(stop - start);
  //cout << "\nExecution time: " << duration.count() << " micro seconds for " << NUMBER_OF_NODES << " itterations." << endl;
  while(ros::ok()){
    if(map_received and not RRT_created){
      itterations = 0;
      tuneGeneration(myMap, false, true, false, 0);
      myGoals.clear();
      generateGoals();
      goalNodes.clear();
      itterations = 0;
      auto start = high_resolution_clock::now();
      generateRRT();
      auto stop = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>(stop - start);
      if(itterations != 100000){
        cout << "\nExecution time: " << duration.count() << " micro seconds for " << itterations << " itterations." << endl;
      }else{
        cout << "\nTimeout after " << duration.count() << " micro seconds for " << itterations << " itterations." << endl;
      }
      RRT_created = true;
      itterations = 0;
    }

    if(map_received and RRT_created){
      visualization_msgs::Marker RRT_points, RRT_line_list, PATH_points, PATH_line_list, GOAL_points;
    
      RRT_points.header.frame_id = RRT_line_list.header.frame_id = "map";
      RRT_points.ns = "points";
      RRT_points.action = visualization_msgs::Marker::ADD;
      RRT_points.pose.orientation.w = 1.0;
      RRT_points.id = 0;
      RRT_line_list.id = 1;
      RRT_points.type = visualization_msgs::Marker::POINTS;
      RRT_line_list.type = visualization_msgs::Marker::LINE_LIST;
      RRT_points.scale.x = 0.2;
      RRT_points.scale.y = 0.2;
      RRT_line_list.scale.x = 0.1;
      RRT_points.color.g = 1.0f;
      RRT_points.color.a = 1.0;
      RRT_line_list.color.b = 1.0;
      RRT_line_list.color.a = 1.0;
      std::list<node>::iterator it_comeon_visualizer;	
      for(it_comeon_visualizer = RRT_TREE.begin(); it_comeon_visualizer != RRT_TREE.end(); it_comeon_visualizer++){
        geometry_msgs::Point p;
        p.x = it_comeon_visualizer->point->x();
        p.y = it_comeon_visualizer->point->y();
        p.z = it_comeon_visualizer->point->z();
        RRT_points.points.push_back(p);
        if(it_comeon_visualizer->myParent != nullptr){
          RRT_line_list.points.push_back(p);
          p.x = it_comeon_visualizer->myParent->point->x();
          p.y = it_comeon_visualizer->myParent->point->y();
          p.z = it_comeon_visualizer->myParent->point->z();
          RRT_line_list.points.push_back(p);
        }
      }
      points_pub.publish(RRT_points);
      points_pub.publish(RRT_line_list);
      
      if(!fetched_path and RRT_created){
      fetched_path = true;
      PATH_points.header.frame_id = PATH_line_list.header.frame_id = "map";
      PATH_points.ns = "points";
      PATH_points.action = visualization_msgs::Marker::ADD;
      PATH_points.pose.orientation.w = 1.0;
      PATH_points.id = 0;
      PATH_line_list.id = 1;
      PATH_points.type = visualization_msgs::Marker::POINTS;
      PATH_line_list.type = visualization_msgs::Marker::LINE_LIST;
      PATH_points.scale.x = 0.2;
      PATH_points.scale.y = 0.2;
      PATH_line_list.scale.x = 0.1;
      PATH_points.color.g = 1.0f;
      PATH_points.color.a = 1.0;
      PATH_line_list.color.b = 1.0;
      PATH_line_list.color.a = 1.0;
      setPath();
      PATH.clear();
      goalNode->getPath(&PATH);
      PATH.push_back(goalNode);
      std::list<node*>::iterator it_comeon_visualizer2;	
      for(it_comeon_visualizer2 = PATH.begin(); it_comeon_visualizer2 != PATH.end(); it_comeon_visualizer2++){
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer2)->point->x();
        p.y = (*it_comeon_visualizer2)->point->y();
        p.z = (*it_comeon_visualizer2)->point->z();
        PATH_points.points.push_back(p);
        if((*it_comeon_visualizer2)->myParent != nullptr){
          PATH_line_list.points.push_back(p);
          p.x = (*it_comeon_visualizer2)->myParent->point->x();
          p.y = (*it_comeon_visualizer2)->myParent->point->y();
          p.z = (*it_comeon_visualizer2)->myParent->point->z();
          PATH_line_list.points.push_back(p);
        }
      }
      path_pub.publish(PATH_points);
      path_pub.publish(PATH_line_list);
      }
      
      GOAL_points.header.frame_id = "map";
      GOAL_points.ns = "points";
      GOAL_points.action = visualization_msgs::Marker::ADD;
      GOAL_points.pose.orientation.w = 1.0;
      GOAL_points.id = 0;
      GOAL_points.type = visualization_msgs::Marker::POINTS;
      GOAL_points.scale.x = 0.2;
      GOAL_points.scale.y = 0.2;
      GOAL_points.color.r = 1.0f;
      GOAL_points.color.a = 1.0;
      std::list<ufo::math::Vector3>::iterator it_comeon_visualizer3;	
      for(it_comeon_visualizer3 = myGoals.begin(); it_comeon_visualizer3 != myGoals.end(); it_comeon_visualizer3++){
        geometry_msgs::Point p;
        p.x = it_comeon_visualizer3->x();
        p.y = it_comeon_visualizer3->y();
        p.z = it_comeon_visualizer3->z();
        GOAL_points.points.push_back(p);
      }
      goal_pub.publish(GOAL_points);
      
      ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
      bool compress = false;
      ufo::map::DepthType pub_depth = 0;
      // Convert UFOMap to ROS message
    if (ufomap_msgs::ufoToMsg(myMap, msg->map, compress, pub_depth)) {
      // Conversion was successful
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "map";
      map_pub.publish(msg);					        
    }
    itterations++;
    if(itterations > 100){
      itterations = 0;
      fetched_path = false;
      RRT_created = false;
    }
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
