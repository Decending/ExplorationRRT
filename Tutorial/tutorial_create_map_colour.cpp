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
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "nav_msgs/Odometry.h"
#include <math.h>
#include "MAV/rrt/rrt_bindings.h"
#include <dlfcn.h>
#include <stdlib.h>

using namespace std::chrono;
using namespace std;
using namespace std::this_thread;

struct node{
   public:
      ufo::math::Vector3* point;
      node* myParent = nullptr;
      double distanceToParent;
      std::list<struct node*> myChilds{};
      std::list<struct node*> myPath{};
      std::list<ufo::math::Vector3> myHits{};
      double distanceToGoal;
      node(float x, float y, float z){
         point = new ufo::math::Vector3(x, y, z);
      }
      node(ufo::math::Vector3* givenPoint){
        point = new ufo::math::Vector3(givenPoint->x(), givenPoint->y(), givenPoint->z());
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
          if(myParent->myParent != nullptr){
            givenPath->push_back(myParent);
          }
        }
        if(myParent == nullptr){
          return;
        }
        return;
      }
      
      int findInformationGain(float SCALER_AABB, ufo::map::OccupancyMapColor const& map){
        if(myHits.empty()){
          //Setting up targets, X / Y targets in either positive (P) or negative (N) direction
          ufo::math::Vector3 targetXP(point->x() + 1, point->y(), point->z());
          ufo::math::Vector3 targetXN(point->x() - 1, point->y(), point->z());
          ufo::math::Vector3 targetYP(point->x(), point->y() + 1, point->z());
          ufo::math::Vector3 targetYN(point->x(), point->y() - 1, point->z());
          ufo::math::Vector3 upwards(point->x(), point->y(), point->z() + 1);
        
          //Angles
          double vertical_angle = 0.262;
          double horizontal_angle = 0.785;
        
          //Distances
          double near_distance = 0.2;
          double far_distance = 10;
        
          //Frustums
          ufo::geometry::Frustum frustXP(*point, targetXP, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          ufo::geometry::Frustum frustXN(*point, targetXN, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          ufo::geometry::Frustum frustYP(*point, targetYP, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          ufo::geometry::Frustum frustYN(*point, targetYN, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          int checks = 0;
          for (auto it = map.beginLeaves(frustXP, false, false, true, false, 4), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 4)){
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustXN, false, false, true, false, 4), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 4)){
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustYP, false, false, true, false, 4), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 4)){
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustYN, false, false, true, false, 4), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 4)){
              myHits.push_back(end_point);
            }
          }
          //std::cout << "My checks: " << checks << std::endl;
          if(myParent != nullptr){
            myParent->findInformationGain(SCALER_AABB, map);
          }
        }else{
          std::list<ufo::math::Vector3>::iterator it_hits;	
          for(it_hits = myHits.begin(); it_hits != myHits.end();){
            // std::cout << "kommer hit? 2.4.1" << std::endl;
            ufo::geometry::LineSegment myLine2(*point, *it_hits);
            if(isInCollision(map, myLine2, true, false, false, 4)){
              // std::cout << "kommer hit? 2.4.2" << std::endl;
              it_hits = myHits.erase(it_hits);
            }else{
              // std::cout << "kommer hit? 2.4.3" << std::endl;
              if(isInCollision(map, *it_hits, false, true, false, 4)){
                it_hits = myHits.erase(it_hits);
              }else{
                it_hits++;
              };
            };
            // std::cout << "kommer hit? 2.4.slut" << std::endl;
          };
        }
        // std::cout << "kommer hit? 2.4.slut.1" << std::endl;
        std::list<ufo::math::Vector3> myTotalHits{};
        // std::cout << "kommer hit? 2.4.slut.2" << std::endl;
        addHits(&myTotalHits);
        // std::cout << "kommer hit? 2.4.slut.3" << std::endl;
        int hits = myTotalHits.size();
        // std::cout << "kommer hit? 2.4.slut.4" << std::endl;
        return hits;
      }
      
      void clearInformationGain(){
        myHits.clear();
        if(myParent != nullptr){
          myParent->clearInformationGain();
        }
      }
      
      void addHits(std::list<ufo::math::Vector3>* hitList){
        bool add = true;
        // std::cout << "addhits.0" << std::endl;
        for(auto it = myHits.begin(), it_end = myHits.end(); it != it_end; ++it){
          //std::cout << "addhits.1" << std::endl;
          for(auto it2 = hitList->begin(), it_end2 = hitList->end(); it2 != it_end2; ++it2){
            //std::cout << "addhits.2" << std::endl;
            if(it->x() == it2->x() and it->y() == it2->y() and it->z() == it2->z()){
              // std::cout << "addhits.3" << std::endl;
              add = false;
              break;
            }
          }
          // std::cout << "addhits.4" << std::endl;
          if(add){
            // std::cout << "addhits.5" << std::endl;
            hitList->push_back(*it);
            // std::cout << "addhits.6" << std::endl;
          }
          add = true;
        }
        // std::cout << "addhits.6.5" << std::endl;
        if(myParent != nullptr){
          // std::cout << "addhits.7" << std::endl;
          myParent->addHits(hitList);
          // std::cout << "addhits.8" << std::endl;
        }
      };
      
      bool findPathImprovement(struct node* targetNode, ufo::map::OccupancyMapColor const& map, float givenDistance, float givenRadious){
        bool improvementFound;
        // std::cout << "kommer in i findPathImprovement" << std::endl;
        if(myParent != nullptr){
          improvementFound = myParent->findPathImprovement(targetNode, map, givenDistance, givenRadious);
        }else{
          ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
          if(!isInCollision(map, myLine, true, false, true, 4)){
            /*ufo::math::Vector3 newVector(targetNode->point->x() - point->x(), targetNode->point->y() - point->y(), targetNode->point->z() - point->z());
            //std::cout << "my newVector: " << newVector.x() << ", " << newVector.y() << ", " << newVector.z() << std::endl;
            float distance = newVector.norm();
            float itterations = (distance / givenDistance);
            //std::cout << "my distance: " << distance << std::endl;
            //std::cout << "my givenDistance: " << givenDistance << std::endl;
            //std::cout << "my itterations: " << itterations << std::endl;
            float part = givenDistance / distance;
            float xStep = (targetNode->point->x() - point->x()) * part;
            float yStep = (targetNode->point->y() - point->y()) * part;
            float zStep = (targetNode->point->z() - point->z()) * part;
            // node* parent = givenNode->myParent;
            // node* nextNode = givenNode->myParent;
            for(int i = 1; i < itterations; i++){
              //std::cout << "\nnewPoints x, y, z: " << parent->point->x() << ", " << parent->point->y() << ", " << parent->point->z() << std::endl;
              ufo::math::Vector3 newVector = ufo::math::Vector3(point->x() + i * xStep, point->y() + i * yStep, point->z() + i * zStep);
              ufo::geometry::Sphere new_sphere(newVector, givenRadious);
              if(isInCollision(map, new_sphere, true, false, true, 4)){
                return false;
              }
              //std::cout << "newPoints x, y, z: " << newPoint->point->x() << ", " << newPoint->point->y() << ", " << newPoint->point->z() << std::endl;
              // newPoint->addParent(parent);
              //std::cout << "newPoints x, y, z: " << newPoint->myParent->point->x() << ", " << newPoint->myParent->point->y() << ", " << newPoint->myParent->point->z() << std::endl;
              // parent = newPoint;
              // RRT_TREE.push_back(*newPoint);
            }*/
            targetNode->addParent(this);
            return true;
          }else{
            return false;
          };
        }
        if(!improvementFound){
          ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
          if(!isInCollision(map, myLine, true, false, true, 4)){
            targetNode->addParent(this);
            improvementFound = findPathImprovement(this, map, givenDistance, givenRadious);
            return improvementFound;
          }else{
            return false;
          }
        }else{
          return improvementFound;
        }
      }
      
      bool isInCollision(ufo::map::OccupancyMapColor const& map, 
                   ufo::geometry::BoundingVar const& bounding_volume, 
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = 0){
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
};

// Variables of interest
int NUMBER_OF_NODES = 3000;
int NUMBER_OF_GOALS = 40;
int NUMBER_OF_ITTERATIONS = 3000;
float DISTANCE_BETWEEN_NODES = 1;
bool RUN_BY_NODES = true;
double SENSOR_RANGE = 2;
double SCALER_INFORMATION_GAIN = 25;
int itterations;
int STEP_LENGTH = 1;
float SCALER_AABB = 20;
float SCALER_X = SCALER_AABB;
float SCALER_Y = SCALER_AABB;
float SCALER_Z = SCALER_AABB;
double radius = 0.6;
bool map_received = false;
bool RRT_created = false;
bool GOALS_generated = false;
bool position_received = false;
bool fetched_path = false;
bool newPath = false;
bool allowNewPath = true;
float position_x = 0;
float position_y = 0;
float position_z = 0;
double arbitraryDistance = 0.5;
node* goalNode = nullptr;
float lowest_x;
float lowest_y;
float lowest_z;
float highest_x;
float highest_y;
float highest_z;
double totalCost = std::numeric_limits<float>::max();
double totalDistance = -1;
int advance_index = 0;


// Create a colored UFOMap
ufo::map::OccupancyMapColor myMap(0.4);
std::list<struct node> RRT_TREE{};
std::list<struct node*> CHOSEN_PATH{};
std::list<struct node*> ALL_PATH{};
std::list<struct node> myGoals{};
std::list<ufo::math::Vector3> hits{};
std::list<geometry_msgs::Point> VISITED_POINTS{};
std::list<node*>::iterator path_itterator;
node* currentTarget;

void linSpace(node* givenNode, float givenDistance){
  //ufo::math::Vector3 newVector = *(givenNode->myParent->point) - *(givenNode->point);
  //std::cout << "här börjar linSpace" << std::endl;
  
  ufo::math::Vector3 newVector(givenNode->myParent->point->x() - givenNode->point->x(), givenNode->myParent->point->y() - givenNode->point->y(), givenNode->myParent->point->z() - givenNode->point->z());
  //std::cout << "my newVector: " << newVector.x() << ", " << newVector.y() << ", " << newVector.z() << std::endl;
  float distance = newVector.norm();
  float itterations = (distance / givenDistance);
  //std::cout << "my distance: " << distance << std::endl;
  //std::cout << "my givenDistance: " << givenDistance << std::endl;
  //std::cout << "my itterations: " << itterations << std::endl;
  float part = givenDistance / distance;
  float xStep = (givenNode->myParent->point->x() - givenNode->point->x()) * part;
  float yStep = (givenNode->myParent->point->y() - givenNode->point->y()) * part;
  float zStep = (givenNode->myParent->point->z() - givenNode->point->z()) * part;
  node* parent = givenNode->myParent;
  node* nextNode = givenNode->myParent;
  for(int i = 1; i < itterations; i++){
    //std::cout << "\nnewPoints x, y, z: " << parent->point->x() << ", " << parent->point->y() << ", " << parent->point->z() << std::endl;
    node* newPoint = new node(givenNode->myParent->point->x() - i * xStep, givenNode->myParent->point->y() - i * yStep, givenNode->myParent->point->z() - i * zStep);
    //std::cout << "newPoints x, y, z: " << newPoint->point->x() << ", " << newPoint->point->y() << ", " << newPoint->point->z() << std::endl;
    newPoint->addParent(parent);
    //std::cout << "newPoints x, y, z: " << newPoint->myParent->point->x() << ", " << newPoint->myParent->point->y() << ", " << newPoint->myParent->point->z() << std::endl;
    parent = newPoint;
    RRT_TREE.push_back(*newPoint);
  }
  givenNode->addParent(parent);
  if(nextNode->myParent != nullptr){
    linSpace(nextNode, givenDistance);
  }
  //std::cout << "kommer ut" << std::endl;
  /*if(givenNode->myParent != nullptr){
    linSpace(givenNode->myParent, givenDistance);
  }*/
}

void tuneGeneration(ufo::map::OccupancyMapColor const& map, bool occupied_space, bool free_space, bool unknown_space, ufo::map::DepthType min_depth = 4){
  highest_x = std::numeric_limits<float>::min();
  highest_y = std::numeric_limits<float>::min();
  highest_z = std::numeric_limits<float>::min();
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();
  ufo::math::Vector3 minPoint(position_x - 1 * SCALER_AABB, position_y - 1 * SCALER_AABB, position_z - 1 * SCALER_AABB);
  ufo::math::Vector3 maxPoint(position_x + 1 * SCALER_AABB, position_y + 1 * SCALER_AABB, position_z + 1 * SCALER_AABB);
  ufo::geometry::AABB aabb(minPoint, maxPoint);
  for (auto it = map.beginLeaves(aabb, occupied_space, free_space, unknown_space, false, min_depth), it_end = map.endLeaves(); it != it_end; ++it) {
    if(it.getX() > highest_x){
      highest_x = it.getX();
    }if(it.getX() < lowest_x){
      lowest_x = it.getX();
    }
    if(it.getY() > highest_y){
      highest_y = it.getY();
    }if(it.getY() < lowest_y){
      lowest_y = it.getY();
    }
    if(it.getZ() > highest_z){
      highest_z = it.getZ();
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
                   bool unknown_space = false, ufo::map::DepthType min_depth = 4)
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

void findShortestPath(){
  for(std::list<node>::iterator it_goals = myGoals.begin(); it_goals != myGoals.end(); it_goals++){
    // std::cout << "är det här? 0" << std::endl;
    struct node* chosenNode = nullptr;
    double distance = std::numeric_limits<double>::max();
    for(std::list<node>::iterator it_RRT = RRT_TREE.begin(); it_RRT != RRT_TREE.end(); it_RRT++){
      // std::cout << "är det här? 1" << std::endl;
      double distanceNodeToGoal = sqrt(pow(it_RRT->point->x() - it_goals->point->x(), 2) + pow(it_RRT->point->y() - it_goals->point->y(), 2) + pow(it_RRT->point->z() - it_goals->point->z(), 2));
      double distanceToNode = it_RRT->sumDistance();
      double totalDistance = distanceNodeToGoal + distanceToNode;
      // std::cout << "är det här? 2" << std::endl;
      if(totalDistance < distance){
        // std::cout << "är det här? 3" << std::endl;
        ufo::geometry::LineSegment myLine(*(it_goals->point), *(it_RRT->point));
        if(!isInCollision(myMap, myLine, true, false, true, 4)){
          // std::cout << "är det här? 4" << std::endl;
          distance = totalDistance;
          chosenNode = &*it_RRT;
        }
      }
    }
    // std::cout << "är det här? 5" << std::endl;
    if(chosenNode != nullptr){
      it_goals->addParent(chosenNode);
      // std::cout << "är det här? 6" << std::endl;
      chosenNode->addChild(&*it_goals);
      // std::cout << "är det här? 7" << std::endl;
    }
  }
}

void generateGoals(ufo::map::OccupancyMapColor const& map){
  // Generate goals, check for occupancy status
  ufo::math::Vector3 goal;
  srand(time(0));
  while((myGoals.size() < NUMBER_OF_GOALS) and (itterations < 10000)){
    float x = lowest_x + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_X;
    float y = lowest_y + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Y;
    float z = lowest_z + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Z;
    ufo::math::Vector3 goal(x, y, z);
    ufo::geometry::Sphere goal_sphere(goal, radius);
    if(!isInCollision(myMap, goal_sphere, true, false, true, 4) and isInCollision(myMap, goal_sphere, false, true, false, 4)){
      ufo::math::Vector3 min_point(x - SENSOR_RANGE, y - SENSOR_RANGE, z - SENSOR_RANGE);
      ufo::math::Vector3 max_point(x + SENSOR_RANGE, y + SENSOR_RANGE, z + SENSOR_RANGE);
      ufo::math::Vector3 position(position_x, position_y, position_z);
      ufo::geometry::AABB aabb(min_point, max_point);
      for (auto it = map.beginLeaves(aabb, false, false, true, false, 4), it_end = map.endLeaves(); it != it_end; ++it) {
        if (it.isUnknown()) {
          ufo::math::Vector3 unknownNode(it.getX(), it.getY(), it.getZ());
          ufo::geometry::LineSegment myLine(goal, unknownNode);
          if(!isInCollision(map, myLine, true, false, false, 4)){
            node* newGoal = new node(x, y, z);
            myGoals.push_back(*newGoal);
            break;
          }
        }
      break;
      }
    };
      itterations++;
  };
  if(myGoals.size() == NUMBER_OF_GOALS){
    std::cout << "Goals generated successfully\n" << std::endl;
    GOALS_generated = true;
  }else if(myGoals.size() == 0){
    std::cout << "No goals found, trying again soon" << std::endl;
    sleep_for(microseconds(100000));
  }else{
    std::cout << "Only " << myGoals.size() << " goals found" << std::endl;
    GOALS_generated = true;
  }
};

void setPath(){
  bool setDistance = false;
  if(goalNode != nullptr){
    goalNode->clearInformationGain();
    totalCost = goalNode->sumDistance() - SCALER_INFORMATION_GAIN * (goalNode->findInformationGain(SCALER_AABB, myMap));
    if(max(totalDistance * 0.1, 0.5) > goalNode->sumDistance()){
      allowNewPath = true;
    }
  }
  double newCost = std::numeric_limits<float>::max();
  if(allowNewPath){
    for(std::list<node>::iterator it_goal = myGoals.begin(); it_goal != myGoals.end(); it_goal++){
      // std::cout << "kommer hit? 2.1" << std::endl;
      if(it_goal->myParent != nullptr){	
        it_goal->findPathImprovement(&*it_goal, myMap, DISTANCE_BETWEEN_NODES, radius);
        // std::cout << "kommer hit? 2.2" << std::endl;
        linSpace(&*it_goal, DISTANCE_BETWEEN_NODES);
        // std::cout << "kommer hit? 2.3" << std::endl;
        it_goal->findPathImprovement(&*it_goal, myMap, DISTANCE_BETWEEN_NODES, radius);
        // std::cout << "kommer hit? 2.4" << std::endl;
        //std::cout << "Krashar efter cost calc?" << std::endl;
        newCost = it_goal->sumDistance() - SCALER_INFORMATION_GAIN * (it_goal->findInformationGain(SCALER_AABB, myMap));
        // std::cout << "kommer hit? 2.5" << std::endl;
        linSpace(&*it_goal, DISTANCE_BETWEEN_NODES);
        //std::cout << "naej" << std::endl;
        //std::cout << "My hits = " << it_goal->findInformationGain(SCALER_AABB, myMap) << std::endl;
        if(&(*it_goal) == goalNode){
          std::cout << it_goal->sumDistance() << " is my sum distance and the following is my total distance  " << 0.2 * totalDistance << std::endl;
        };
        if((newCost < totalCost) and (it_goal->findInformationGain(SCALER_AABB, myMap) > 0) and allowNewPath){
          std::cout << newCost << " < " << totalCost << ", " << it_goal->sumDistance() << " < " << 0.2 * totalDistance << std::endl;
          totalCost = newCost;
          goalNode = &*it_goal;
          newPath = true;
          setDistance = true;
          CHOSEN_PATH.clear();
          linSpace(goalNode, DISTANCE_BETWEEN_NODES);
          goalNode->getPath(&CHOSEN_PATH);
          CHOSEN_PATH.push_back(goalNode);
          path_itterator = CHOSEN_PATH.begin();
          currentTarget = *path_itterator;
          std::cout << currentTarget->point->x() << std::endl;
          std::cout << currentTarget->point->y() << std::endl;
          std::cout << currentTarget->point->z() << std::endl;
        }
      }
    }
  }else{
    goalNode->findPathImprovement(goalNode, myMap, DISTANCE_BETWEEN_NODES, radius);
    // std::cout << "kommer hit? 2.2" << std::endl;
    linSpace(goalNode, DISTANCE_BETWEEN_NODES);
    // std::cout << "kommer hit? 2.3" << std::endl;
    goalNode->findPathImprovement(goalNode, myMap, DISTANCE_BETWEEN_NODES, radius);
    // std::cout << "kommer hit? 2.4" << std::endl;
    //std::cout << "Krashar efter cost calc?" << std::endl;
    newCost = goalNode->sumDistance() - SCALER_INFORMATION_GAIN * (goalNode->findInformationGain(SCALER_AABB, myMap));
    // std::cout << "kommer hit? 2.5" << std::endl;
    linSpace(goalNode, DISTANCE_BETWEEN_NODES);
    //std::cout << "naej" << std::endl;
    //std::cout << "My hits = " << it_goal->findInformationGain(SCALER_AABB, myMap) << std::endl;
    if((newCost < totalCost) and (goalNode->findInformationGain(SCALER_AABB, myMap) > 0)){
      std::cout << newCost << " < " << totalCost << ", " << goalNode->sumDistance() << " < " << 0.2 * totalDistance << std::endl;
      totalCost = newCost;
      setDistance = true;
      CHOSEN_PATH.clear();
      goalNode->getPath(&CHOSEN_PATH);
      CHOSEN_PATH.push_back(goalNode);
      path_itterator = CHOSEN_PATH.begin();
      currentTarget = *path_itterator;
      std::cout << currentTarget->point->x() << std::endl;
      std::cout << currentTarget->point->y() << std::endl;
      std::cout << currentTarget->point->z() << std::endl;
    }
  }
  if(setDistance){
    totalDistance = goalNode->sumDistance();
  };
}

void generateRRT(){
  RRT_TREE.clear();
  std::cout << "Building RRT-tree" << std::endl;
  float step_length = 1;
  float sensor_range = 2;
  node origin(position_x, position_y, position_z);
  std::cout << "My guessed point: " << position_x << ", " << position_y << ", " << position_z << std::endl;
  origin.addParent(nullptr);
  RRT_TREE.push_back(origin);
  srand(time(0));
  while(((RRT_TREE.size() <= NUMBER_OF_NODES and RUN_BY_NODES) or (itterations <= NUMBER_OF_ITTERATIONS and !RUN_BY_NODES)) and itterations < 100000){
    // Generate a random point
    float x = lowest_x + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_X;
    float y = lowest_y + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Y;
    float z = lowest_z + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Z;
    ufo::math::Vector3 random_point(x, y, z);
    ufo::geometry::Sphere point_sphere(random_point, radius);
    // If the point is not occupied, continue
    if(!isInCollision(myMap, point_sphere, true, false, true, 0) and isInCollision(myMap, point_sphere, false, true, false, 0)){
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
      if(!isInCollision(myMap, myLine, true, false, true, 0)) {
        node new_node(x, y, z);
        new_node.addParent(parent);
        parent->addChild(&new_node);
        RRT_TREE.push_back(new_node);
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
    if(total_childs != 0){
      RRT_created = true;
    }
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

auto trajectory(std::list<float> x, std::list<float> u, float N, float dt, std::list<float> nmpc_ref, std::list<float> u_ref, std::list<float> u_old){
    // Based on the initial condition and optimized trajectory u, computed the path as (x,y,z).
    // Calculate the dynamic costs based on selected weights  
    float ns = 8;
    std::list<float> p_hist{};
    std::list<float> x_hist{};
    float cost = 0;
    // Weight matrices
    std::list<float> Qx = {0,0,0, 0, 0,0, 5, 5};
    // P = 2*Qx; #final state weight
    std::list<float> Ru = {7,7,7}; // input weights
    std::list<float> Rd = {3, 3, 3}; // input rate weights
    // print(x, u, N, dt)
    for(int i = 0; i < N; i++){
      // State costs
      // std::list<float> x_ref = nmpc_ref[(ns*i):(ns*i+ns)];
      // #print(x_ref)
      // Setting up itterators
      std::list<float>::iterator Qx_itterator = Qx.begin();
      std::advance(Qx_itterator, i);
      std::list<float>::iterator x_itterator = x.begin();
      std::advance(x_itterator, i);
      std::list<float>::iterator x_ref_itterator = nmpc_ref.begin();
      std::advance(x_ref_itterator, i*ns);
      
      for(int j = 0; j < 8; j++){
        cost = cost + pow((*Qx_itterator) * (*x_itterator - *x_ref_itterator), 2);
        Qx_itterator++;
        x_itterator++;
        x_ref_itterator++;
      }
      //cost = cost + pow((*Qx_itterator) * (*x_itterator - *x_ref_itterator), 2) + Qx[1]*(x[1]-x_ref[i + 1])**2 + Qx[2]*(x[2]-x_ref[i + 2])**2 + Qx[3]*(x[3]-x_ref[i + 3])**2 + Qx[4]*(x[4]-x_ref[i + 4])**2 + Qx[5]*(x[5]-x_ref[i + 5])**2 + Qx[6]*(x[6]-x_ref[i + 6])**2 + Qx[7]*(x[7]-x_ref[i + 7])**2;  // State weights
      // Input Cost
      
      //Setting up itterators
      std::list<float>::iterator Ru_itterator = Ru.begin();
      std::list<float>::iterator Rd_itterator = Rd.begin();
      std::list<float>::iterator u_n_itterator = u.begin();
      std::advance(u_n_itterator, 3 * i);
      std::list<float>::iterator u_ref_itterator = u_ref.begin();
      std::list<float>::iterator u_old_itterator = u.begin();
      
      for(int j = 0; j < 3; j++){
        cost = cost + *Ru_itterator * pow((*u_n_itterator) - (*u_ref_itterator), 2);
        cost = cost + *Rd_itterator * pow((*u_n_itterator) - (*u_old_itterator), 2);
        Qx_itterator++;
        x_itterator++;
        x_ref_itterator++;
      }
      
      //u_n = u[(3*i):3*i+3];
      //cost += Ru[0]*(u_n[0] - u_ref[0])**2 + Ru[1]*(u_n[1] - u_ref[1])**2 + Ru[2]*(u_n[2] - u_ref[2])**2; // Input weights
      //cost += Rd[0]*(u_n[0] - u_old[0])**2 + Rd[1]*(u_n[1] - u_old[1])**2 + Rd[2]*(u_n[2] - u_old[2])**2; // Input rate weights
      //u_old = u_n;
      // x_hist = x_hist + [x];
      for(x_itterator = x.begin(); x_itterator != x.end(); x_itterator++){
        x_hist.push_back(*x_itterator);
      }
      
      x_itterator = x.begin();
      std::list<float>::iterator x2_itterator = x.begin();
      std::advance(x2_itterator, 3);
      std::advance(u_n_itterator, -2);
      for(int j = 0; j < 3; j++){
        *x_itterator = *x_itterator + dt * *x2_itterator;
        x_itterator++;
        x2_itterator++;
      }
      std::list<float>::iterator x3_itterator = x.begin();
      std::advance(x3_itterator, 7); // x[7]
      *x_itterator = *x_itterator + dt * sin(*x3_itterator) * cos(*x2_itterator) * *u_n_itterator - 0.1 * *x_itterator;
      x_itterator++; // x[4]
      *x_itterator = *x_itterator + dt * (-sin(*x3_itterator)) * *u_n_itterator - 0.1 * *x_itterator;
      x_itterator++; // x[5]
      *x_itterator = *x_itterator + dt * cos(*x3_itterator) * cos(*x2_itterator) * *u_n_itterator - 0.2 * *x_itterator - 9.81;
      x_itterator++;
      u_n_itterator++;
      *x_itterator = *x_itterator + dt * ((1.0 / 0.3) * (*u_n_itterator - *x_itterator));
      x_itterator++;
      u_n_itterator++;
      *x_itterator = *x_itterator + dt * ((1.0 / 0.3) * (*u_n_itterator - *x_itterator));
      /*
      p_hist = p_hist + [[x[0],x[1],x[2]]];*/
      x_itterator = x.begin();
      p_hist.push_back(*x_itterator);
      x_itterator++;
      p_hist.push_back(*x_itterator);
      x_itterator++;      
      p_hist.push_back(*x_itterator);    
    }
    // print(cost)
    // print(p_hist)
    return(p_hist, cost, x_hist);
}

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

typedef rrtCache* (*arbitrary)();
typedef rrtSolverStatus (*arbitrary2)(void*, double*, double*, double, double*);

int main(int argc, char *argv[])
{ 
  ros::init(argc, argv, "RRT_TREE");
  ros::NodeHandle nh;
  ros::Publisher points_pub = nh.advertise<visualization_msgs::Marker>("RRT_NODES", 1);
  ros::Publisher chosen_path_visualization_pub = nh.advertise<visualization_msgs::Marker>("CHOSEN_RRT_PATH_VISUALIZATION", 1);
  // ros::Publisher chosen_path_pub = nh.advertise<nav_msgs::Path>("CHOSEN_RRT_PATH", 1);
  ros::Publisher chosen_path_pub = nh.advertise<geometry_msgs::PoseStamped>("/pelican/reference", 1);
  ros::Publisher all_path_pub = nh.advertise<visualization_msgs::Marker>("RRT_PATHS", 1);
  ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>("RRT_GOALS", 1);
  ros::Publisher map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("goe_map", 11);
  ros::Subscriber map_sub = nh.subscribe("ufomap_mapping_server_node/map_depth_4", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("/pelican/ground_truth/odometry", 1, odomCallback);
  ros::Publisher hits_pub = nh.advertise<visualization_msgs::Marker>("HITS", 1);
  ros::Publisher position_pub = nh.advertise<visualization_msgs::Marker>("POSITION", 1);
  ros::Publisher taken_path_pub = nh.advertise<visualization_msgs::Marker>("PATH_TAKEN", 1);
  ros::Rate rate(10);
  
  // C++ bindings battlefield
  /* parameters             */
  int i;
  double p[RRT_NUM_PARAMETERS] = {0};
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  p[3] = 0;
  p[4] = 0;
  p[5] = 0;
  p[6] = 0;
  p[7] = 0;
  
  for (i = 1; i < 51; ++i) {
    p[8*i] = 0.5*i;
    p[8*i+1] = 0.5*i;
    p[8*i+2] = 0;
    p[8*i+3] = 0;
    p[8*i+4] = 0;
    p[8*i+5] = 0;
    p[8*i+6] = 0;
    p[8*i+7] = 0;
        /*printf("%d\n", 8*i+7); */
  }
  
  p[408] = 9.81;
  p[409] = 0;
  p[410] = 0;
  p[411] = 9.81;
  p[412] = 0;
  p[413] = 0;
  p[414] = 0.5;
  
  /* initial guess          */
  double u[RRT_NUM_DECISION_VARIABLES] = {0};

  for (i = 0; i < 50; ++i) {
    u[3*i] = 9.81;
    u[3*i + 1] = 0;
    u[3*i + 2] = 0;
  }

  /* initial penalty        */
  double init_penalty = 15.0;
  void *handle = dlopen("./MAV/rrt/target/release/librrt.so", RTLD_LAZY);
  if (!handle) {
    fprintf(stderr, "%s\n", dlerror());
    exit(EXIT_FAILURE);
  }
  arbitrary rrt_new;
  *(void **) (&rrt_new) = dlsym(handle, "rrt_new");
  std::cout << rrt_new << std::endl;
  void* cache = rrt_new();
  std::cout << cache << std::endl;
  arbitrary2 rrt_solve;
  *(void **) (&rrt_solve) = dlsym(handle, "rrt_solve");
  std::cout << rrt_solve << std::endl;
  std::cout << init_penalty << std::endl;
  rrtSolverStatus status = rrt_solve(cache, u, p, 0, &init_penalty);
  printf("\n\n-------------------------------------------------\n");
  printf("  Solution\n");
  printf("-------------------------------------------------\n");

  for (i = 0; i < RRT_NUM_DECISION_VARIABLES; ++i) {
    printf("u[%d] = %g\n", i, u[i]);
  }

  printf("\n");
  for (i = 0; i < RRT_N1; ++i) {
    printf("y[%d] = %g\n", i, status.lagrange[i]);
  }

  printf("\n\n-------------------------------------------------\n");
  printf("  Solver Statistics\n");
  printf("-------------------------------------------------\n");
  printf("exit status      : %d\n", status.exit_status);
  printf("iterations       : %lu\n", status.num_inner_iterations);
  printf("outer iterations : %lu\n", status.num_outer_iterations);
  printf("solve time       : %f ms\n", (double)status.solve_time_ns / 1000000.0);
  printf("penalty          : %f\n", status.penalty);
  printf("||Dy||/c         : %f\n", status.delta_y_norm_over_c);
  printf("||F2(u)||        : %f\n", status.f2_norm);
  printf("Cost             : %f\n", status.cost);
  printf("||FRP||          : %f\n\n", status.last_problem_norm_fpr);

  // double (*cosine)(double);
  // cosine = (double (*)(double)) dlsym(handle, "cos");
  // printf("%f\n", (*cosine)(2.0));
  // printf("%f\n", cos(2.0));
  /*auto start = high_resolution_clock::now();
  createRRT();
  auto stop = high_resolution_clock::now();
  */
  
  //auto duration = duration_cast<microseconds>(stop - start);
  //cout << "\nExecution time: " << duration.count() << " micro seconds for " << NUMBER_OF_NODES << " itterations." << endl;
  while(ros::ok()){
    std::cout << "start" << std::endl;
    if(map_received and not GOALS_generated and position_received){
      itterations = 0;
      tuneGeneration(myMap, false, true, false, 4);
      std::list<node>::iterator it_goal2;
      for(it_goal2 = myGoals.begin(); it_goal2 != myGoals.end(); it_goal2++){
        if(&*it_goal2 == goalNode){
          it_goal2++;
          myGoals.erase(it_goal2, myGoals.end());
          it_goal2=myGoals.end();
          it_goal2--;
          myGoals.erase(myGoals.begin(), it_goal2);
        }
      }
      generateGoals(myMap);
    }
    if(map_received and not RRT_created and GOALS_generated){
      itterations = 0;
      high_resolution_clock::time_point start = high_resolution_clock::now();
      generateRRT();
      high_resolution_clock::time_point stop = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>(stop - start);
      if(itterations != 100000){
        cout << "\nExecution time: " << duration.count() << " micro seconds for " << itterations << " itterations." << endl;
      }else{
        cout << "\nTimeout after " << duration.count() << " micro seconds for " << itterations << " itterations." << endl;
      }
      if(RRT_created){
        std::cout << "går in mot findshortestpath" << std::endl;
        high_resolution_clock::time_point start2 = high_resolution_clock::now();
        std::cout << "är problemet findshortestpath?" << std::endl;
        findShortestPath();
        std::cout << "problemet är inte findshortestpath" << std::endl;
        high_resolution_clock::time_point stop2 = high_resolution_clock::now();
        auto duration2 = duration_cast<microseconds>(stop2 - start2);
        cout << "\nExecution time: " << duration2.count() << ", this is to find the shortest paths to our goals" << endl;
        itterations = 0;
      }
    }
  
    if(map_received and RRT_created){
      std::cout << "Går förbi map_received" << std::endl;
      visualization_msgs::Marker RRT_points, RRT_line_list, CHOSEN_PATH_points, CHOSEN_PATH_line_list, PATH_points, PATH_line_list, GOAL_points, HITS_points;
      // geometry_msgs::PoseStamped NEXT_POINT;
      // nav_msgs::Path MY_CHOSEN_PATH;
    
      RRT_points.header.frame_id = RRT_line_list.header.frame_id = "world";
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
      std::cout << "Försöker visualisera rrt_tree" << std::endl;	
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
      std::cout << "lyckades visualisera rrt_tree" << std::endl;
      points_pub.publish(RRT_points);
      points_pub.publish(RRT_line_list);
      
      if(!fetched_path and RRT_created){
      std::cout << "kommer hit? 1" << std::endl;
      fetched_path = true;
      CHOSEN_PATH_points.header.frame_id = CHOSEN_PATH_line_list.header.frame_id = "world"; //MY_CHOSEN_PATH.header.frame_id = "pelican/velodyne";
      CHOSEN_PATH_points.ns = "points";
      CHOSEN_PATH_points.action = visualization_msgs::Marker::ADD;
      CHOSEN_PATH_points.pose.orientation.w = 1.0;
      CHOSEN_PATH_points.id = 0;
      CHOSEN_PATH_line_list.id = 1;
      CHOSEN_PATH_points.type = visualization_msgs::Marker::POINTS;
      CHOSEN_PATH_line_list.type = visualization_msgs::Marker::LINE_LIST;
      CHOSEN_PATH_points.scale.x = 0.2;
      CHOSEN_PATH_points.scale.y = 0.2;
      CHOSEN_PATH_line_list.scale.x = 0.1;
      CHOSEN_PATH_points.color.g = 1.0f;
      CHOSEN_PATH_points.color.a = 1.0;
      CHOSEN_PATH_line_list.color.b = 1.0;
      CHOSEN_PATH_line_list.color.a = 1.0;
      std::cout << "kommer hit? 2" << std::endl;
      high_resolution_clock::time_point start = high_resolution_clock::now();
      setPath();
      std::cout << "kommer hit? 3" << std::endl;
      /*
      CHOSEN_PATH.clear();
      std::cout << "kommer hit? 4" << std::endl;
      goalNode->getPath(&CHOSEN_PATH);
      CHOSEN_PATH.push_back(goalNode);
      path_itterator = CHOSEN_PATH.begin();
      std::advance(path_itterator, advance_index);
      */
      std::cout << "kommer hit? 5" << std::endl;
      high_resolution_clock::time_point stop = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>(stop - start);
      cout << "\nExecution time: " << duration.count() << " micro seconds for " << myGoals.size() << " path/s." << endl;
      std::cout << "kommer hit? 6" << std::endl;
      if(newPath and allowNewPath){
        std::cout << "kommer hit? 6.1" << std::endl;
        newPath = false;
        allowNewPath = false;
        path_itterator = CHOSEN_PATH.begin();
        std::cout << "kommer hit? 6.2" << std::endl;
        currentTarget = *path_itterator;
        advance_index = 0;
        std::cout << "kommer hit? 6.2.0" << std::endl;
        std::cout << currentTarget << std::endl;
        std::cout << "kommer hit? 6.3" << std::endl;
      };
      std::cout << "kommer hit? 7" << std::endl;
      std::list<node*>::iterator it_comeon_visualizer2;
      if(!CHOSEN_PATH.empty()){	
      for(it_comeon_visualizer2 = CHOSEN_PATH.begin(); it_comeon_visualizer2 != CHOSEN_PATH.end(); it_comeon_visualizer2++){
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer2)->point->x();
        p.y = (*it_comeon_visualizer2)->point->y();
        p.z = (*it_comeon_visualizer2)->point->z();
        CHOSEN_PATH_points.points.push_back(p);
        /* geometry_msgs::PoseStamped CHOSEN_PATH_POINT;
        CHOSEN_PATH_POINT.pose.position.x = (*it_comeon_visualizer2)->point->x();
        CHOSEN_PATH_POINT.pose.position.y = (*it_comeon_visualizer2)->point->y();
        CHOSEN_PATH_POINT.pose.position.z = (*it_comeon_visualizer2)->point->z();
        MY_CHOSEN_PATH.poses.push_back(CHOSEN_PATH_POINT);
        CHOSEN_PATH_POINT.pose.orientation.x = 0;
        CHOSEN_PATH_POINT.pose.orientation.y = 0;
        CHOSEN_PATH_POINT.pose.orientation.z = 0;
        CHOSEN_PATH_POINT.pose.orientation.w = 0; */
        if((*it_comeon_visualizer2)->myParent != nullptr){
          if(*it_comeon_visualizer2 != *CHOSEN_PATH.begin()){
            CHOSEN_PATH_line_list.points.push_back(p);
            p.x = (*it_comeon_visualizer2)->myParent->point->x();
            p.y = (*it_comeon_visualizer2)->myParent->point->y();
            p.z = (*it_comeon_visualizer2)->myParent->point->z();
            CHOSEN_PATH_line_list.points.push_back(p);
          }
        }
      }
      }
      chosen_path_visualization_pub.publish(CHOSEN_PATH_points);
      //chosen_path_visualization_pub.publish(CHOSEN_PATH_line_list);
      
      // chosen_path_pub.publish(MY_CHOSEN_PATH);
      
      
      PATH_points.header.frame_id = PATH_line_list.header.frame_id = "world";
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
      std::list<node>::iterator it_comeon_visualizer5;
      ALL_PATH.clear();
      for(it_comeon_visualizer5 = myGoals.begin(); it_comeon_visualizer5 != myGoals.end(); it_comeon_visualizer5++){
        (*it_comeon_visualizer5).getPath(&ALL_PATH);
        ALL_PATH.push_back((&*it_comeon_visualizer5));
      }
      std::cout << ALL_PATH.size() << std::endl;
      std::list<node*>::iterator it_comeon_visualizer6;	
      for(it_comeon_visualizer6 = ALL_PATH.begin(); it_comeon_visualizer6 != ALL_PATH.end(); it_comeon_visualizer6++){
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer6)->point->x();
        p.y = (*it_comeon_visualizer6)->point->y();
        p.z = (*it_comeon_visualizer6)->point->z();
        PATH_points.points.push_back(p);
        if((*it_comeon_visualizer6)->myParent != nullptr){
          PATH_line_list.points.push_back(p);
          p.x = (*it_comeon_visualizer6)->myParent->point->x();
          p.y = (*it_comeon_visualizer6)->myParent->point->y();
          p.z = (*it_comeon_visualizer6)->myParent->point->z();
          PATH_line_list.points.push_back(p);
        }
      }
      std::cout << "har vi en path?" << std::endl;
      all_path_pub.publish(PATH_points);
      all_path_pub.publish(PATH_line_list);
      std::cout << "kommer hit? slut" << std::endl;
      }
      if(fetched_path){
      std::cout << "Kommer hit? slut.0" << std::endl;
      if((sqrt(pow(position_x - currentTarget->point->x(), 2) + pow(position_y - currentTarget->point->y(), 2) + pow(position_z - currentTarget->point->z(), 2)) < 0.5) and path_itterator != CHOSEN_PATH.end()){
        std::cout << "Kommer hit? slut.0.1" << std::endl;
        //path_itterator++;
        advance_index++;
        path_itterator = CHOSEN_PATH.begin();
        std::advance(path_itterator, advance_index);
        currentTarget = *path_itterator;
        std::cout << "Kommer hit? slut.0.2" << std::endl;
        if(path_itterator == CHOSEN_PATH.end()){
          path_itterator--;
          currentTarget = *path_itterator;
        }
        std::cout << "kommer hit? slut.0.2.0" << std::endl;
        std::cout << currentTarget << std::endl;
        std::cout << "Kommer hit? slut.0.3" << std::endl;
      }
      std::cout << "Kommer hit? slut.0.slut" << std::endl;
      if(path_itterator != CHOSEN_PATH.end()){
        std::cout << "kommer hit? slut.1" << std::endl;
        geometry_msgs::PoseStamped nextPoint;
        std::cout << "kommer hit? slut.1.1" << std::endl;
        nextPoint.pose.position.x = (currentTarget)->point->x();
        std::cout << "kommer hit? slut.1.2" << std::endl;
        nextPoint.pose.position.y = (currentTarget)->point->y();
        std::cout << "kommer hit? slut.3" << std::endl;
        nextPoint.pose.position.z = (currentTarget)->point->z();
        std::cout << "kommer hit? slut.4" << std::endl;
        nextPoint.pose.orientation.x = 0;
        nextPoint.pose.orientation.y = 0;
        nextPoint.pose.orientation.z = 0;
        nextPoint.pose.orientation.w = 0;
        nextPoint.header.stamp = ros::Time::now();
        nextPoint.header.frame_id = "world";
        chosen_path_pub.publish(nextPoint);
      }
      std::cout << "kommer hit? slut.1.3" << std::endl;
      }
      std::cout << "kommer hit? slut.2" << std::endl;
      GOAL_points.header.frame_id = "world";
      GOAL_points.ns = "points";
      GOAL_points.action = visualization_msgs::Marker::ADD;
      GOAL_points.pose.orientation.w = 1.0;
      GOAL_points.id = 0;
      GOAL_points.type = visualization_msgs::Marker::POINTS;
      GOAL_points.scale.x = 0.2;
      GOAL_points.scale.y = 0.2;
      GOAL_points.color.r = 1.0f;
      GOAL_points.color.a = 1.0;
      std::list<node>::iterator it_comeon_visualizer3;	
      for(it_comeon_visualizer3 = myGoals.begin(); it_comeon_visualizer3 != myGoals.end(); it_comeon_visualizer3++){
        geometry_msgs::Point p;
        p.x = it_comeon_visualizer3->point->x();
        p.y = it_comeon_visualizer3->point->y();
        p.z = it_comeon_visualizer3->point->z();
        GOAL_points.points.push_back(p);
      }
      goal_pub.publish(GOAL_points);
      std::cout << "kommer hit? slut.3" << std::endl;
      
      hits.clear();
      std::cout << "kommer hit? slut.4" << std::endl;
      goalNode->addHits(&hits);
      std::cout << "kommer hit? slut.5" << std::endl;
      HITS_points.header.frame_id = "world";
      HITS_points.ns = "points";
      HITS_points.action = visualization_msgs::Marker::ADD;
      HITS_points.pose.orientation.w = 1.0;
      HITS_points.id = 0;
      HITS_points.type = visualization_msgs::Marker::POINTS;
      HITS_points.scale.x = 0.2;
      HITS_points.scale.y = 0.2;
      HITS_points.color.r = 1.0f;
      HITS_points.color.a = 1.0;
      std::list<ufo::math::Vector3>::iterator it_comeon_visualizer4;	
      for(it_comeon_visualizer4 = hits.begin(); it_comeon_visualizer4 != hits.end(); it_comeon_visualizer4++){
        geometry_msgs::Point p;
        p.x = it_comeon_visualizer4->x();
        p.y = it_comeon_visualizer4->y();
        p.z = it_comeon_visualizer4->z();
        HITS_points.points.push_back(p);
      }
      hits_pub.publish(HITS_points);
      /*
      visualization_msgs::Marker TAKEN_PATH_points, TAKEN_PATH_line_list;
      TAKEN_PATH_points.header.frame_id = TAKEN_PATH_line_list.header.frame_id = "world"; //MY_CHOSEN_PATH.header.frame_id = "pelican/velodyne";
      TAKEN_PATH_points.ns = "points";
      TAKEN_PATH_points.action = visualization_msgs::Marker::ADD;
      TAKEN_PATH_points.pose.orientation.w = 1.0;
      TAKEN_PATH_points.id = 0;
      TAKEN_PATH_line_list.id = 1;
      TAKEN_PATH_points.type = visualization_msgs::Marker::POINTS;
      TAKEN_PATH_line_list.type = visualization_msgs::Marker::LINE_LIST;
      TAKEN_PATH_points.scale.x = 0.2;
      TAKEN_PATH_points.scale.y = 0.2;
      TAKEN_PATH_line_list.scale.x = 0.1;
      TAKEN_PATH_points.color.g = 1.0f;
      TAKEN_PATH_points.color.a = 1.0;
      TAKEN_PATH_line_list.color.b = 1.0;
      TAKEN_PATH_line_list.color.a = 1.0;
      if(VISITED_POINTS.empty() and position_received){
        geometry_msgs::Point p;
        p.x = position_x;
        p.y = position_y;
        p.z = position_z;
        VISITED_POINTS.push_back(p);
      }else{
        std::list<geometry_msgs::Point>::iterator taken_path_visualizer;
        taken_path_visualizer = VISITED_POINTS.end();
        taken_path_visualizer--;
        if(sqrt(pow((*taken_path_visualizer).x - position_x, 2) + pow((*taken_path_visualizer).y - position_y, 2) + pow((*taken_path_visualizer).z - position_z, 2)) >= 1.0){
          geometry_msgs::Point p;
          p.x = position_x;
          p.y = position_y;
          p.z = position_z;
          VISITED_POINTS.push_back(p);
        }
      }
      std::list<geometry_msgs::Point>::iterator taken_path_visualizer;	
      for(taken_path_visualizer = VISITED_POINTS.begin(); taken_path_visualizer != VISITED_POINTS.end(); taken_path_visualizer++){
        TAKEN_PATH_points.points.push_back(*taken_path_visualizer);
      }
      taken_path_pub.publish(TAKEN_PATH_points);
      taken_path_pub.publish(TAKEN_PATH_line_list);*/
      
    std::cout << "kommer hit? slut.6" << std::endl;
    itterations++;
    if(itterations > 10){
      itterations = 0;
      fetched_path = false;
      RRT_created = false;
      GOALS_generated = false;
      position_received = false;
    }
    std::cout << "kommer hit? slut.7" << std::endl;
    }
    ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
    bool compress = false;
    ufo::map::DepthType pub_depth = 0;
    // Convert UFOMap to ROS message
    std::cout << "kommer hit? slut.8" << std::endl;
    if (ufomap_msgs::ufoToMsg(myMap, msg->map, compress, pub_depth)) {
      //std::cout << "Map conversion success!" << std::endl;
      // Conversion was successful
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "world";
      map_pub.publish(msg);					        
    }else{
      std::cout << "Map conversion failed!" << std::endl;
    }
    std::cout << "kommer hit? slut.slut" << std::endl;
    ros::spinOnce();
    std::cout << "kommer hit? slut.slut.slut\n" << std::endl;
    if(goalNode != nullptr){
      for(std::list<node*>::iterator path_itterator_helper = CHOSEN_PATH.begin(); path_itterator_helper != CHOSEN_PATH.end();){
        std::cout << "target: " << (*path_itterator_helper)->point->x() << ", " << (*path_itterator_helper)->point->y() << ", " << (*path_itterator_helper)->point->z() << std::endl;
        path_itterator_helper++;
      }
      std::cout << "This is total distance: " << totalDistance << std::endl;
      std::cout << "This is sum distance: " << goalNode->sumDistance() << std::endl;
    }
    if(position_received){
      visualization_msgs::Marker POSITION_point;
      POSITION_point.header.frame_id = "world";
      POSITION_point.ns = "points";
      POSITION_point.action = visualization_msgs::Marker::ADD;
      POSITION_point.pose.orientation.w = 1.0;
      POSITION_point.id = 0;
      POSITION_point.type = visualization_msgs::Marker::POINTS;
      POSITION_point.scale.x = 0.2;
      POSITION_point.scale.y = 0.2;
      POSITION_point.color.g = 1.0f;
      POSITION_point.color.a = 1.0;
      geometry_msgs::Point p;
      p.x = position_x;
      p.y = position_y;
      p.z = position_z;
      POSITION_point.points.push_back(p);
      position_pub.publish(POSITION_point);
    }
    std::cout << "This is my position: " << position_x << ", " << position_y << ", " << position_z << std::endl;
    if(currentTarget != nullptr){
      std::cout << "This is my target: " << currentTarget->point->x() << ", " << currentTarget->point->y() << ", " << currentTarget->point->z() << std::endl;
      std::cout << sqrt(pow(position_x - currentTarget->point->x(), 2) + pow(position_y - currentTarget->point->y(), 2) + pow(position_z - currentTarget->point->z(), 2)) << " < " << 0.5 << std::endl;
    }
    std::cout << advance_index << std::endl;
    rate.sleep();
  }
  return 0;
}
