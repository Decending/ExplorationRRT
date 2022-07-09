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
#include <tf/tf.h>

using namespace std::chrono;
using namespace std;
using namespace std::this_thread;

struct rrtSolverStatus rrt_solve(struct rrtCache *instance,
                                 double *u,
                                 const double *params,
                                 const double *y0,
                                 const double *c0);
                                 
struct rrtCache *rrt_new(void);

struct node{
   public:
      ufo::math::Vector3* point;
      node* myParent = nullptr;
      double distanceToParent = -1;
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
        if(myParent != nullptr and distanceToParent != -1){
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
            if(myParent->point->x() != point->x() or myParent->point->y() != point->y() or myParent->point->z() != point->z()){ 
              givenPath->push_back(new node(myParent->point->x(), myParent->point->y(), myParent->point->z()));
            }
          }
        }
        if(myParent == nullptr){
          return;
        }
        return;
      }
      
      int findInformationGain(float SCALER_AABB, ufo::map::OccupancyMapColor const& map, bool excludePath){
        if(myHits.empty()){
          //Setting up targets, X / Y targets in either positive (P) or negative (N) direction
          ufo::math::Vector3 targetXP(point->x() + 1, point->y(), point->z());
          ufo::math::Vector3 targetXN(point->x() - 1, point->y(), point->z());
          ufo::math::Vector3 targetYP(point->x(), point->y() + 1, point->z());
          ufo::math::Vector3 targetYN(point->x(), point->y() - 1, point->z());
          ufo::math::Vector3 upwards(point->x(), point->y(), point->z() + 1);
        
          //Angles
          double vertical_angle = 0.393;
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
          for (auto it = map.beginLeaves(frustXP, false, false, true, false, 3), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 3)){
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustXN, false, false, true, false, 3), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 3)){
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustYP, false, false, true, false, 3), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 3)){
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustYN, false, false, true, false, 3), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 3)){
              myHits.push_back(end_point);
            }
          }
          //std::cout << "My checks: " << checks << std::endl;
          if(myParent != nullptr and not excludePath){
            myParent->findInformationGain(SCALER_AABB, map, excludePath);
          }
        }else{
          std::list<ufo::math::Vector3>::iterator it_hits;	
          for(it_hits = myHits.begin(); it_hits != myHits.end();){
            // std::cout << "kommer hit? 2.4.1" << std::endl;
            ufo::geometry::LineSegment myLine2(*point, *it_hits);
            if(isInCollision(map, myLine2, true, false, false, 3)){
              // std::cout << "kommer hit? 2.4.2" << std::endl;
              it_hits = myHits.erase(it_hits);
            }else{
              // std::cout << "kommer hit? 2.4.3" << std::endl;
              if(isInCollision(map, *it_hits, false, true, false, 3)){
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
        if(targetNode == this){
          return false;
        }
        if(myParent != nullptr){
          //std::cout << "kommer hit? 2.3.1" << std::endl;
          improvementFound = myParent->findPathImprovement(targetNode, map, givenDistance, givenRadious);
        }else{
          ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
          //std::cout << "kommer hit? 2.3.2" << std::endl;
          if(!isInCollision(map, myLine, true, false, true, 3)){
            ufo::math::Vector3 newVector(targetNode->point->x() - point->x(), targetNode->point->y() - point->y(), targetNode->point->z() - point->z());
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
            //std::cout << "kommer hit? 2.3.3" << std::endl;
            for(int i = 1; i < itterations; i++){
              //std::cout << "\nnewPoints x, y, z: " << parent->point->x() << ", " << parent->point->y() << ", " << parent->point->z() << std::endl;
              ufo::math::Vector3 newVector = ufo::math::Vector3(point->x() + i * xStep, point->y() + i * yStep, point->z() + i * zStep);
              ufo::geometry::Sphere new_sphere(newVector, givenRadious);
              if(isInCollision(map, new_sphere, true, false, true, 3)){
                return false;
              }
              //std::cout << "newPoints x, y, z: " << newPoint->point->x() << ", " << newPoint->point->y() << ", " << newPoint->point->z() << std::endl;
              // newPoint->addParent(parent);
              //std::cout << "newPoints x, y, z: " << newPoint->myParent->point->x() << ", " << newPoint->myParent->point->y() << ", " << newPoint->myParent->point->z() << std::endl;
              // parent = newPoint;
              // RRT_TREE.push_back(*newPoint);
            }
            //std::cout << "kommer hit? 2.3.4" << std::endl;
            targetNode->addParent(this);
            return true;
          }else{
            return false;
          };
        }
        if(!improvementFound){
          ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
          if(!isInCollision(map, myLine, true, false, true, 3)){
            ufo::math::Vector3 newVector(targetNode->point->x() - point->x(), targetNode->point->y() - point->y(), targetNode->point->z() - point->z());
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
            //std::cout << "kommer hit? 2.3.3" << std::endl;
            for(int i = 1; i < itterations; i++){
              //std::cout << "\nnewPoints x, y, z: " << parent->point->x() << ", " << parent->point->y() << ", " << parent->point->z() << std::endl;
              ufo::math::Vector3 newVector = ufo::math::Vector3(point->x() + i * xStep, point->y() + i * yStep, point->z() + i * zStep);
              ufo::geometry::Sphere new_sphere(newVector, givenRadious);
              if(isInCollision(map, new_sphere, true, false, true, 3)){
                return false;
              }
              //std::cout << "newPoints x, y, z: " << newPoint->point->x() << ", " << newPoint->point->y() << ", " << newPoint->point->z() << std::endl;
              // newPoint->addParent(parent);
              //std::cout << "newPoints x, y, z: " << newPoint->myParent->point->x() << ", " << newPoint->myParent->point->y() << ", " << newPoint->myParent->point->z() << std::endl;
              // parent = newPoint;
              // RRT_TREE.push_back(*newPoint);
            }
            //std::cout << "kommer hit? 2.3.5" << std::endl;
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
      
      void readyForDeletion(){
        delete point;
      }
      
      bool isInCollision(ufo::map::OccupancyMapColor const& map, 
                   ufo::geometry::BoundingVar const& bounding_volume, 
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = 3){
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
float DISTANCE_BETWEEN_NODES = 0.5; // 1.0;
float DISTANCE_BETWEEN_GOALS = 0.4;
float MINIMUM_DISTANCE_TO_GOAL = 0.5;
bool RUN_BY_NODES = true;
double SENSOR_RANGE = 2;
double SCALER_INFORMATION_GAIN = 0.1;
double SCALER_DISTANCE = 4;
int itterations;
int STEP_LENGTH = 1;
float SCALER_AABB = 20;
float SCALER_X = SCALER_AABB;
float SCALER_Y = SCALER_AABB;
float SCALER_Z = SCALER_AABB;
double radius = 1.0; // 0.5;
bool map_received = false;
bool RRT_created = false;
bool GOALS_generated = false;
bool position_received = false;
bool fetched_path = false;
bool newPath = false;
bool allowNewPath = true;
bool recoveryUnderway = false;
float position_x = 0;
float position_y = 0;
float position_z = 0;
float velocity_x = 0;
float velocity_y = 0;
float velocity_z = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;
double arbitraryDistance = 0.5;
node* goalNode = nullptr;
node* reserveGoalNode = nullptr;
float lowest_x;
float lowest_y;
float lowest_z;
float highest_x;
float highest_y;
float highest_z;
float averageInfo = 100;
float initialGoalInfo = 0.0;
int averageInfoCounter = 0;
double totalCost = std::numeric_limits<float>::max();
double totalDistance = -1;
int advance_index = 0;


// Create a colored UFOMap
ufo::map::OccupancyMapColor myMap(0.4);
std::list<struct node*> RRT_TREE{};
std::list<struct node*> CHOSEN_PATH{};
std::list<struct node*> ALL_PATH{};
std::list<struct node*> myGoals{};
std::list<struct node*> myReserveGoals{};
std::list<ufo::math::Vector3> hits{};
std::list<node*> VISITED_POINTS{};
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
    RRT_TREE.push_back(newPoint);
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

void tuneGeneration(ufo::map::OccupancyMapColor const& map, bool occupied_space, bool free_space, bool unknown_space, float given_x, float given_y, float given_z, ufo::map::DepthType min_depth = 3){
  highest_x = -9999;
  highest_y = -9999;
  highest_z = -9999;
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();
  ufo::math::Vector3 minPoint(given_x - 1 * SCALER_AABB, given_y - 1 * SCALER_AABB, given_z - 1 * SCALER_AABB);
  ufo::math::Vector3 maxPoint(given_x + 1 * SCALER_AABB, given_y + 1 * SCALER_AABB, given_z + 1 * SCALER_AABB);
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
                   bool unknown_space = false, ufo::map::DepthType min_depth = 3)
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
  for(std::list<node*>::iterator it_goals = myGoals.begin(); it_goals != myGoals.end(); it_goals++){
    // std::cout << "är det här? 0" << std::endl;
    struct node* chosenNode = nullptr;
    double distance = std::numeric_limits<double>::max();
    for(std::list<node*>::iterator it_RRT = RRT_TREE.begin(); it_RRT != RRT_TREE.end(); it_RRT++){
      // std::cout << "är det här? 1" << std::endl;
      double distanceNodeToGoal = sqrt(pow((*it_RRT)->point->x() - (*it_goals)->point->x(), 2) + pow((*it_RRT)->point->y() - (*it_goals)->point->y(), 2) + pow((*it_RRT)->point->z() - (*it_goals)->point->z(), 2));
      double distanceToNode = (*it_RRT)->sumDistance();
      double totalDistance = distanceNodeToGoal + distanceToNode;
      // std::cout << "är det här? 2" << std::endl;
      if(totalDistance < distance){
        // std::cout << "är det här? 3" << std::endl;
        ufo::geometry::LineSegment myLine(*((*it_goals)->point), *((*it_RRT)->point));
        if(!isInCollision(myMap, myLine, true, false, true, 3)){
          ufo::math::Vector3 newVector((*it_goals)->point->x() - (*it_RRT)->point->x(), (*it_goals)->point->y() - (*it_RRT)->point->y(), (*it_goals)->point->z() - (*it_RRT)->point->z());
          //std::cout << "my newVector: " << newVector.x() << ", " << newVector.y() << ", " << newVector.z() << std::endl;
          float distance_calc = newVector.norm();
          float itterations = (distance_calc / radius);
          //std::cout << "my distance: " << distance << std::endl;
          //std::cout << "my givenDistance: " << givenDistance << std::endl;
          //std::cout << "my itterations: " << itterations << std::endl;
          float part = radius / distance_calc;
          float xStep = ((*it_goals)->point->x() - (*it_RRT)->point->x()) * part;
          float yStep = ((*it_goals)->point->y() - (*it_RRT)->point->y()) * part;
          float zStep = ((*it_goals)->point->z() - (*it_RRT)->point->z()) * part;
          // node* parent = givenNode->myParent;
          // node* nextNode = givenNode->myParent;
          //std::cout << "kommer hit? 2.3.3" << std::endl;
          bool add = true;
          for(int i = 1; i < itterations; i++){
            //std::cout << "\nnewPoints x, y, z: " << parent->point->x() << ", " << parent->point->y() << ", " << parent->point->z() << std::endl;
            ufo::math::Vector3 newVector = ufo::math::Vector3((*it_RRT)->point->x() + i * xStep, (*it_RRT)->point->y() + i * yStep, (*it_RRT)->point->z() + i * zStep);
            ufo::geometry::Sphere new_sphere(newVector, radius);
            if(isInCollision(myMap, new_sphere, true, false, true, 3)){
              add = false;
              break;
            }
            //std::cout << "newPoints x, y, z: " << newPoint->point->x() << ", " << newPoint->point->y() << ", " << newPoint->point->z() << std::endl;
            // newPoint->addParent(parent);
            //std::cout << "newPoints x, y, z: " << newPoint->myParent->point->x() << ", " << newPoint->myParent->point->y() << ", " << newPoint->myParent->point->z() << std::endl;
            // parent = newPoint;
            // RRT_TREE.push_back(*newPoint);
          }
          // std::cout << "är det här? 4" << std::endl;
          if(add){
            distance = totalDistance;
            chosenNode = *it_RRT;
          }
        }
      }
    }
    // std::cout << "är det här? 5" << std::endl;
    if(chosenNode != nullptr){
      (*it_goals)->addParent(chosenNode);
      // std::cout << "är det här? 6" << std::endl;
      chosenNode->addChild(*it_goals);
      // std::cout << "är det här? 7" << std::endl;
    }
  }
}

void generateGoals(ufo::map::OccupancyMapColor const& map, bool evaluateOldGoals){
  // Generate goals, check for occupancy status
  if(!myGoals.empty() and evaluateOldGoals){ //Evaluate old goals and save the best one for backtracking
    double newCost = std::numeric_limits<float>::max();
    double totalCost = std::numeric_limits<float>::max();
    for(std::list<node*>::iterator it_goal = myGoals.begin(); it_goal != myGoals.end(); it_goal++){
      // std::cout << "kommer hit? 2.1" << std::endl;
      if((*it_goal)->myParent != nullptr){
        // std::cout << "Test 1" << std::endl;
        //(*it_goal)->findPathImprovement(*it_goal, myMap, DISTANCE_BETWEEN_NODES, radius);
        // std::cout << "Test 2" << std::endl;
        //std::cout << "kommer hit? 2.2 ------------------------------------------------------------" << std::endl;
        //linSpace(*it_goal, DISTANCE_BETWEEN_NODES);
        //std::cout << "kommer hit? 2.3" << std::endl;
        //(*it_goal)->findPathImprovement(*it_goal, myMap, DISTANCE_BETWEEN_NODES, radius);
        //std::cout << "kommer hit? 2.4 ------------------------------------------------------------" << std::endl;
        //std::cout << "Krashar efter cost calc?" << std::endl;
        double distanceCost = (*it_goal)->sumDistance() * SCALER_DISTANCE;
        // std::cout << "Test 2.1" << std::endl;
        (*it_goal)->clearInformationGain();
        double informationGain = SCALER_INFORMATION_GAIN * ((*it_goal)->findInformationGain(SCALER_AABB, myMap, true));
        // std::cout << "Test 2.2" << std::endl;
        //linSpace(*it_goal, DISTANCE_BETWEEN_NODES);
        // std::cout << "Test 2.3" << std::endl;
        
        //The battleground:
        typedef rrtCache* (*arbitrary)();
        typedef rrtSolverStatus (*arbitrary2)(void*, double*, double*, double, double*);
        typedef void (*rrt_clearer)(rrtCache*);
        int i = 0;
        double p[159] = {0};
        std::list<double> xref = {};
        std::list<double> x0 = {position_x, position_y, position_z, velocity_x, velocity_y, velocity_z, roll, pitch};
        // Current position
        p[0] = position_x;
        p[1] = position_y;
        p[2] = position_z;
        p[3] = velocity_x;
        p[4] = velocity_y;
        p[5] = velocity_z;
        p[6] = roll;
        p[7] = pitch;
        // std::cout << "Trajectory" << std::endl;
        // Trajectory
        std::list<struct node*> EVALUATE_PATH{};
        EVALUATE_PATH.clear();
        // std::cout << "Get path" << std::endl;
        (*it_goal)->getPath(&EVALUATE_PATH);
        // std::cout << "get path finished!" << std::endl;
        EVALUATE_PATH.push_back(new node((*it_goal)->point->x(), (*it_goal)->point->y(), (*it_goal)->point->z()));
        // std::cout << "print 1" << std::endl;
        xref.push_back((*it_goal)->point->x());
        // std::cout << "print 2" << std::endl;
        xref.push_back((*it_goal)->point->y());
        // std::cout << "print 3" << std::endl;
        xref.push_back((*it_goal)->point->z()); 
        // std::cout << "print 4" << std::endl;
        std::list<node*>::iterator it_evaluator = EVALUATE_PATH.begin();
        for (i = 1; i < 50; ++i){
          // std::cout << i << std::endl;
          p[8+3*i] = (*it_evaluator)->point->x();
          xref.push_back((*it_evaluator)->point->x());
          p[8+3*i+1] = (*it_evaluator)->point->y();
          xref.push_back((*it_evaluator)->point->y());
          p[8+3*i+2] = (*it_evaluator)->point->z();
          xref.push_back((*it_evaluator)->point->z());
          if(it_evaluator != --EVALUATE_PATH.end()){
            it_evaluator++;
          }
        }
        // std::cout << "print 5" << std::endl;
        p[158] = 0.5;
        
        /*std::cout << "This is path: " << std::endl;
        for(i = 0; i < 51; i++){
          std::cout << p[8*i] << ", " << p[8*i + 1] << ", " << p[8*i + 2] << std::endl;
        }
        std::cout << "This is goal: " << std::endl;
        std::cout << (*it_goal)->point->x() << ", " << (*it_goal)->point->y() << ", " << (*it_goal)->point->z() << std::endl;*/
  
        double u[150] = {0};

        for (i = 0; i < 50; ++i) {
          // std::cout << i << std::endl; 
          u[3*i] = 9.81;
          u[3*i + 1] = 0;
          u[3*i + 2] = 0;
        }

        double init_penalty = 0;
        void *handle = dlopen("./MAV/rrt/target/release/librrt.so", RTLD_LAZY);
        if (!handle) {
          fprintf(stderr, "%s\n", dlerror());
          exit(EXIT_FAILURE);
        }
        arbitrary rrt_new;
        *(void **) (&rrt_new) = dlsym(handle, "rrt_new");
        // std::cout << rrt_new << std::endl;
        rrtCache* cache = rrt_new();
        // std::cout << cache << std::endl;
        arbitrary2 rrt_solve;
        *(void **) (&rrt_solve) = dlsym(handle, "rrt_solve");
        rrt_clearer rrt_free;
        *(void **) (&rrt_free) = dlsym(handle, "rrt_free");
        // std::cout << rrt_free << std::endl;
        std::cout << init_penalty << std::endl;
        rrtSolverStatus status = rrt_solve(cache, u, p, 0, &init_penalty);
        
        std::list<double> uold = {9.81,0.0,0.0};
        std::list<double> uref = {9.81,0.0,0.0};
        
        std::list<double> x_hist;
        std::list<double> p_hist;
        double cost;
        std::tuple<std::list<double>, double, std::list<double>> trajectory(std::list<double> x, double* u, double N, double dt, std::list<double> nmpc_ref);
        std::tie(x_hist, cost, p_hist) = trajectory(x0, u, 50, 0.5, xref);
        xref.clear();
        rrt_free(cache);
        // std::cout << "This is cost: " << cost << std::endl;
        // std::cout << "This is distance cost: " << distanceCost << std::endl;
        // std::cout << "This is informationGain: " << informationGain << std::endl; */
        // newCost = distanceCost + 0.2 * cost - informationGain;
        newCost = distanceCost - informationGain;
        if(informationGain > initialGoalInfo){
          initialGoalInfo = informationGain;
        }
        //std::cout << "kommer hit? 2.5" << std::endl;
        //std::cout << "Test 3" << std::endl;
        //std::cout << "Test 4" << std::endl;
        //std::cout << "naej" << std::endl;
        //std::cout << "My hits = " << it_goal->findInformationGain(SCALER_AABB, myMap) << std::endl;
        /*if((*it_goal) == goalNode){
          std::cout << (*it_goal)->sumDistance() << " is my sum distance and the following is my total distance  " << 0.2 * totalDistance << std::endl;
        };*/
        int stickyCounter = 0;
        for(std::list<ufo::math::Vector3>::iterator it_floor = (*it_goal)->myHits.begin(); it_floor != (*it_goal)->myHits.end(); it_floor++){
          if(it_floor->z() < (*it_goal)->point->z()){
            stickyCounter++;
          }
        }
        bool infoRequirement = ((*it_goal)->myHits.size() > 0.2 * averageInfo or averageInfoCounter < 6);
        bool stickyFloor = ((stickyCounter < 0.8 * (*it_goal)->myHits.size()) or averageInfoCounter < 6);
        // std::cout << "This is requirements: " << (newCost < totalCost) << ((*it_goal)->findInformationGain(SCALER_AABB, myMap) > 0) << (stickyFloor and infoRequirement) << std::endl;
        if((newCost < totalCost) and ((*it_goal)->findInformationGain(SCALER_AABB, myMap, false) > 0) and (stickyFloor and infoRequirement) and (*it_goal)->myParent != nullptr){
          // std::cout << newCost << " < " << totalCost << ", " << (*it_goal)->sumDistance() << " < " << 0.2 * totalDistance << std::endl;
          totalCost = newCost;
          reserveGoalNode = *it_goal;
          // std::cout << p_hist << std::endl;
          // initialGoalInfo = goalNode->myHits.size();
        }
      }
    }
    if(reserveGoalNode != nullptr){
      myReserveGoals.push_back(reserveGoalNode);
      std::cout << "testy test" << std::endl;
      for(std::list<node*>::iterator it_parent_finder = --VISITED_POINTS.end(); it_parent_finder != VISITED_POINTS.begin(); it_parent_finder--){
        ufo::geometry::LineSegment myLine((*(*it_parent_finder)->point), (*reserveGoalNode->point));
        std::cout << "oh, we be printin!" << std::endl;
        if(!isInCollision(map, myLine, true, false, true, 3)){
          reserveGoalNode->addParent((*it_parent_finder));
          break;
        }
      }
    std::cout << "got out" << std::endl;
    }
  }
  if(goalNode != nullptr){
    // std::cout << "Goalnode is not nullptr!" << std::endl;
    if(sqrt(pow(position_x - goalNode->point->x(), 2) + pow(position_y - goalNode->point->y(), 2) + pow(position_z - goalNode->point->z(), 2)) > 0.5){
      std::list<node*>::iterator it_goal2;
      // std::cout << "this is my goal size: " << myGoals.size() << std::endl;
      int help_counter = 0;
      for(it_goal2 = myGoals.begin(); it_goal2 != myGoals.end(); it_goal2++){
        // std::cout << help_counter << std::endl;
        help_counter++;
        if(*it_goal2 != goalNode and *it_goal2 != reserveGoalNode){
          // std::cout << (*it_goal2)->point->x() << std::endl;
          (*it_goal2)->readyForDeletion();
          delete(*it_goal2);
        }
      }
      // std::cout << "Big boy" << std::endl;
      // std::cout << goalNode->point->x() << std::endl;
      myGoals.clear();
      myGoals.push_back(goalNode);
    }else{
      // std::cout << "Deleting goals!" << std::endl;
      std::list<node*>::iterator it_goal2;
      for(it_goal2 = myGoals.begin(); it_goal2 != myGoals.end(); it_goal2++){
        if(*it_goal2 != reserveGoalNode){
          (*it_goal2)->readyForDeletion();
          delete(*it_goal2);
        }
      }
      // std::cout << "this is my goal size: " << myGoals.size() << std::endl;
      goalNode = nullptr;
      reserveGoalNode = nullptr;
      myGoals.clear();
      allowNewPath = true;
    }
  }
  ufo::math::Vector3 goal;
  srand(time(0));
  itterations = 0;
  while((myGoals.size() < NUMBER_OF_GOALS) and (itterations < 10000)){
    float x = lowest_x + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_X;
    float y = lowest_y + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Y;
    float z = lowest_z + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Z;
    ufo::math::Vector3 goal(x, y, z);
    ufo::geometry::Sphere goal_sphere(goal, radius);
    if(sqrt(pow(position_x - x, 2) + pow(position_y - y, 2) + pow(position_z - z, 2)) > MINIMUM_DISTANCE_TO_GOAL){
      if(!isInCollision(myMap, goal_sphere, true, false, true, 3) and isInCollision(myMap, goal_sphere, false, true, false, 3)){
        bool add = true;
        for(std::list<node*>::iterator it_goal = myGoals.begin(); it_goal != myGoals.end(); it_goal++){
          if(sqrt(pow((*it_goal)->point->x() - x, 2) + pow((*it_goal)->point->y() - y, 2) + pow((*it_goal)->point->z() - z, 2)) < DISTANCE_BETWEEN_GOALS){
            add = false;
            break;
          }
        }
        if(add){
          ufo::math::Vector3 min_point(x - SENSOR_RANGE, y - SENSOR_RANGE, z - SENSOR_RANGE);
          ufo::math::Vector3 max_point(x + SENSOR_RANGE, y + SENSOR_RANGE, z + SENSOR_RANGE);
          ufo::math::Vector3 position(position_x, position_y, position_z);
          ufo::geometry::AABB aabb(min_point, max_point);
          for (auto it = map.beginLeaves(aabb, false, false, true, false, 3), it_end = map.endLeaves(); it != it_end; ++it) {
            if(it.isUnknown()){
              ufo::math::Vector3 unknownNode(it.getX(), it.getY(), it.getZ());
              ufo::geometry::LineSegment myLine(goal, unknownNode);
              if(!isInCollision(map, myLine, true, false, false, 3)){
                node* newGoal = new node(x, y, z);
                myGoals.push_back(newGoal);
                break;
              }
            }
          // break;
          }
        }
      };
      itterations++;
    }
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
    if(sqrt(pow(position_x - goalNode->point->x(), 2) + pow(position_y - goalNode->point->y(), 2) + pow(position_z - goalNode->point->z(), 2)) < 0.5){
      allowNewPath = true;
      totalCost = std::numeric_limits<float>::max();
    }else{
      totalCost = goalNode->sumDistance() * SCALER_DISTANCE - SCALER_INFORMATION_GAIN * (goalNode->findInformationGain(SCALER_AABB, myMap, false));
    }
  }else{
    totalCost = std::numeric_limits<float>::max();
  }
  double newCost = std::numeric_limits<float>::max();
  if(allowNewPath){
    std::list<double> PATH_CONTAINER{};
    initialGoalInfo = 0;
    for(std::list<node*>::iterator it_goal = myGoals.begin(); it_goal != myGoals.end(); it_goal++){
      //std::cout << "kommer hit? 2.1" << std::endl;
      if((*it_goal)->myParent != nullptr){
        // std::cout << "Test 1" << std::endl;
        (*it_goal)->findPathImprovement(*it_goal, myMap, DISTANCE_BETWEEN_NODES, radius);
        // std::cout << "Test 2" << std::endl;
        //std::cout << "kommer hit? 2.2 ------------------------------------------------------------" << std::endl;
        //linSpace(*it_goal, DISTANCE_BETWEEN_NODES);
        //std::cout << "kommer hit? 2.3" << std::endl;
        //(*it_goal)->findPathImprovement(*it_goal, myMap, DISTANCE_BETWEEN_NODES, radius);
        //std::cout << "kommer hit? 2.4 ------------------------------------------------------------" << std::endl;
        //std::cout << "Krashar efter cost calc?" << std::endl;
        double distanceCost = (*it_goal)->sumDistance() * SCALER_DISTANCE;
        // std::cout << "Test 2.1" << std::endl;
        double informationGain = SCALER_INFORMATION_GAIN * ((*it_goal)->findInformationGain(SCALER_AABB, myMap, false));
        // std::cout << "Test 2.2" << std::endl;
        linSpace(*it_goal, DISTANCE_BETWEEN_NODES);
        // std::cout << "Test 2.3" << std::endl;
        
        //The battleground:
        typedef rrtCache* (*arbitrary)();
        typedef rrtSolverStatus (*arbitrary2)(void*, double*, double*, double, double*);
        typedef void (*rrt_clearer)(rrtCache*);
        int i = 0;
        double p[159] = {0};
        std::list<double> xref = {};
        std::list<double> x0 = {position_x, position_y, position_z, velocity_x, velocity_y, velocity_z, roll, pitch};
        // Current position
        p[0] = position_x;
        p[1] = position_y;
        p[2] = position_z;
        p[3] = velocity_x;
        p[4] = velocity_y;
        p[5] = velocity_z;
        p[6] = roll;
        p[7] = pitch;
        // std::cout << "Trajectory" << std::endl;
        // Trajectory
        std::list<struct node*> EVALUATE_PATH{};
        EVALUATE_PATH.clear();
        // std::cout << "Get path" << std::endl;
        (*it_goal)->getPath(&EVALUATE_PATH);
        // std::cout << "get path finished!" << std::endl;
        EVALUATE_PATH.push_back(new node((*it_goal)->point->x(), (*it_goal)->point->y(), (*it_goal)->point->z()));
        // std::cout << "print 1" << std::endl;
        xref.push_back((*it_goal)->point->x());
        // std::cout << "print 2" << std::endl;
        xref.push_back((*it_goal)->point->y());
        // std::cout << "print 3" << std::endl;
        xref.push_back((*it_goal)->point->z()); 
        // std::cout << "print 4" << std::endl;
        std::list<node*>::iterator it_evaluator = EVALUATE_PATH.begin();
        for (i = 1; i < 50; ++i){
          // std::cout << i << std::endl;
          p[8+3*i] = (*it_evaluator)->point->x();
          xref.push_back((*it_evaluator)->point->x());
          p[8+3*i+1] = (*it_evaluator)->point->y();
          xref.push_back((*it_evaluator)->point->y());
          p[8+3*i+2] = (*it_evaluator)->point->z();
          xref.push_back((*it_evaluator)->point->z());
          if(it_evaluator != --EVALUATE_PATH.end()){
            it_evaluator++;
          }
        }
        // std::cout << "print 5" << std::endl;
        p[158] = 0.5;
        
        /*std::cout << "This is path: " << std::endl;
        for(i = 0; i < 51; i++){
          std::cout << p[8*i] << ", " << p[8*i + 1] << ", " << p[8*i + 2] << std::endl;
        }
        std::cout << "This is goal: " << std::endl;
        std::cout << (*it_goal)->point->x() << ", " << (*it_goal)->point->y() << ", " << (*it_goal)->point->z() << std::endl;*/
  
        double u[150] = {0};

        for (i = 0; i < 50; ++i) {
          // std::cout << i << std::endl; 
          u[3*i] = 9.81;
          u[3*i + 1] = 0;
          u[3*i + 2] = 0;
        }

        double init_penalty = 0;
        void *handle = dlopen("./MAV/rrt/target/release/librrt.so", RTLD_LAZY);
        if (!handle) {
          fprintf(stderr, "%s\n", dlerror());
          exit(EXIT_FAILURE);
        }
        arbitrary rrt_new;
        *(void **) (&rrt_new) = dlsym(handle, "rrt_new");
        // std::cout << rrt_new << std::endl;
        rrtCache* cache = rrt_new();
        // std::cout << cache << std::endl;
        arbitrary2 rrt_solve;
        *(void **) (&rrt_solve) = dlsym(handle, "rrt_solve");
        rrt_clearer rrt_free;
        *(void **) (&rrt_free) = dlsym(handle, "rrt_free");
        // std::cout << rrt_free << std::endl;
        std::cout << init_penalty << std::endl;
        rrtSolverStatus status = rrt_solve(cache, u, p, 0, &init_penalty);
        
        std::list<double> uold = {9.81,0.0,0.0};
        std::list<double> uref = {9.81,0.0,0.0};
        
        std::list<double> x_hist;
        std::list<double> p_hist;
        double cost;
        std::tuple<std::list<double>, double, std::list<double>> trajectory(std::list<double> x, double* u, double N, double dt, std::list<double> nmpc_ref);
        std::tie(x_hist, cost, p_hist) = trajectory(x0, u, 50, 0.5, xref);
        xref.clear();
        rrt_free(cache);
        // std::cout << "This is cost: " << cost << std::endl;
        // std::cout << "This is distance cost: " << distanceCost << std::endl;
        // std::cout << "This is informationGain: " << informationGain << std::endl; */
        // newCost = distanceCost + 0.2 * cost - informationGain;
        newCost = distanceCost - informationGain;
        if(informationGain > initialGoalInfo){
          initialGoalInfo = informationGain;
        }
        //std::cout << "kommer hit? 2.5" << std::endl;
        //std::cout << "Test 3" << std::endl;
        //std::cout << "Test 4" << std::endl;
        //std::cout << "naej" << std::endl;
        //std::cout << "My hits = " << it_goal->findInformationGain(SCALER_AABB, myMap) << std::endl;
        /*if((*it_goal) == goalNode){
          std::cout << (*it_goal)->sumDistance() << " is my sum distance and the following is my total distance  " << 0.2 * totalDistance << std::endl;
        };*/
        int stickyCounter = 0;
        for(std::list<ufo::math::Vector3>::iterator it_floor = (*it_goal)->myHits.begin(); it_floor != (*it_goal)->myHits.end(); it_floor++){
          if(it_floor->z() < (*it_goal)->point->z()){
            stickyCounter++;
          }
        }
        bool infoRequirement = ((*it_goal)->myHits.size() > 0.2 * averageInfo or averageInfoCounter < 6);
        bool stickyFloor = ((stickyCounter < 0.8 * (*it_goal)->myHits.size()) or averageInfoCounter < 6);
        if((newCost < totalCost) and ((*it_goal)->findInformationGain(SCALER_AABB, myMap, false) > 0) and allowNewPath and (stickyFloor and infoRequirement) and (*it_goal)->myParent != nullptr){
          // std::cout << newCost << " < " << totalCost << ", " << (*it_goal)->sumDistance() << " < " << 0.2 * totalDistance << std::endl;
          totalCost = newCost;
          goalNode = *it_goal;
          newPath = true;
          // std::cout << "\n" << std::endl;
          PATH_CONTAINER.clear();
          PATH_CONTAINER.push_back(position_x);
          PATH_CONTAINER.push_back(position_y);
          PATH_CONTAINER.push_back(position_z);
          for(std::list<double>::iterator path_itterator_helper = p_hist.begin(); path_itterator_helper != p_hist.end();){
            double x = *path_itterator_helper;
            PATH_CONTAINER.push_back(x);
            path_itterator_helper++;
            double y = *path_itterator_helper;
            PATH_CONTAINER.push_back(y);
            path_itterator_helper++;
            double z = *path_itterator_helper;
            PATH_CONTAINER.push_back(z);
            // std::cout << "This is point: " << x << ", " << y << ", " << z << std::endl;
            path_itterator_helper++;
          }
          // std::cout << p_hist << std::endl;
          // initialGoalInfo = goalNode->myHits.size();
        }
      }
    }
    if(not recoveryUnderway){
      std::list<node*>::iterator it_clear_helper;
      // std::cout << "Start deleting" << std::endl;
      for(it_clear_helper = CHOSEN_PATH.begin(); it_clear_helper != --CHOSEN_PATH.end(); it_clear_helper++){
        //std::cout << "Deleting 1" << std::endl;
        (*it_clear_helper)->readyForDeletion();
        delete(*it_clear_helper);
        //std::cout << "Deleting 2" << std::endl;
      }
      // std::cout << "Deleting done" << std::endl;
    }else{
      // std::cout << "Start deleting 2" << std::endl;
      std::list<node*>::iterator it_clear_helper = --CHOSEN_PATH.end();
      (*it_clear_helper)->readyForDeletion();
      delete(*it_clear_helper);
      recoveryUnderway = false;
      // std::cout << "Deleting done 2" << std::endl;
    }
    CHOSEN_PATH.clear();
    if(goalNode != nullptr and newPath){
      averageInfoCounter++;
      averageInfo = 100; //averageInfo + (goalNode->myHits.size() - averageInfo)/averageInfoCounter;
      setDistance = true;
      //linSpace(goalNode, DISTANCE_BETWEEN_NODES);
      //std::cout << "Test 5" << std::endl;
      // goalNode->getPath(&CHOSEN_PATH);
      std::list<double>::iterator it_path_helper = PATH_CONTAINER.begin();
      for(int i = 0; i < 50; i++){
        double x = *it_path_helper;
        it_path_helper++;
        double y = *it_path_helper;
        it_path_helper++;
        double z = *it_path_helper;
        it_path_helper++;
        CHOSEN_PATH.push_back(new node(x, y, z));
      }
      PATH_CONTAINER.clear();
      // std::cout << "Test 6" << std::endl;
      CHOSEN_PATH.push_back(new node(goalNode->point->x(), goalNode->point->y(), goalNode->point->z()));
      // std::cout << "Test 7" << std::endl;
      path_itterator = CHOSEN_PATH.begin();
      // std::cout << "Test 8" << std::endl;
      currentTarget = *path_itterator;
      // std::cout << "Test 9" << std::endl;
      // std::cout << currentTarget->point->x() << std::endl;
      // std::cout << currentTarget->point->y() << std::endl;
      // std::cout << currentTarget->point->z() << std::endl;
     }
  }/*else{
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
  }*/
  if(setDistance){
    totalDistance = goalNode->sumDistance();
  };
}

void generateRRT(float given_x, float given_y, float given_z){
  //std::cout << "Start deleting" << std::endl;
  for(std::list<node*>::iterator it_clear_helper = RRT_TREE.begin(); it_clear_helper != --RRT_TREE.end(); it_clear_helper++){
    //std::cout << "Deleting 1" << std::endl;
    (*it_clear_helper)->readyForDeletion();
    delete(*it_clear_helper);
    //std::cout << "Deleting 2" << std::endl;
  }
  for(std::list<node*>::iterator it_clear_helper = myGoals.begin(); it_clear_helper != --myGoals.end(); it_clear_helper++){
    (*it_clear_helper)->addParent(nullptr);
  }
  RRT_TREE.clear();
  std::cout << "Building RRT-tree" << std::endl;
  float step_length = 1;
  float sensor_range = 2;
  node* origin = new node(given_x, given_y, given_z);
  // std::cout << "My guessed point: " << given_x << ", " << given_y << ", " << given_z << std::endl;
  origin->addParent(nullptr);
  RRT_TREE.push_back(origin);
  srand(time(0));
  itterations = 0;
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
      std::list<node*>::iterator it_node;
      for(it_node = RRT_TREE.begin(); it_node != RRT_TREE.end(); it_node++){
        ufo::math::Vector3 direction = random_point - *((*it_node)->point);
        double new_distance = abs(direction.norm());
        if(new_distance < distance){
          distance = new_distance;
          parent = *it_node;
        }
      };
      ufo::math::Vector3 start_point(parent->point->x(), parent->point->y(), parent->point->z());
      ufo::geometry::LineSegment myLine(random_point, start_point);
      if(!isInCollision(myMap, myLine, true, false, true, 0)) {
        /*ufo::math::Vector3 newVector(parent->point->x() - x, parent->point->y() - y, parent->point->z() - z);
        float distance = newVector.norm();
        float itterations = (distance / radius);
        float part = radius / distance;
        float xStep = (parent->point->x() - x) * part;
        float yStep = (parent->point->y() - y) * part;
        float zStep = (parent->point->z() - z) * part;
        bool add = true;
        for(int i = 1; i < itterations; i++){
          ufo::math::Vector3 test_point(x + i * xStep, y + i * yStep, z + i * zStep);
          ufo::geometry::Sphere test_sphere(test_point, radius);
          if(isInCollision(myMap, test_sphere, true, false, true, 0)){
            add = false;
            break;
          }
        }
        if(add){*/
          node* new_node = new node(x, y, z);
          new_node->addParent(parent);
          parent->addChild(new_node);
          RRT_TREE.push_back(new_node);
        //}
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
    std::list<node*>::iterator it_comeon;
    for(it_comeon = RRT_TREE.begin(); it_comeon != RRT_TREE.end(); it_comeon++){
      total_childs = total_childs + (*it_comeon)->myChilds.size();
      if((*it_comeon)->myParent != nullptr){
        total_parents = total_parents + 1;
      }
    };
    if(total_childs != 0){
      RRT_created = true;
    }
    if(total_childs == NUMBER_OF_NODES){
      std::cout << "All children accounted for" << std::endl;
    }else{
      std::cout << "Expected " << NUMBER_OF_NODES << " children, but " << total_childs << " was found." << std::endl;
    };
    if(total_parents == NUMBER_OF_NODES){
      std::cout << "All parents accounted for" << std::endl;
    }else{
      std::cout << "Expected " << NUMBER_OF_NODES << " parents, but " << total_parents << " was found." << std::endl;
    };
  }else{
    std::cout << "Running by itterations, so the amount of nodes are unknown and hence can't be verified" << std::endl;
  }
};

std::tuple<std::list<double>, double, std::list<double>> trajectory(std::list<double> x, double* u, double N, double dt, std::list<double> nmpc_ref){
    // Based on the initial condition and optimized trajectory u, computed the path as (x,y,z).
    // Calculate the dynamic costs based on selected weights  
    double ns = 8;
    std::list<double> p_traj{};
    std::list<double> v_traj{};
    double cost = 0;
    // Weight matrices
    std::list<double> Qx = {5,5,5, 0, 0,0, 5, 5};
    // P = 2*Qx; #final state weight
    std::list<double> Ru = {15,15,15}; // input weights
    std::list<double> Rd = {10, 10, 10}; // input rate weights
    // print(x, u, N, dt)
    std::list<double> u_old = {9.81,0.0,0.0};
    std::list<double> u_ref = {9.81,0.0,0.0};
    for(int i = 0; i < N; i++){
      // State costs
      // std::list<float> x_ref = nmpc_ref[(ns*i):(ns*i+ns)];
      // #print(x_ref)
      // Setting up itterators
      std::list<double>::iterator Qx_itterator = Qx.begin();
      //std::advance(Qx_itterator, i);
      std::list<double>::iterator x_itterator = x.begin();
      //std::advance(x_itterator, i);
      std::list<double>::iterator x_ref_itterator = nmpc_ref.begin();
      // std::advance(x_ref_itterator, i*ns);
      
      for(int j = 0; j < 8; j++){
        // std::cout << (*Qx_itterator) << ", " <<  (*x_itterator) << ", " << (*x_ref_itterator) << std::endl;
        if(i < 3){
          cost = cost + (*Qx_itterator) * pow((*x_itterator) - (*x_ref_itterator), 2);
        }else{
          cost = cost + (*Qx_itterator) * pow((*x_itterator), 2);
        }
        Qx_itterator++;
        x_itterator++;
        x_ref_itterator++;
      }
      //std::cout << "\n" << std::endl;
      //std::cout << "this is cost: " << cost << std::endl;
      //cost = cost + pow((*Qx_itterator) * (*x_itterator - *x_ref_itterator), 2) + Qx[1]*(x[1]-x_ref[i + 1])**2 + Qx[2]*(x[2]-x_ref[i + 2])**2 + Qx[3]*(x[3]-x_ref[i + 3])**2 + Qx[4]*(x[4]-x_ref[i + 4])**2 + Qx[5]*(x[5]-x_ref[i + 5])**2 + Qx[6]*(x[6]-x_ref[i + 6])**2 + Qx[7]*(x[7]-x_ref[i + 7])**2;  // State weights
      // Input Cost
      
      //Setting up itterators
      std::list<double>::iterator Ru_itterator = Ru.begin();
      std::list<double>::iterator Rd_itterator = Rd.begin();
      //std::list<float>::iterator u_n_itterator = u.begin();
      //std::advance(u_n_itterator, 3 * i);
      std::list<double>::iterator u_ref_itterator = u_ref.begin();
      std::list<double>::iterator u_old_itterator = u_old.begin();
      
      for(int j = 0; j < 3; j++){
        // std::cout << (*Rd_itterator) << ", " <<  u[3*i+j] << ", " << *u_old_itterator << std::endl;
        cost = cost + *Ru_itterator * pow(u[3*i+j] - (*u_ref_itterator), 2);
        cost = cost + *Rd_itterator * pow(u[3*i+j] - *u_old_itterator, 2);
        Ru_itterator++;
        u_old_itterator++;
        Rd_itterator++;
        u_ref_itterator++;
      }
      //std::cout << "this is cost" << cost << std::endl;
      u_old = {u[3*i], u[3*i+1], u[3*i+2]};
      //u_n = u[(3*i):3*i+3];
      //cost += Ru[0]*(u_n[0] - u_ref[0])**2 + Ru[1]*(u_n[1] - u_ref[1])**2 + Ru[2]*(u_n[2] - u_ref[2])**2; // Input weights
      //cost += Rd[0]*(u_n[0] - u_old[0])**2 + Rd[1]*(u_n[1] - u_old[1])**2 + Rd[2]*(u_n[2] - u_old[2])**2; // Input rate weights
      //u_old = u_n;
      // x_hist = x_hist + [x];
      
      x_itterator = x.begin();
      std::list<double>::iterator x2_itterator = x.begin();
      std::advance(x2_itterator, 3);
      //std::advance(u_n_itterator, -2);
      for(int j = 0; j < 3; j++){
        *x_itterator = *x_itterator + dt * *x2_itterator;
        x_itterator++;
        x2_itterator++;
      }
      std::list<double>::iterator x3_itterator = x.begin();
      std::advance(x3_itterator, 7); // x[7]
      *x_itterator = *x_itterator + dt * (sin(*x3_itterator) * cos(*x2_itterator) * u[3*i] - 0.5 * (*x_itterator));
      // std::cout << "THIS IS IMPORTANT: " << *x_itterator << std::endl;
      x_itterator++; // x[4]
      *x_itterator = *x_itterator + dt * (-sin(*x2_itterator) * u[3*i] - 0.5 * (*x_itterator));
      x_itterator++; // x[5]
      // std::cout << "THIS IS IMPORTANT: "<< *x_itterator << ", " << *x3_itterator << ", " << *x2_itterator << ", " << u[3*i] << std::endl;
      *x_itterator = *x_itterator + dt * (cos(*x3_itterator) * cos(*x2_itterator) * u[3*i] - 0.5 * *x_itterator - 9.81);
      x_itterator++;
      //u_n_itterator++;
      *x_itterator = *x_itterator + dt * ((1.0 / 0.8) * (u[3*i + 1] - *x_itterator));
      x_itterator++;
      //u_n_itterator++;
      *x_itterator = *x_itterator + dt * ((1.0 / 0.8) * (u[3*i + 2] - *x_itterator));
      /*
      p_hist = p_hist + [[x[0],x[1],x[2]]];*/
      x_itterator = x.begin();
      p_traj.push_back(*x_itterator);
      x_itterator++;
      p_traj.push_back(*x_itterator);
      x_itterator++;      
      p_traj.push_back(*x_itterator);
      x_itterator++;
      v_traj.push_back(*x_itterator);
      x_itterator++;
      v_traj.push_back(*x_itterator);
      x_itterator++;      
      v_traj.push_back(*x_itterator);
    }
    // std::cout << "\n" << std::endl;
    //int schme = 0;
    // std::cout << "This is p_hist:" << std::endl;
    /*for(std::list<double>::iterator x5_itterator = p_hist.begin(); x5_itterator != p_hist.end(); x5_itterator++){
      if(schme%3 == 0 and schme != 0){
        std::cout << "\n" << std::endl;
      }
      std::cout << *x5_itterator << std::endl;
      schme++;
    }*
    /*schme = 0;
    std::cout << "This is x_hist:\n" << std::endl;
    for(std::list<double>::iterator x4_itterator = x_hist.begin(); x4_itterator != x_hist.end(); x4_itterator++){
      if(schme%8 == 0 and schme != 0){
        std::cout << "\n" << std::endl;
      }
      std::cout << *x4_itterator << std::endl;
      schme++;
    }*/
    // std::cout << cost << std::endl;
    // std::cout << x_hist << std::endl;
    // std::cout << "skibidibap: " << cost << std::endl;
    return std::make_tuple(v_traj, cost, p_traj);
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
  velocity_x = msg->twist.twist.linear.x;
  velocity_y = msg->twist.twist.linear.y;
  velocity_z = msg->twist.twist.linear.z;
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

typedef rrtCache* (*arbitrary)();
typedef rrtSolverStatus (*arbitrary2)(void*, double*, double*, double, double*);
typedef void (*rrt_clearer)(rrtCache*);

int main(int argc, char *argv[])
{ 
  ros::init(argc, argv, "RRT_TREE");
  ros::NodeHandle nh;
  ros::Publisher points_pub = nh.advertise<visualization_msgs::Marker>("RRT_NODES", 1);
  ros::Publisher chosen_path_visualization_pub = nh.advertise<visualization_msgs::Marker>("CHOSEN_RRT_PATH_VISUALIZATION", 1);
  // ros::Publisher chosen_path_pub = nh.advertise<nav_msgs::Path>("CHOSEN_RRT_PATH", 1);
  //ros::Publisher chosen_path_pub = nh.advertise<geometry_msgs::PoseStamped>("/shafter3d/reference", 1);
  ros::Publisher chosen_path_pub = nh.advertise<geometry_msgs::PoseStamped>("/pelican/reference", 1);
  ros::Publisher all_path_pub = nh.advertise<visualization_msgs::Marker>("RRT_PATHS", 1);
  ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>("RRT_GOALS", 1);
  ros::Publisher map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("goe_map", 11);
  ros::Subscriber map_sub = nh.subscribe("ufomap_mapping_server_node/map_depth_3", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("/pelican/ground_truth/odometry", 1, odomCallback);
  //ros::Subscriber sub = nh.subscribe("/odometry/imu", 1, odomCallback);
  ros::Publisher hits_pub = nh.advertise<visualization_msgs::Marker>("HITS", 1);
  ros::Publisher position_pub = nh.advertise<visualization_msgs::Marker>("POSITION", 1);
  ros::Publisher taken_path_pub = nh.advertise<visualization_msgs::Marker>("PATH_TAKEN", 1);
  ros::Rate rate(10);
  
  // C++ bindings battlefield
  /* parameters             */
  int i;
  double p[159] = {0};
  // Current position
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  p[3] = 0;
  p[4] = 0;
  p[5] = 0;
  p[6] = 0;
  p[7] = 0;
  
  // Trajectory
  for (i = 1; i < 50; ++i) {
    p[8+3*i] = 0.5 * i; 
    p[8+3*i+1] = 0.5 * i;
    p[8+3*i+2] = 0;
  }
  
  p[158] = 0.5;
  //std::cout << "This is good: " << p[8] << std::endl;
  /*for (i = 0; i < 159; ++i) {
    std::cout << i << std::endl;
    printf("p[%d] = %g\n", i, p[i]);
  }*/
  
  /* initial guess          */
  double u[150] = {0};

  for (i = 0; i < 50; ++i) {
    u[8+3*i] = 9.81;
    u[8+3*i+1] = 0;
    u[8+3*i+2] = 0;
  }

  /* initial penalty        */
  double init_penalty = 1.0;
  void *handle = dlopen("./MAV/rrt/target/release/librrt.so", RTLD_LAZY);
  if (!handle) {
    fprintf(stderr, "%s\n", dlerror());
    exit(EXIT_FAILURE);
  }
  arbitrary rrt_new;
  *(void **) (&rrt_new) = dlsym(handle, "rrt_new");
  // std::cout << rrt_new << std::endl;
  rrtCache* cache = rrt_new();
  // std::cout << cache << std::endl;
  arbitrary2 rrt_solve;
  *(void **) (&rrt_solve) = dlsym(handle, "rrt_solve");
  rrt_clearer rrt_free;
  *(void **) (&rrt_free) = dlsym(handle, "rrt_free");
  // std::cout << rrt_free << std::endl;
  std::cout << init_penalty << std::endl;
  rrtSolverStatus status = rrt_solve(cache, u, p, 0, &init_penalty);
  // std::cout << "This is good 2: " << p[8] << std::endl;
  // std::cout << "This is good 2: " << u[0] << std::endl;
  printf("\n\n-------------------------------------------------\n");
  printf("  Solution\n");
  printf("-------------------------------------------------\n");

  /*for (i = 0; i < RRT_NUM_DECISION_VARIABLES; ++i) {
    printf("u[%d] = %g\n", i, u[i]);
  }*/

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
  
  
  rrt_free(cache);
  
  // Trajectory battleground:
  double N = 50;
  double dt = 0.5;
  std::list<double> x0 = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  std::list<double> xref_ref = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  std::list<double> xref = {};
  std::list<double>::iterator xref_ref_itterator_helper = xref_ref.begin();
  for(int i = 0; i < N; i++){
    xref.push_back(*xref_ref_itterator_helper);
    xref_ref_itterator_helper++;
    if(xref_ref_itterator_helper == xref_ref.end()){
      xref_ref_itterator_helper = xref_ref.begin();
    }
  }
  std::list<double> x_hist;
  std::list<double> p_hist;
  double cost;
  // std::cout << "trajectory time" << std::endl;
  std::tie(p_hist, cost, x_hist) = trajectory(x0, u, N, dt, xref);
  // std::cout << "trajectory time finished" << std::endl;
  /*std::cout << "p_hist " << cost << std::endl;
  
  for(std::list<float>::iterator path_itterator_helper = p_hist.begin(); path_itterator_helper != p_hist.end(); path_itterator_helper++){
    std::cout << *path_itterator_helper << std::endl;
  }
  
  std::cout << cost << std::endl;
  std::cout << "x_hist" << std::endl;
  for(std::list<float>::iterator path_itterator_helper = x_hist.begin(); path_itterator_helper != x_hist.end(); path_itterator_helper++){
    std::cout << *path_itterator_helper << std::endl;
  }*/

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
    high_resolution_clock::time_point start_total = high_resolution_clock::now();
    //high_resolution_clock::time_point stop_total;
    std::cout << "start 1" << std::endl;
    if(map_received and not GOALS_generated and position_received){
      // std::cout << "start 2" << std::endl;
      tuneGeneration(myMap, false, true, false, position_x, position_y, position_z, 3);
      // std::cout << "start 3" << std::endl;
      generateGoals(myMap, true);
    }
    // std::cout << "start 4" << std::endl;
    if(map_received and not RRT_created and GOALS_generated){
      high_resolution_clock::time_point start = high_resolution_clock::now();
      generateRRT(position_x, position_y, position_z);
      high_resolution_clock::time_point stop = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>(stop - start);
      if(itterations != 100000){
        cout << "\nExecution time: " << duration.count() << " micro seconds for " << itterations << " itterations." << endl;
      }else{
        cout << "\nTimeout after " << duration.count() << " micro seconds for " << itterations << " itterations." << endl;
      }
      if(RRT_created){
        // std::cout << "går in mot findshortestpath" << std::endl;
        // high_resolution_clock::time_point start2 = high_resolution_clock::now();
        // std::cout << "är problemet findshortestpath?" << std::endl;
        findShortestPath();
        // std::cout << "problemet är inte findshortestpath" << std::endl;
        // high_resolution_clock::time_point stop2 = high_resolution_clock::now();
        // auto duration2 = duration_cast<microseconds>(stop2 - start2);
        // cout << "\nExecution time: " << duration2.count() << ", this is to find the shortest paths to our goals" << endl;
      }
      itterations = 0; //DO NOT TOUCH!
    }
    if(map_received and RRT_created){
      // std::cout << "Går förbi map_received" << std::endl;
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
      std::list<node*>::iterator it_comeon_visualizer;
      // std::cout << "Försöker visualisera rrt_tree" << std::endl;	
      for(it_comeon_visualizer = RRT_TREE.begin(); it_comeon_visualizer != RRT_TREE.end(); it_comeon_visualizer++){
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer)->point->x();
        p.y = (*it_comeon_visualizer)->point->y();
        p.z = (*it_comeon_visualizer)->point->z();
        RRT_points.points.push_back(p);
        if((*it_comeon_visualizer)->myParent != nullptr){
          RRT_line_list.points.push_back(p);
          p.x = (*it_comeon_visualizer)->myParent->point->x();
          p.y = (*it_comeon_visualizer)->myParent->point->y();
          p.z = (*it_comeon_visualizer)->myParent->point->z();
          RRT_line_list.points.push_back(p);
        }
      }
      // std::cout << "lyckades visualisera rrt_tree" << std::endl;
      points_pub.publish(RRT_points);
      points_pub.publish(RRT_line_list);
      
      if(!fetched_path and RRT_created){
        // std::cout << "kommer hit? 1" << std::endl;
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
        CHOSEN_PATH_points.color.r = 1.0f;
        CHOSEN_PATH_points.color.a = 1.0;
        CHOSEN_PATH_line_list.color.b = 1.0;
        CHOSEN_PATH_line_list.color.a = 1.0;
        // std::cout << "kommer hit? 2" << std::endl;
        high_resolution_clock::time_point start = high_resolution_clock::now();
        // std::cout << "clock crash?" << std::endl;
        std::cout << "Är problemet setPath?" << std::endl;
        setPath();
        std::cout << "Problemet är inte setPath!" << std::endl;
        // stop_total = high_resolution_clock::now();
        high_resolution_clock::time_point stop = high_resolution_clock::now();
        // std::cout << "kommer hit? 3" << std::endl;
        /*
        CHOSEN_PATH.clear();
        std::cout << "kommer hit? 4" << std::endl;
        goalNode->getPath(&CHOSEN_PATH);
        CHOSEN_PATH.push_back(goalNode);
        path_itterator = CHOSEN_PATH.begin();
        std::advance(path_itterator, advance_index);
        */
        // std::cout << "kommer hit? 5" << std::endl;
        // high_resolution_clock::time_point stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        cout << "\nExecution time: " << duration.count() << " micro seconds for " << myGoals.size() << " path/s. asdf" << endl;
        std::cout << "kommer hit? 6" << std::endl;
        if(newPath and allowNewPath){
          // std::cout << "kommer hit? 6.1" << std::endl;
          newPath = false;
          allowNewPath = false;
          path_itterator = CHOSEN_PATH.begin();
          // std::cout << "kommer hit? 6.2" << std::endl;
          currentTarget = *path_itterator;
          advance_index = 0;
          // std::cout << "kommer hit? 6.2.0" << std::endl;
          // std::cout << currentTarget << std::endl;
          // std::cout << "kommer hit? 6.3" << std::endl;
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
      
        std::cout << "kommer hit? 8" << std::endl;
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
        std::list<node*>::iterator it_comeon_visualizer5;
        ALL_PATH.clear();
        for(it_comeon_visualizer5 = myGoals.begin(); it_comeon_visualizer5 != myGoals.end(); it_comeon_visualizer5++){
          if(*it_comeon_visualizer5 == goalNode){
            (*it_comeon_visualizer5)->getPath(&ALL_PATH);
            ALL_PATH.push_back((*it_comeon_visualizer5));
          }
        }
        // std::cout << ALL_PATH.size() << std::endl;
        std::list<node*>::iterator it_comeon_visualizer6;	
        for(it_comeon_visualizer6 = ALL_PATH.begin(); it_comeon_visualizer6 != ALL_PATH.end(); it_comeon_visualizer6++){
          geometry_msgs::Point p;
          p.x = (*it_comeon_visualizer6)->point->x();
          p.y = (*it_comeon_visualizer6)->point->y();
          p.z = (*it_comeon_visualizer6)->point->z();
          PATH_points.points.push_back(p);
          /*if((*it_comeon_visualizer6)->myParent != nullptr){
            PATH_line_list.points.push_back(p);
            p.x = (*it_comeon_visualizer6)->myParent->point->x();
            p.y = (*it_comeon_visualizer6)->myParent->point->y();
            p.z = (*it_comeon_visualizer6)->myParent->point->z();
            PATH_line_list.points.push_back(p);
          }*/
        }
        // std::cout << "har vi en path?" << std::endl;
        all_path_pub.publish(PATH_points);
        all_path_pub.publish(PATH_line_list);
        // std::cout << "kommer hit? slut" << std::endl;
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
      std::list<node*>::iterator it_comeon_visualizer3;	
      for(it_comeon_visualizer3 = myGoals.begin(); it_comeon_visualizer3 != myGoals.end(); it_comeon_visualizer3++){
        geometry_msgs::Point p;
        //if(*it_comeon_visualizer3 == goalNode){
          p.x = (*it_comeon_visualizer3)->point->x();
          p.y = (*it_comeon_visualizer3)->point->y();
          p.z = (*it_comeon_visualizer3)->point->z();
          GOAL_points.points.push_back(p);
        //}
      }
      goal_pub.publish(GOAL_points);
      std::cout << "kommer hit? slut.3" << std::endl;
      
      if(goalNode != nullptr){
        hits.clear();
        // std::cout << "kommer hit? slut.4" << std::endl;
        goalNode->addHits(&hits);
        // std::cout << "kommer hit? slut.5" << std::endl;
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
      }
      
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
        node* myNode = new node(position_x, position_y, position_z);
        VISITED_POINTS.push_back(myNode);
      }else{
        std::list<node*>::iterator taken_path_visualizer;
        taken_path_visualizer = --VISITED_POINTS.end();
        if(sqrt(pow((*taken_path_visualizer)->point->x() - position_x, 2) + pow((*taken_path_visualizer)->point->y() - position_y, 2) + pow((*taken_path_visualizer)->point->z() - position_z, 2)) >= 1.0){
          node* myNode = new node(position_x, position_y, position_z);
          (*taken_path_visualizer)->addParent(myNode);
          VISITED_POINTS.push_back(myNode);
        }
      }
      std::list<node*>::iterator taken_path_visualizer;	
      for(taken_path_visualizer = VISITED_POINTS.begin(); taken_path_visualizer != VISITED_POINTS.end(); taken_path_visualizer++){
        geometry_msgs::Point p;
        //if(*it_comeon_visualizer3 != goalNode){
          p.x = (*taken_path_visualizer)->point->x();
          p.y = (*taken_path_visualizer)->point->y();
          p.z = (*taken_path_visualizer)->point->z();
          TAKEN_PATH_points.points.push_back(p);
        //}
        taken_path_pub.publish(TAKEN_PATH_points);
        //taken_path_pub.publish(TAKEN_PATH_line_list);
      
        // std::cout << "kommer hit? slut.6" << std::endl;
        // std::cout << "kommer hit? slut.7" << std::endl;
      }
      std::cout << "kommer hit? 714" << std::endl;
      if(fetched_path and goalNode != nullptr){  
        itterations++;
        std::cout << "kommer hit? 715" << std::endl;
        if(sqrt(pow(position_x - goalNode->point->x(), 2) + pow(position_y - goalNode->point->y(), 2) + pow(position_z - goalNode->point->z(), 2)) < 0.5){
          std::cout << "kommer hit? 716" << std::endl;
          itterations = 0;
          fetched_path = false;
          RRT_created = false;
          GOALS_generated = false;
          position_received = false;
          //allowNewPath = true;
          /*if(recoveryUnderway){
            for(std::list<node*>::iterator erase_reserveGoal_it = myReserveGoals.end(); erase_reserveGoal_it != myReserveGoals.begin(); erase_reserveGoal_it--){
              if(*erase_reserveGoal_it == goalNode){
                myReserveGoals.erase(erase_reserveGoal_it);
                goalNode = nullptr;
                std::cout << "Old reserve goal deleted, new size is now: " << myReserveGoals.size() << std::endl;
                allowNewPath = true;
                break;
              }
            }
          }*/
        }
        std::cout << "Kommer hit? slut.0" << std::endl;
        if((sqrt(pow(position_x - currentTarget->point->x(), 2) + pow(position_y - currentTarget->point->y(), 2) + pow(position_z - currentTarget->point->z(), 2)) < 0.5) and path_itterator != --CHOSEN_PATH.end()){
          // std::cout << "Kommer hit? slut.0.1" << std::endl;
          //path_itterator++;
          advance_index++;
          path_itterator = CHOSEN_PATH.begin();
          std::advance(path_itterator, advance_index);
          currentTarget = *path_itterator;
          // std::cout << "Kommer hit? slut.0.2" << std::endl;
          if(path_itterator == CHOSEN_PATH.end()){
            path_itterator--;
            currentTarget = *path_itterator;
          }
          // std::cout << "kommer hit? slut.0.2.0" << std::endl;
          // std::cout << currentTarget << std::endl;
          // std::cout << "Kommer hit? slut.0.3" << std::endl;
        }
        // std::cout << "Kommer hit? slut.0.slut" << std::endl;
        if(path_itterator != CHOSEN_PATH.end()){
          // std::cout << "kommer hit? slut.1" << std::endl;
          geometry_msgs::PoseStamped nextPoint;
          // std::cout << "kommer hit? slut.1.1" << std::endl;
          nextPoint.pose.position.x = (currentTarget)->point->x();
          // std::cout << "kommer hit? slut.1.2" << std::endl;
          nextPoint.pose.position.y = (currentTarget)->point->y();
          // std::cout << "kommer hit? slut.3" << std::endl;
          nextPoint.pose.position.z = (currentTarget)->point->z();
          // std::cout << "kommer hit? slut.4" << std::endl;
          nextPoint.pose.orientation.x = 0;
          nextPoint.pose.orientation.y = 0;
          nextPoint.pose.orientation.z = 0;
          nextPoint.pose.orientation.w = 0;
          nextPoint.header.stamp = ros::Time::now();
          nextPoint.header.frame_id = "world";
          chosen_path_pub.publish(nextPoint);
        }
        // std::cout << "kommer hit? slut.1.3" << std::endl;
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
      // std::cout << "kommer hit? slut.slut" << std::endl;
      // ros::spinOnce();
      // std::cout << "kommer hit? slut.slut.slut\n" << std::endl;
      /*if(goalNode != nullptr){
        for(std::list<node*>::iterator path_itterator_helper = CHOSEN_PATH.begin(); path_itterator_helper != CHOSEN_PATH.end();){
          std::cout << "target: " << (*path_itterator_helper)->point->x() << ", " << (*path_itterator_helper)->point->y() << ", " << (*path_itterator_helper)->point->z() << std::endl;
          path_itterator_helper++;
        }
        std::cout << "This is total distance: " << totalDistance << std::endl;
        if(goalNode != nullptr){
          std::cout << "This is sum distance: " << goalNode->sumDistance() << std::endl;
        }
      }*/
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
      /*if(currentTarget != nullptr){
        std::cout << "This is my current target: " << currentTarget->point->x() << ", " << currentTarget->point->y() << ", " << currentTarget->point->z() << std::endl;
        // std::cout << sqrt(pow(position_x - currentTarget->point->x(), 2) + pow(position_y - currentTarget->point->y(), 2) + pow(position_z - currentTarget->point->z(), 2)) << " < " << 0.5 << std::endl;
      }*/
      // std::cout << advance_index << std::endl;
      // std::cout << fetched_path << std::endl;
      if((goalNode != nullptr or (goalNode == nullptr and GOALS_generated)) or averageInfoCounter > 2){
        // std::cout << "This is my found infoGain for the chosen path: " << goalNode->myHits.size() << std::endl;
        std::cout << "This is my average infoGain: " << averageInfo << std::endl;
        std::cout << "This is my infoGain counter: " << averageInfoCounter << std::endl;
        std::cout << "This is my goalNodes infogain: " << initialGoalInfo << std::endl;
        if(initialGoalInfo < (0.2 * averageInfo) and averageInfoCounter > 5 and not recoveryUnderway){
          for(int i = 0; i < 3; i++){
            generateGoals(myMap, false);
            generateRRT(position_x, position_y, position_z);
            allowNewPath = true;
            setPath();
            if(initialGoalInfo > (0.2 * averageInfo)){
              break;
            }
          }
        }
        if((initialGoalInfo < (0.2 * averageInfo) and averageInfoCounter > 5 and not recoveryUnderway)){
          /*
          1. Evaluate last added reserveGoal, remove if informationGain is too low ( informationGain < 20)
          2. Itterate through the visited points until one passes the collision check to the goal (line check / sphear check?)
          3. Add to CHOSEN_PATH and set variables accordingly.
          */
          std::cout << "TRIGGER!" << std::endl;
          for(std::list<node*>::iterator retrace_path_itterator = --myReserveGoals.end(); retrace_path_itterator != myReserveGoals.begin(); retrace_path_itterator--){
            std::cout << "global planner 1" << std::endl;
            if(!CHOSEN_PATH.empty()){
              for(std::list<node*>::iterator it_clear_helper = CHOSEN_PATH.begin(); it_clear_helper != --CHOSEN_PATH.end(); it_clear_helper++){
                //std::cout << "Deleting 1" << std::endl;
                std::cout << "global planner 2" << std::endl;
                (*it_clear_helper)->readyForDeletion();
                delete(*it_clear_helper);
                //std::cout << "Deleting 2" << std::endl;
              }
              //std::cout << "Deleting done" << std::endl;
              std::cout << "global planner 3" << std::endl;
              CHOSEN_PATH.clear();
            }
            std::cout << "global planner 4" << std::endl;
            (*retrace_path_itterator)->clearInformationGain();
            std::cout << "global planner 5" << std::endl;
            double informationGain = (*retrace_path_itterator)->findInformationGain(SCALER_AABB, myMap, true);
            std::cout << "global planner 6" << std::endl;
            if(informationGain > 0.2 * averageInfo){
              (*retrace_path_itterator)->findPathImprovement(*retrace_path_itterator, myMap, DISTANCE_BETWEEN_NODES, radius);
              std::cout << "global planner 7" << std::endl;
              // linSpace(*retrace_path_itterator, DISTANCE_BETWEEN_NODES); // Core dump here?
              // std::cout << "global planner 6" << std::endl;
              std::list<struct node*> PATH_CONTAINER{};
              node* currentPoint = nullptr;
              node* savedPoint = nullptr;
              (*retrace_path_itterator)->getPath(&PATH_CONTAINER);
              PATH_CONTAINER.push_back(new node((*retrace_path_itterator)->point->x(), (*retrace_path_itterator)->point->y(), (*retrace_path_itterator)->point->z()));
              for(std::list<node*>::iterator it_path_helper1 = PATH_CONTAINER.begin(); it_path_helper1 != PATH_CONTAINER.end(); it_path_helper1++){ //This doesn't shorten the paths correctly, it can only attatch to a point it has direct vision of and if the path goes around a corner and comes back it will still take that route instead of cutting out the roundabout.
                if(it_path_helper1 == --PATH_CONTAINER.end()){
                  CHOSEN_PATH.push_back(*it_path_helper1);
                  break;
                }
                if(CHOSEN_PATH.empty()){ //If path is empty, add first point to path and save it as the currentPoint to go from
                  CHOSEN_PATH.push_back(*it_path_helper1);
                  currentPoint = *it_path_helper1;
                }else{ // Otherwise, see if the first point can reach the next point in the path, if that's the case, save the point that was reached and step it_path_helper
                  ufo::geometry::LineSegment myLine(*(currentPoint->point), (*(*it_path_helper1)->point));
                  if(!isInCollision(myMap, myLine, true, false, true, 3)){ // If we're not in collision, save the reached point
                    savedPoint = *it_path_helper1;
                  }else{ // If we can't reach the new point, save the previous point found
                    CHOSEN_PATH.push_back(savedPoint);
                    currentPoint = savedPoint;
                    ufo::geometry::LineSegment myLine(*(currentPoint->point), *((*it_path_helper1)->point));
                    if(!isInCollision(myMap, myLine, true, false, true, 3)){ // Evaluate the point we pushed to the path and make sure we can reach the point the itterator is pointing to
                      savedPoint = *it_path_helper1;
                    }else{
                      savedPoint = currentPoint;
                    }
                  }
                }
              }
              CHOSEN_PATH.push_back(new node((*retrace_path_itterator)->point->x(), (*retrace_path_itterator)->point->y(), (*retrace_path_itterator)->point->z()));
              std::cout << "global planner 8" << std::endl;
              goalNode = *retrace_path_itterator;
              path_itterator = CHOSEN_PATH.begin();
              
              currentTarget = *path_itterator;
              advance_index = 0;
              allowNewPath = false;
              recoveryUnderway = true;
              std::list<node*>::iterator erase_it = myReserveGoals.end();
              retrace_path_itterator++;
              myReserveGoals.erase(retrace_path_itterator, erase_it);
              std::cout << "global planner 9" << std::endl;
              break;
            }/*else{
              myReserveGoals.erase(retrace_path_itterator);
              std::cout << "This is the size of reserve goals: " << myReserveGoals.size() << std::endl;
            }*/
            
          }
          /*std::list<node*>::iterator retrace_path_itterator = --VISITED_POINTS.end();
          node* lastChecked = *retrace_path_itterator;
          for(retrace_path_itterator; retrace_path_itterator != VISITED_POINTS.begin(); retrace_path_itterator--){
            if(sqrt(pow(lastChecked->point->x() - (*retrace_path_itterator)->point->x(), 2) + pow(lastChecked->point->y() - (*retrace_path_itterator)->point->y(), 2) + pow(lastChecked->point->z() - (*retrace_path_itterator)->point->z(), 2)) >= SCALER_AABB){
              lastChecked = *retrace_path_itterator;
              tuneGeneration(myMap, false, true, false, (*retrace_path_itterator)->point->x(), (*retrace_path_itterator)->point->y(), (*retrace_path_itterator)->point->z(), 3);
              generateGoals(myMap);
              int largestInformationGain = 0;
              for(std::list<node*>::iterator retrace_path_goal_itterator = myGoals.begin(); retrace_path_goal_itterator != myGoals.end(); retrace_path_goal_itterator++){
                int newInformationGain = (*retrace_path_goal_itterator)->findInformationGain(SCALER_AABB, myMap);
                if(newInformationGain > largestInformationGain){
                  goalNode = *retrace_path_goal_itterator;
                  largestInformationGain = newInformationGain;
                }
              }
              int stickyCounter = 0;
              for(std::list<ufo::math::Vector3>::iterator it_floor = goalNode->myHits.begin(); it_floor != goalNode->myHits.end(); it_floor++){
                if(it_floor->z() < goalNode->point->z()){
                  stickyCounter++;
                }
              }
              bool stickyFloor = ((stickyCounter < 0.8 * goalNode->myHits.size()) or averageInfoCounter < 6);
              if(goalNode->myHits.size() > averageInfo * 0.5 and stickyFloor){
                //create path between where we're at and the point we found
                averageInfoCounter++;
                averageInfo = 100; // averageInfo + (goalNode->myHits.size() - averageInfo)/averageInfoCounter;
                ufo::math::Vector3 myPoint1((*retrace_path_itterator)->point->x(), (*retrace_path_itterator)->point->y(), (*retrace_path_itterator)->point->z());
                ufo::geometry::LineSegment myLine(*(goalNode->point), myPoint1);
                if(!isInCollision(myMap, myLine, true, false, true, 3)){
                  // add to path
                  std::list<node*>::iterator it_clear_helper;
                  //std::cout << "Start deleting" << std::endl;
                  for(it_clear_helper = CHOSEN_PATH.begin(); it_clear_helper != --CHOSEN_PATH.end(); it_clear_helper++){
                    //std::cout << "Deleting 1" << std::endl;
                    (*it_clear_helper)->readyForDeletion();
                    delete(*it_clear_helper);
                    //std::cout << "Deleting 2" << std::endl;
                  }
                  //std::cout << "Deleting done" << std::endl;
                  CHOSEN_PATH.clear();
                  std::list<node*>::iterator retrace_path_itterator_helper = VISITED_POINTS.end();
                  std::advance(retrace_path_itterator_helper, -2);
                  retrace_path_itterator--;
                  for(retrace_path_itterator_helper; retrace_path_itterator_helper != retrace_path_itterator; retrace_path_itterator_helper--){
                    // node* myNode = new node((*retrace_path_itterator_helper)->point->x, (*retrace_path_itterator_helper)->point->y, (*retrace_path_itterator_helper)->point->z);
                    CHOSEN_PATH.push_back(*retrace_path_itterator_helper);
                  }
                  CHOSEN_PATH.push_back(new node(goalNode->point->x(), goalNode->point->y(), goalNode->point->z()));
                  initialGoalInfo = goalNode->myHits.size();
                  path_itterator = CHOSEN_PATH.begin();
                  currentTarget = *path_itterator;
                  advance_index = 0;
                  allowNewPath = false;
                  recoveryUnderway = true;
                  break;
                }else{
                  std::cout << "fan" << std::endl;
                }
              }
            }
          }*/
          //break;
        }
      }
    }
    std::cout << SCALER_X << " = " << highest_x << " - " << lowest_x << std::endl;
    std::cout << SCALER_Y << " = " << highest_y << " - " << lowest_y << std::endl;
    std::cout << SCALER_Z << " = " << highest_z << " - " << lowest_z << std::endl;
    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<microseconds>(stop_total - start_total);
    cout << "\nExecution time: " << duration_total.count() << " micro seconds for " << myGoals.size() << " path/s." << endl;
    /*if(allowNewPath == false){
      break;
    }*/
    /*if(!myReserveGoals.empty()){
      std::list<node*>::iterator reserveGoalItterator = --myReserveGoals.end();
      std::cout << "This is the most recent reserve goal: " << (*reserveGoalItterator)->myParent->point->x() << ", " << (*reserveGoalItterator)->myParent->point->y() << ", " << (*reserveGoalItterator)->myParent->point->z() << std::endl;
    }*/
    if(!CHOSEN_PATH.empty() and recoveryUnderway){
      std::cout << "Let's try to print the path!" << std::endl;
      for(std::list<node*>::iterator path_itterator_helper = CHOSEN_PATH.begin(); path_itterator_helper != CHOSEN_PATH.end();){
        std::cout << "target: " << (*path_itterator_helper)->point->x() << ", " << (*path_itterator_helper)->point->y() << ", " << (*path_itterator_helper)->point->z() << std::endl;
        path_itterator_helper++;
      }
    }
    std::cout << "This is size of reserveGoals: "  << myReserveGoals.size() << std::endl;
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
// export ROS_MASTER_URI=http://10.42.0.1:11311; export ROS_HOSTNAME=10.0.2.15; export ROS_IP=10.0.2.15

/*
To run the sensors on the drone:

1. sudo ptpd -M -i eno1 -C
2. roslaunch initialization all_components.launch (gnc_ws)
3. roslaunch ouster_ros ouster.launch (sensors_ws)
4. roslaunch lio_sam run_ousters.launch (gnc_ws)
 

*/
