// Standard header files
#include <iostream>
#include <memory>

// CLiFFMapROS headers
#include <cliffmap_ros/cliffmap.hpp>

// This package headers
#include <smp/samplers/cliff.hpp>

// SMP HEADER FILES ------
#include <smp/collision_checkers/multiple_circles_mrpt.hpp>
#include <smp/distance_evaluators/kdtree.hpp>
#include <smp/extenders/dubins.hpp>
#include <smp/multipurpose/minimum_time_reachability.hpp>
#include <smp/planners/rrtstar.hpp>

#include <smp/trajectory.hpp>
#include <smp/vertex_edge.hpp>

// ROS headers
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

// SMP TYPE DEFINITIONS -------
using State = smp::StateDubins;
using Input = smp::InputDubins;
using VertexData = smp::MTRVertexData;
using EdgeData = smp::MTREdgeData;
using Trajectory = smp::Trajectory<State, Input>;

namespace cliff_planners {

class DownTheCLiFFPlanner : public nav_core::BaseGlobalPlanner {

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  smp::samplers::CLiFF<State, 3> sampler;
  smp::extenders::Dubins extender;
  std::shared_ptr<smp::collision_checkers::MultipleCirclesMRPT<State, Input>>
      collision_checker;

  cliffmap_ros::CLiFFMapPtr cliffmap;

  ros::Publisher graph_pub;
  ros::Publisher marker_pub;
  ros::Publisher path_pub;
  ros::Publisher performance_pub;

  std::shared_ptr<mrpt::maps::COccupancyGridMap2D> map;
  std::shared_ptr<mrpt::math::CPolygon> footprint;

  // Debugging
  geometry_msgs::PoseArray graph;
  visualization_msgs::MarkerArray ma;

protected:
  virtual void initialize(std::string name,
                          costmap_2d::Costmap2DROS *costmap_ros);
  virtual bool makePlan(const geometry_msgs::PoseStamped &start,
                        const geometry_msgs::PoseStamped &goal,
                        std::vector<geometry_msgs::PoseStamped> &plan);

  /**
   * This function computes the Down-the-cliff cost.
   * This includes the cost of distance and the Mahalanobis cost.
   */
  double cost_function_cliff(State *state_initial_in,
                             Trajectory *trajectory_in,
                             State *state_final_in,
                             bool only_distance_cost, bool upstream_cost);

  double cost_function_upstream(State *state_initial_in,
                                Trajectory *trajectory_in,
                                State *state_final_in);

  std::vector<double> getSpeeds(State *state_initial_in,
                                Trajectory *trajectory_in);

  void publishPath(const Trajectory &trajectory_final,
                   std::vector<geometry_msgs::PoseStamped> &plan);

public:
  inline bool makePlan_1(const geometry_msgs::PoseStamped &start,
                         const geometry_msgs::PoseStamped &goal,
                         std::vector<geometry_msgs::PoseStamped> &plan) {
    makePlan(start, goal, plan);
  }
  void init(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  inline DownTheCLiFFPlanner() {}
  inline virtual ~DownTheCLiFFPlanner() {}
};

} // namespace cliff_planners

// Given a state and goal array, find the distance between them.
// Angles have a different distance formula. That is why we need this one.

std::array<double, 3> distanceBetweenStates(const std::array<double, 3> &state,
                                            const std::array<double, 3> &goal);

template <class State, class Input, class VertexData, class EdgeData>
void graphToMsg(ros::NodeHandle &nh, geometry_msgs::PoseArray &graph,
                smp::Vertex<State, Input, VertexData, EdgeData> *root) {
  geometry_msgs::Pose p;
  p.position.x = root->state->state_vars[0];
  p.position.y = root->state->state_vars[1];
  p.orientation.w = cos(root->state->state_vars[2] / 2);
  p.orientation.z = sin(root->state->state_vars[2] / 2);

  graph.poses.push_back(p);
  for (auto another_root : root->outgoing_edges) {
    graphToMsg(nh, graph, another_root->vertex_dst);
  }
}

