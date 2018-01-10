// Standard header files
#include <iostream>
#include <memory>

// CLiFFMapROS headers
#include <cliffmap_ros/cliffmap.hpp>

// This package headers
#include <smp/components/samplers/cliff.hpp>

// SMP HEADER FILES ------
#include <smp/components/collision_checkers/multiple_circles_mrpt.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/dubins.hpp>
#include <smp/components/extenders/single_integrator.hpp>
#include <smp/components/multipurpose/minimum_time_reachability.hpp>
#include <smp/planners/rrtstar.hpp>

#include <smp/planner_utils/trajectory.hpp>
#include <smp/planner_utils/vertex_edge.hpp>

// ROS headers
#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// SMP TYPE DEFINITIONS -------
// State, input, vertex_data, and edge_data definitions
typedef smp::state_dubins StateDubins;
typedef smp::input_dubins InputDubins;
typedef smp::minimum_time_reachability_vertex_data vertex_data_t;
typedef smp::minimum_time_reachability_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
  typedef StateDubins state;
  typedef InputDubins input;
  typedef vertex_data_t vertex_data;
  typedef edge_data_t edge_data;
} typeparams;

// Define the trajectory type
typedef smp::trajectory<typeparams> trajectory_t;

// Define all planner component types
typedef smp::sampler_cliff<typeparams, 3> CLiFFMapSampler;
typedef smp::distance_evaluator_kdtree<typeparams, 3> KDTreeDistanceEvaluator;
typedef smp::extender_dubins<typeparams> ExtenderDubins;
typedef smp::collision_checker_mc_mrpt<typeparams> CollisionCheckerMCMRPT;
typedef smp::minimum_time_reachability<typeparams, 3> MinimumTimeReachability;

// Define all algorithm types
typedef smp::rrtstar<typeparams> RRTStar;

namespace cliff_planners {

class DownTheCLiFFPlanner : public nav_core::BaseGlobalPlanner {

private:
  ros::NodeHandle nh;
  CLiFFMapSampler sampler;
  ExtenderDubins extender;
  std::shared_ptr<CollisionCheckerMCMRPT> collision_checker;

  cliffmap_ros::CLiFFMapPtr cliffmap;

  ros::Publisher graph_pub;
  ros::Publisher marker_pub;
  ros::Publisher path_pub;

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
  double cost_function_cliff(typeparams::state *state_initial_in,
                             trajectory_t *trajectory_in,
                             typeparams::state *state_final_in);

  void publishPath(const trajectory_t &trajectory_final,
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

template <typename T>
void graphToMsg(ros::NodeHandle &nh, geometry_msgs::PoseArray &graph,
                smp::vertex<T> *root) {
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

template <typename T> void freeGraph(smp::vertex<T> *root) {
  for (auto another_root : root->outgoing_edges) {
    freeGraph(another_root->vertex_dst);
  }
  delete root;
}
