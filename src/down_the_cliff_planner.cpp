#include <pluginlib/class_list_macros.h>

#include <cliff_planners/down_the_cliff_planner.hpp>

std::array<double, 3> distanceBetweenStates(const std::array<double, 3> &state,
                                            const std::array<double, 3> &goal) {

  std::array<double, 3> result;

  result[0] = state[0] - goal[0];
  result[1] = state[1] - goal[1];
  result[2] = state[2] - goal[2];
  result[2] = atan2(sin(result[2]), cos(result[2]));
  return result;
}

/**
 * This function computes the Down-the-cliff cost.
 * This includes the cost of distance and the Mahalanobis cost. 
 */

double cost_function_cliff(typeparams::state *state_initial_in,
                           trajectory_t *trajectory_in,
                           typeparams::state *state_final_in) {

  typedef typeparams::state state_t;
  typedef typeparams::input input_t;

  double total_time = 0.0;
  double total_distance = 0.0;
  state_t *state_prev = state_initial_in;

  std::list<state_t *>::iterator iter_state =
      trajectory_in->list_states.begin();
  for (std::list<input_t *>::iterator iter =
           trajectory_in->list_inputs.begin();
       iter != trajectory_in->list_inputs.end(); iter++) {
    input_t *input_curr = *iter;
    state_t *state_curr = *iter_state;

    std::array<double, 3> s_curr, s_prev;
    for (int i = 0; i < 3; i++) {
      s_curr[i] = state_curr->state_vars[i];
      s_prev[i] = state_prev->state_vars[i];
    }

    std::array<double, 3> p = distanceBetweenStates(s_curr, s_prev);
    double this_distance = 0.0;
    for (int i = 0; i < 3; i++) {
      this_distance += p[i] * p[i];
    }
    this_distance = sqrt(this_distance);

    total_distance += this_distance;
    state_prev = *iter_state;
    iter_state++;

    total_time += (*input_curr)[0];
  }

  // return total_time;
  return total_distance;
}

void mrptMapFromROSMsg(
    const std::shared_ptr<mrpt::maps::COccupancyGridMap2D> &map,
    const costmap_2d::Costmap2D *rosMap) {

  map->setSize(
      rosMap->getOriginX(), rosMap->getOriginX() + rosMap->getSizeInMetersX(),
      rosMap->getOriginY(), rosMap->getOriginY() + rosMap->getSizeInMetersY(),
      rosMap->getResolution(), 1.0);

  for (int h = 0; h < rosMap->getSizeInCellsY(); h++) {
    for (int w = 0; w < rosMap->getSizeInCellsX(); w++) {
      float value = -1.0f;
      const uint8_t &occ_map_value = rosMap->getCost(w, h);
      if (occ_map_value == 255 || occ_map_value == 254)
        value = 0.0f;
      else
        value = 1.0f;
      map->setCell(w, h, value);
    }
  }
  /* [DEBUG] */ map->saveAsBitmapFile("/home/chitt/remap.png");
}

namespace cliff_planners {

void DownTheCLiFFPlanner::initialize(std::string name,
                                     costmap_2d::Costmap2DROS *costmap_ros) {

  std::string cliffmapFileName;
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param<std::string>("cliffmap_file_name", cliffmapFileName, "");

  if(cliffmapFileName.empty()) {
    ROS_ERROR("No cliffmap file name provided. Use simple RRT if cliffmap is unavailable.");
    exit(1);
  }

  cliffmap.readFromXML(cliffmapFileName);
  cliffmap.organizeAsGrid(1.0);

  ROS_INFO("Read a cliffmap XML organized as a grid @ 1.0 m/cell resolution.");

  // TODO: All parameters must be configurable.
  graph_pub = nh.advertise<geometry_msgs::PoseArray>("/graph", 100);

  map = std::make_shared<mrpt::maps::COccupancyGridMap2D>();
  mrptMapFromROSMsg(map, costmap_ros->getCostmap());

  std::vector<geometry_msgs::Point> footprint_pt =
      costmap_ros->getRobotFootprint();
  footprint = std::make_shared<mrpt::math::CPolygon>();

  for (const auto &point : footprint_pt) {
    footprint->AddVertex(point.x, point.y);
  }

  if (footprint_pt.size() != 4) {
    ROS_WARN("Footprint wasn't a polygon. Setting to default values.");
    footprint->AddVertex(0.25, 0.125);
    footprint->AddVertex(0.25, -0.125);
    footprint->AddVertex(-0.25, 0.125);
    footprint->AddVertex(-0.25, -0.125);
  } else {
    ROS_INFO("DownTheCLiFFPlanner got a polygon footprint.");
  }

  // TODO: Inflation radius and footprint must be configurable.
  // sampler can't be std::shared_ptr
  collision_checker = std::shared_ptr<CollisionCheckerMCMRPT>(
      new CollisionCheckerMCMRPT(map, 0.15, footprint));

  // Sampler support should also be configurable.
  smp::region<3> sampler_support;
  sampler_support.center[0] = 0.0;
  sampler_support.size[0] = 10.0;
  sampler_support.center[1] = 0.0;
  sampler_support.size[1] = 10.0;
  sampler_support.center[2] = 0.0;
  sampler_support.size[2] = 3.14;
  sampler.set_support(sampler_support);
}

bool DownTheCLiFFPlanner::makePlan(
    const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &goal,
    std::vector<geometry_msgs::PoseStamped> &plan) {

  KDTreeDistanceEvaluator distance_evaluator;
  MinimumTimeReachability min_time_reachability;

  min_time_reachability.set_cost_function(cost_function_cliff);

  RRTStar planner(sampler, distance_evaluator, extender, *collision_checker,
                  min_time_reachability, min_time_reachability);

  planner.parameters.set_phase(2);
  planner.parameters.set_gamma(std::max(map->getXMax(), map->getYMax()));
  planner.parameters.set_dimension(3);
  planner.parameters.set_max_radius(10.0);

  ROS_INFO("Start: (%lf,%lf,%lf degrees)", start.pose.position.x,
           start.pose.position.y,
           2 * atan2(start.pose.orientation.z, start.pose.orientation.w) * 180 /
               M_PI);

  ROS_INFO("Going to goal: (%lf,%lf,%lf degrees)", goal.pose.position.x,
           goal.pose.position.y,
           2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w) * 180 /
               M_PI);

  ros::Publisher path_pub = nh.advertise<geometry_msgs::PoseArray>("/path", 10);

  smp::region<3> region_goal;
  region_goal.center[0] = goal.pose.position.x;
  region_goal.size[0] = 0.75;

  region_goal.center[1] = goal.pose.position.y;
  region_goal.size[1] = 0.75;

  region_goal.center[2] =
      2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);
  region_goal.size[2] = 0.2;

  min_time_reachability.set_goal_region(region_goal);
  min_time_reachability.set_distance_function(distanceBetweenStates);

  StateDubins *state_initial = new StateDubins;

  state_initial->state_vars[0] = start.pose.position.x;
  state_initial->state_vars[1] = start.pose.position.y;
  state_initial->state_vars[2] =
      2 * atan2(start.pose.orientation.z, start.pose.orientation.w);

  if (collision_checker->check_collision_state(state_initial) == 0) {
    ROS_INFO("Start state is in collision. Planning failed.");
    return false;
  } else
    ROS_INFO("Start state is not in collision.");

  planner.initialize(state_initial);

  ros::Time t = ros::Time::now();
  // 3. RUN THE PLANNER
  int i = 0;
  double planning_time = 5.0;

  graph.header.frame_id = "map";
  while (ros::ok()) {
    ++i;
    if (ros::Time::now() - t > ros::Duration(planning_time)) {
      ROS_INFO("Planning time of %lf sec. elapsed.", planning_time);
      break;
    }

    planner.iteration();

    graph.poses.clear();

    graphToMsg(nh, graph, planner.get_root_vertex());
    graph.header.stamp = ros::Time::now();

    graph_pub.publish(graph);
    ROS_INFO_THROTTLE(1.0, "Planner iteration : %d", i);
  }

  trajectory_t trajectory_final;
  min_time_reachability.get_solution(trajectory_final);

  geometry_msgs::PoseArray path;

  for (const auto &state : trajectory_final.list_states) {
    geometry_msgs::Pose p;
    p.position.x = (*state)[0];
    p.position.y = (*state)[1];
    p.orientation.z = sin((*state)[2] / 2);
    p.orientation.w = cos((*state)[2] / 2);
    path.poses.push_back(p);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = (*state)[0];
    pose.pose.position.y = (*state)[1];
    pose.pose.orientation.z = sin((*state)[2] / 2);
    pose.pose.orientation.w = cos((*state)[2] / 2);
    plan.push_back(pose);
  }

  int k = 0;
  for (const auto &time : trajectory_final.list_inputs) {
    plan[k].header.frame_id = "map";
    plan[k].header.stamp = ros::Time(0) + ros::Duration((*time)[0]);
    k++;
  }

  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path_pub.publish(path);

  sleep(2);

  return true;
}
}

PLUGINLIB_EXPORT_CLASS(cliff_planners::DownTheCLiFFPlanner,
                       nav_core::BaseGlobalPlanner)
