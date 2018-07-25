#include <cliff_planners/down_the_cliff_planner.hpp>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Core>

std::array<double, 3> distanceBetweenStates(const std::array<double, 3> &state,
                                            const std::array<double, 3> &goal) {

  std::array<double, 3> result;

  result[0] = state[0] - goal[0];
  result[1] = state[1] - goal[1];
  result[2] = state[2] - goal[2];
  result[2] = atan2(sin(result[2]), cos(result[2]));
  return result;
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
}

namespace cliff_planners {

void DownTheCLiFFPlanner::init(std::string name,
                               costmap_2d::Costmap2DROS *costmap_ros) {
  initialize(name, costmap_ros);
}

void DownTheCLiFFPlanner::initialize(std::string name,
                                     costmap_2d::Costmap2DROS *costmap_ros) {

  std::string cliffmapFileName;
  private_nh = ros::NodeHandle("~/" + name);
  private_nh.param<std::string>("cliffmap_file_name", cliffmapFileName, "");

  if (cliffmapFileName.empty()) {
    ROS_ERROR("No cliffmap file name parameter in namespace %s. Use simple RRT "
              "if cliffmap is unavailable.",
              private_nh.getNamespace().c_str());
    exit(1);
  }

  cliffmap = std::make_shared<cliffmap_ros::CLiFFMap>();
  cliffmap->readFromXML(cliffmapFileName);
  cliffmap->organizeAsGrid();

  ROS_INFO("Read a cliffmap XML organized as a grid @ %lf m/cell resolution.",
           cliffmap->getResolution());

  marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1);
  ma = cliffmap->toVisualizationMarkers();
  marker_pub.publish(ma);

  // TODO: All parameters must be configurable.
  graph_pub = nh.advertise<geometry_msgs::PoseArray>("/graph", 100);

  // Path publisher
  path_pub = nh.advertise<nav_msgs::Path>("/path", 10);

  // Performance measures
  performance_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/performance_measures", 100);

  map = std::make_shared<mrpt::maps::COccupancyGridMap2D>();
  mrptMapFromROSMsg(map, costmap_ros->getCostmap());

  std::vector<geometry_msgs::Point> footprint_pt =
      costmap_ros->getRobotFootprint();
  footprint = std::make_shared<mrpt::math::CPolygon>();

  for (const auto &point : footprint_pt) {
    footprint->AddVertex(point.x, point.y);
  }

  if (footprint_pt.size() != 4) {
    ROS_WARN(
        "Footprint wasn't a polygon. Setting to default values. Size was: %ld",
        footprint_pt.size());
    footprint->AddVertex(0.25, 0.125);
    footprint->AddVertex(0.25, -0.125);
    footprint->AddVertex(-0.25, 0.125);
    footprint->AddVertex(-0.25, -0.125);
  } else {
    ROS_INFO("DownTheCLiFFPlanner got a polygon footprint.");
  }

  collision_checker = std::make_shared<
      smp::collision_checkers::MultipleCirclesMRPT<State>>(map, 0.15,
                                                                  footprint);

  sampler.set_support(cliffmap);
}

bool DownTheCLiFFPlanner::makePlan(
    const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &goal,
    std::vector<geometry_msgs::PoseStamped> &plan) {

  smp::distance_evaluators::KDTree<State, Input, 3> distance_evaluator;
  smp::multipurpose::MinimumTimeReachability<State, Input, 3>
      min_time_reachability;

  marker_pub.publish(ma);

  // extender.set_turning_radius(0.5);

  std::string cost_type = "cliff";
  private_nh.param("cost_type", cost_type, std::string("cliff"));

  char blue[] = {0x1b, '[', '1', ';', '3', '4', 'm', 0};

  if (cost_type == "distance") {
    min_time_reachability.set_cost_function(std::bind(
        &DownTheCLiFFPlanner::cost_function_cliff, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, true, false));
    std::cout << blue << "ONLY USING DISTANCE COSTS.\n";
  } else if (cost_type == "cliff") {
    min_time_reachability.set_cost_function(std::bind(
        &DownTheCLiFFPlanner::cost_function_cliff, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, false, false));
    std::cout << blue << "USING CLIFF COSTS AND DISTANCE COSTS.\n";
  } else if (cost_type == "upstream") {
    min_time_reachability.set_cost_function(std::bind(
        &DownTheCLiFFPlanner::cost_function_cliff, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, false, true));
    std::cout << blue << "USING UPSTREAM COSTS AND DISTANCE COSTS.\n";
  }

  smp::planners::RRTStar<State, Input> planner(
      sampler, distance_evaluator, extender, *collision_checker,
      min_time_reachability, min_time_reachability);

  planner.parameters.set_phase(2);
  planner.parameters.set_gamma(std::max(map->getXMax(), map->getYMax()));
  planner.parameters.set_dimension(3);
  planner.parameters.set_max_radius(10.0);
  sampler.reset_rejections();

  ROS_INFO("Start: (%lf,%lf,%lf degrees)", start.pose.position.x,
           start.pose.position.y,
           2 * atan2(start.pose.orientation.z, start.pose.orientation.w) * 180 /
               M_PI);

  ROS_INFO("Going to goal: (%lf,%lf,%lf degrees)", goal.pose.position.x,
           goal.pose.position.y,
           2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w) * 180 /
               M_PI);

  smp::Region<3> region_goal;
  region_goal.center[0] = goal.pose.position.x;
  region_goal.size[0] = 0.1;

  region_goal.center[1] = goal.pose.position.y;
  region_goal.size[1] = 0.1;

  region_goal.center[2] =
      2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);
  region_goal.size[2] = 0.1;

  min_time_reachability.set_goal_region(region_goal);
  min_time_reachability.set_distance_function(distanceBetweenStates);

  sampler.set_goal_bias(0.05, region_goal);

  State *state_initial = new State;

  state_initial->state_vars[0] = start.pose.position.x;
  state_initial->state_vars[1] = start.pose.position.y;
  state_initial->state_vars[2] =
      2 * atan2(start.pose.orientation.z, start.pose.orientation.w);

  if (collision_checker->check_collision(state_initial) == 0) {
    ROS_ERROR("Start state is in collision. Planning failed.");
    return false;
  } else
    ROS_INFO("Start state is not in collision.");

  planner.initialize(state_initial);

  ros::Time t = ros::Time::now();
  // 3. RUN THE PLANNER
  int i = 0;

  double planning_time = 5.0;
  private_nh.param("planning_time", planning_time, 60.0);
  std::cout << blue << "Planning time limit is " << planning_time
            << " seconds.\n";

  bool no_cliff_sampling = false;
  private_nh.param("no_cliff_sampling", no_cliff_sampling, false);

  if (no_cliff_sampling) {
    std::cout << blue << "Using Uniform Sampler.\n";
    sampler.dontUseCLiFFSampling();
  } else {
    std::cout << blue << "Using CLiFFMap Sampler.\n";
  }

  double last_best_cost = -1.0;

  graph.header.frame_id = "map";
  bool no_soln = true;

  std::chrono::steady_clock clock;
  std::chrono::duration<long int, std::ratio<1l, 1000000000l>> total_time =
      clock.now() - clock.now();

  Trajectory trajectory_final;
  while (ros::ok()) {
    auto start_time = clock.now();
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

    if (min_time_reachability.get_best_cost() > 0.0) {
      if (min_time_reachability.get_best_cost() < last_best_cost || no_soln) {
        ROS_INFO("New solution found after %lf seconds.",
                 planner.get_planning_time());

        last_best_cost = min_time_reachability.get_best_cost();
        min_time_reachability.get_solution(trajectory_final);
        ROS_INFO("New solution has: %ld states and %ld inputs.",
                 trajectory_final.list_states.size(),
                 trajectory_final.list_inputs.size());

        publishPath(trajectory_final, plan);

        // Performance measures
        std_msgs::Float64MultiArray performance_msg;
        // Time to solution;
        performance_msg.data.push_back(planner.get_planning_time());
        // Cost of solution;
        performance_msg.data.push_back(cost_function_cliff(
            state_initial, &trajectory_final, NULL, false, false));
        // Distance cost
        performance_msg.data.push_back(cost_function_cliff(
            state_initial, &trajectory_final, NULL, true, false));
        // Upstream cost
        performance_msg.data.push_back(cost_function_cliff(
            state_initial, &trajectory_final, NULL, false, true));

        // Nodes in tree
        performance_msg.data.push_back(graph.poses.size());
        performance_pub.publish(performance_msg);
        no_soln = false;
      }
    }

    // ROS_INFO_THROTTLE(1.0, "Planner iteration : %d. Rejections: %u", i,
    //                  sampler.get_total_rejections());
    auto end_time = clock.now();
    total_time += end_time - start_time;
  }

  ROS_INFO("Total time: %lf seconds", total_time.count() / 1e9);
  if (plan.empty()) {
    ROS_ERROR("NO PLAN FOUND!");
  }

  return true;
}

void DownTheCLiFFPlanner::publishPath(
    const Trajectory &trajectory_final,
    std::vector<geometry_msgs::PoseStamped> &plan) {
  plan.clear();
  nav_msgs::Path path;

  for (const auto &state : trajectory_final.list_states) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = (*state)[0];
    pose.pose.position.y = (*state)[1];
    pose.pose.orientation.z = sin((*state)[2] / 2);
    pose.pose.orientation.w = cos((*state)[2] / 2);
    plan.push_back(pose);
  }

  int k = 0;
  ros::Duration d(0);
  for (const auto &time : trajectory_final.list_inputs) {
    d = d + ros::Duration((*time)[0]);
    plan[k].header.seq = k;
    plan[k].header.stamp = ros::Time(0) + d;
    plan[k].header.frame_id = "map";
    k++;
  }

  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path.poses = plan;
  path_pub.publish(path);
}

std::vector<double> DownTheCLiFFPlanner::getSpeeds(State *state_initial_in,
                                                   Trajectory *trajectory_in) {
  std::vector<double> speeds;

  State *state_prev = state_initial_in;

  std::list<State *>::iterator iter_state = trajectory_in->list_states.begin();

  for (std::list<Input *>::iterator iter_input =
           trajectory_in->list_inputs.begin();
       iter_input != trajectory_in->list_inputs.end(); iter_input++) {
    Input *input_curr = *iter_input;
    State *state_curr = *iter_state;

    // 1. Compute distance between the previous and current state.
    double this_distance_sq =
        ((state_curr->state_vars[0] - state_prev->state_vars[0]) *
         (state_curr->state_vars[0] - state_prev->state_vars[0])) +
        ((state_curr->state_vars[1] - state_prev->state_vars[1]) *
         (state_curr->state_vars[1] - state_prev->state_vars[1]));

    double this_time = (*input_curr)[0];

    speeds.push_back(sqrt(this_distance_sq) / this_time);

    state_prev = *iter_state;
    iter_state++;
  }
  return speeds;
}

double DownTheCLiFFPlanner::cost_function_cliff(State *state_initial_in,
                                                Trajectory *trajectory_in,
                                                State *state_final_in,
                                                bool only_distance_cost,
                                                bool upstream_cost) {

  if (upstream_cost)
    return cost_function_upstream(state_initial_in, trajectory_in,
                                  state_final_in);

  State *state_prev = state_initial_in;
  double total_cost = 0.0;

  std::list<State *>::iterator iter_state = trajectory_in->list_states.begin();

  for (std::list<Input *>::iterator iter_input =
           trajectory_in->list_inputs.begin();
       iter_input != trajectory_in->list_inputs.end(); iter_input++) {
    Input *input_curr = *iter_input;
    State *state_curr = *iter_state;

    // 1. Compute distance between the previous and current state.
    double this_distance_sq =
        pow(state_curr->state_vars[0] - state_prev->state_vars[0], 2) +
        pow(state_curr->state_vars[1] - state_prev->state_vars[1], 2);

    // 2. Compute the quaternion distance.
    double dot = cos(state_curr->state_vars[2] / 2.0) *
                     cos(state_prev->state_vars[2] / 2.0) +
                 sin(state_curr->state_vars[2] / 2.0 *
                     sin(state_prev->state_vars[2] / 2.0));

    double q_dist = pow(1.0 - fabs(dot), 2);

    // Total distance for now.
    double distance_cost = sqrt(this_distance_sq) + 10.0 * sqrt(q_dist);

    double this_time = (*input_curr)[0];

    double cliffcost = 0.0;
    Eigen::Vector2d V;
    V[0] = state_curr->state_vars[2];
    V[1] = 1.0; // sqrt(this_distance_sq) / this_time;
    double x = state_curr->state_vars[0];
    double y = state_curr->state_vars[1];
    double trust = (*cliffmap)(x, y).p * (*cliffmap)(x, y).q;
    for (const auto &dist : (*cliffmap)(x, y).distributions) {
      Eigen::Matrix2d Sigma;
      std::array<double, 4> sigma_array = dist.getCovariance();
      Sigma(0, 0) = sigma_array[0];
      Sigma(0, 1) = sigma_array[1];
      Sigma(1, 0) = sigma_array[2];
      Sigma(1, 1) = sigma_array[3];
      Eigen::Vector2d myu;
      myu[0] = dist.getMeanHeading();
      myu[1] = dist.getMeanSpeed();

      double inc_cost = 0.0;
      if (Sigma.determinant() < 1e-4 && Sigma.determinant() > -1e-4)
        inc_cost = 10000.00;
      else
        inc_cost =
            sqrt((V - myu).transpose() * Sigma.inverse() * (V - myu)) * trust;

      if (inc_cost < 0.0) {
        printf("WHAT THE HOLY?!");
      }
      // if (inc_cost > 10000.00)
      //  inc_cost = 10000.00f;

      cliffcost += inc_cost;

      if (std::isnan(cliffcost)) {
        std::cout << "Sigma: " << Sigma << std::endl;
        std::cout << "V: " << V << std::endl;
        std::cout << "myu: " << myu << std::endl;
        std::cout << "Trust: " << trust << std::endl;
        std::cout << "Incremental: " << inc_cost << std::endl;
        printf("cliffcost: %lf\n", cliffcost);
        printf("_________________________\n");
        std::cout << "SHIT!" << std::endl;
      }
    }
    // 3. Add the cliffcost
    if (only_distance_cost) {
      total_cost = total_cost + distance_cost;
    } else {
      total_cost = total_cost + distance_cost + (cliffcost / 5.0);
    }

    state_prev = *iter_state;
    iter_state++;
  }
  return total_cost;
}

double DownTheCLiFFPlanner::cost_function_upstream(State *state_initial_in,
                                                   Trajectory *trajectory_in,
                                                   State *state_final_in) {

  State *state_prev = state_initial_in;
  double total_cost = 0.0;

  std::list<State *>::iterator iter_state = trajectory_in->list_states.begin();

  for (std::list<Input *>::iterator iter_input =
           trajectory_in->list_inputs.begin();
       iter_input != trajectory_in->list_inputs.end(); iter_input++) {
    Input *input_curr = *iter_input;
    State *state_curr = *iter_state;

    // 1. Compute distance between the previous and current state.
    double this_distance_sq =
        pow(state_curr->state_vars[0] - state_prev->state_vars[0], 2) +
        pow(state_curr->state_vars[1] - state_prev->state_vars[1], 2);

    // 2. Compute the quaternion distance.
    double dot = cos(state_curr->state_vars[2] / 2.0) *
                     cos(state_prev->state_vars[2] / 2.0) +
                 sin(state_curr->state_vars[2] / 2.0 *
                     sin(state_prev->state_vars[2] / 2.0));

    double q_dist = pow(1.0 - fabs(dot), 2);

    // Total distance for now.
    double distance_cost = sqrt(this_distance_sq + q_dist);

    double this_time = (*input_curr)[0];

    double upstreamCost = 0.0;
    Eigen::Vector2d V;
    V[0] = state_curr->state_vars[2];
    V[1] = sqrt(this_distance_sq) / this_time;
    double x = state_curr->state_vars[0];
    double y = state_curr->state_vars[1];
    for (const auto &dist : (*cliffmap)(x, y).distributions) {
      Eigen::Vector2d myu;
      myu[0] = dist.getMeanHeading();
      myu[1] = dist.getMeanSpeed();

      double inc_cost = myu[1] - (myu[1] * cos(myu[0]) * cos(V[0]) +
                                  myu[1] * sin(myu[0]) * sin(V[0]));

      upstreamCost += inc_cost;

      if (std::isnan(upstreamCost)) {
        std::cout << "V: " << V << std::endl;
        std::cout << "myu: " << myu << std::endl;
        std::cout << "Incremental: " << inc_cost << std::endl;
        printf("upstreamCost: %lf\n", upstreamCost);
        printf("_________________________\n");
        std::cout << "SHIT!" << std::endl;
      }
    }
    // 3. Add the upstreamCost
    total_cost = total_cost + distance_cost + upstreamCost;

    state_prev = *iter_state;
    iter_state++;
  }
  return total_cost;
}
} // namespace cliff_planners

PLUGINLIB_EXPORT_CLASS(cliff_planners::DownTheCLiFFPlanner,
                       nav_core::BaseGlobalPlanner)
