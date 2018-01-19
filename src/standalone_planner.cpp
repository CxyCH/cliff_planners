#include <cliff_planners/down_the_cliff_planner.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

#include <functional>
#include <thread>

class StandAlonePlanner {

protected:
  ros::NodeHandle nh;
  ros::Publisher path_pub;
  ros::Subscriber pose_sub;

  geometry_msgs::PoseStampedConstPtr start;
  geometry_msgs::PoseStampedConstPtr goal;

  tf::TransformListener tf;
  costmap_2d::Costmap2DROS costmap;
  ros::ServiceClient make_plan_client;
  ros::ServiceServer make_plan_server;

  cliff_planners::DownTheCLiFFPlanner dtcp;

  bool makePlanCallback(nav_msgs::GetPlan::Request &req,
                        nav_msgs::GetPlan::Response &res);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose);

public:
  StandAlonePlanner();

  inline void reset() {
    start.reset();
    goal.reset();
  }

  bool makePlan(nav_msgs::GetPlan &msg);

  inline geometry_msgs::PoseStampedConstPtr getStart() { return start; }

  inline geometry_msgs::PoseStampedConstPtr getGoal() { return goal; }

  inline void publishPath(const nav_msgs::Path &path) {
    path_pub.publish(path);
  }
};

bool StandAlonePlanner::makePlan(nav_msgs::GetPlan &msg) {
  if (!make_plan_client.call(msg)) {
    ROS_FATAL("Couldn't call make plan. Is move_base running?");
    sleep(1);
    return false;
  }
  sleep(1);
  return true;
}

bool StandAlonePlanner::makePlanCallback(nav_msgs::GetPlan::Request &req,
                                         nav_msgs::GetPlan::Response &res) {
  dtcp.makePlan_1(req.start, req.goal, res.plan.poses);
  res.plan.header.frame_id = "map";
  res.plan.header.stamp = ros::Time::now();
  return true;
}

StandAlonePlanner::StandAlonePlanner()
    : tf(ros::Duration(10)), costmap("my_costmap", tf) {
  make_plan_server = nh.advertiseService(
      "make_plan", &StandAlonePlanner::makePlanCallback, this);

  costmap.start();
  make_plan_client = nh.serviceClient<nav_msgs::GetPlan>("make_plan");
  make_plan_client.waitForExistence();
  ROS_INFO("Connect to move base. Make plans by setting two navigation goals\n"
           "in rviz. The first one will be interpretted as the start pose.\n"
           "The second will be interpretted as the goal.\n");

  pose_sub = nh.subscribe("pose", 1, &StandAlonePlanner::poseCallback, this);

  path_pub = nh.advertise<nav_msgs::Path>("planned_path", 10);
  dtcp.init("DownTheCLiFFPlanner", &costmap);
}

void StandAlonePlanner::poseCallback(
    const geometry_msgs::PoseStampedConstPtr &pose) {
  if (!start) {
    start = pose;
    ROS_INFO("Got start pose.");
    return;
  }
  if (!goal) {
    goal = pose;
    ROS_INFO("Got goal pose.");
    return;
  }

  ROS_INFO("Waiting for plan to complete. Send start and goal again.");
}

void spin() { ros::spin(); }

int main(int argn, char *args[]) {

  setlocale(LC_NUMERIC, "");
  ros::init(argn, args, "stand_alone_planner");
  StandAlonePlanner sap;

  std::thread spin_thread_t(spin);

  nav_msgs::GetPlan get_plan_msg;

  while (ros::ok()) {
    if (sap.getStart() && sap.getGoal()) {

      get_plan_msg.request.start = *(sap.getStart());
      get_plan_msg.request.goal = *(sap.getGoal());

      if (sap.makePlan(get_plan_msg)) {
        get_plan_msg.response.plan.header.stamp = ros::Time::now();
        get_plan_msg.response.plan.header.frame_id = "map";
        sap.publishPath(get_plan_msg.response.plan);
        sap.reset();
      }
    } else {
      usleep(10000);
    }
  }

  return 1;
}
