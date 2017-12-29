#include <cliff_planners/down_the_cliff_planner.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

#include <functional>
#include <thread>

class StandAlonePlanner {

protected:
  ros::NodeHandle nh;
  ros::Publisher path_pub;
  ros::Subscriber pose_sub;

  geometry_msgs::PoseStampedConstPtr start;
  geometry_msgs::PoseStampedConstPtr goal;

  void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose);

public:
  ros::ServiceClient make_plan_client;

  inline void reset() {start.reset(); goal.reset();}
  inline geometry_msgs::PoseStampedConstPtr getStart() { return start; }
  inline geometry_msgs::PoseStampedConstPtr getGoal() { return goal; }
  inline void publishPath(const nav_msgs::Path &path) { path_pub.publish(path); }
  inline StandAlonePlanner() {
    make_plan_client =
        nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    make_plan_client.waitForExistence();
    ROS_INFO("Connect to move base. Make plans by setting two navigation goals "
             "in rviz. The first one will be interpretted as the start pose. "
             "The second will be interpretted as the goal.");

    pose_sub = nh.subscribe("pose", 1, &StandAlonePlanner::poseCallback, this);

    path_pub = nh.advertise<nav_msgs::Path>("planned_path", 10);
  }
};

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

  ros::init(argn, args, "stand_alone_planner");
  StandAlonePlanner sap;

  std::thread spin_thread_t(spin);

  nav_msgs::GetPlan get_plan_msg;

  while (ros::ok()) {
    if (sap.getStart() && sap.getGoal()) {

      get_plan_msg.request.start = *(sap.getStart());
      get_plan_msg.request.goal = *(sap.getGoal());

      if (!sap.make_plan_client.call(get_plan_msg)) {
        ROS_FATAL("Couldn't call make plan. Is move_base running?");
        sleep(1);
      } else {
        get_plan_msg.response.plan.header.stamp = ros::Time::now();
        get_plan_msg.response.plan.header.frame_id = "map";
        sap.publishPath(get_plan_msg.response.plan);
        sap.reset();
      }
    } else {
      sleep(1);
      ROS_INFO("Waiting for start, end.");
    }
  }

  return 1;
}
