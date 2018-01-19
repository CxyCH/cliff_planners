#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/filesystem.hpp>
#include <fstream>

struct Data {
  double pt;
  double cost;
  long int nodes;
};

int main(int argn, char *argv[]) {

  rosbag::Bag bag;

  boost::filesystem::path p(argv[1]);
  boost::filesystem::directory_iterator end_itr;

  std::fstream file_pm((argv[1] + std::string("-pm.txt")).c_str(),
                       std::fstream::out);
  std::fstream file_pm_first((argv[1] + std::string("-pm-first.txt")).c_str(),
                             std::fstream::out);
  std::fstream file_pm_last((argv[1] + std::string("-pm-last.txt")).c_str(),
                            std::fstream::out);

  std::fstream file_paths_first(
      (argv[1] + std::string("-paths-first.txt")).c_str(), std::fstream::out);
  std::fstream file_paths_last(
      (argv[1] + std::string("-paths-last.txt")).c_str(), std::fstream::out);

  // cycle through the directory
  for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr) {
    bag.open(itr->path().string(), rosbag::bagmode::Read);
    printf("Opened bag ... & ... ");
    fflush(stdout);
    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>(
                               {"/performance_measures"})));

    std::vector<std::vector<std::pair<double, double>>> paths;
    std::vector<Data> data;

    for (const auto m : view) {
      std_msgs::Float64MultiArray::ConstPtr pm =
          m.instantiate<std_msgs::Float64MultiArray>();
      if (pm != NULL) {
        data.push_back({pm->data[0], pm->data[1], (long int)pm->data[3]});
        file_pm << data.back().pt << "," << data.back().cost << ","
                << data.back().nodes << ";\n";
      }
    }

    file_pm << "\n";
    if (data.size() > 0) {
      file_pm_last << data.back().pt << "," << data.back().cost << ","
                   << data.back().nodes << ";\n";
      file_pm_first << data.front().pt << "," << data.front().cost << ","
                    << data.front().nodes << ";\n";
    }

    rosbag::View view_path(
        bag, rosbag::TopicQuery(std::vector<std::string>({"/path"})));
    for (rosbag::MessageInstance m : view_path) {
      nav_msgs::Path::ConstPtr path = m.instantiate<nav_msgs::Path>();
      if (path != NULL) {
        std::vector<std::pair<double, double>> pat;
        for (const auto &pose : path->poses) {
          pat.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        paths.push_back(pat);
      }
    }

    if (paths.size() > 0) {
      for (const auto &p : paths.back()) {
        file_paths_last << p.first << "," << p.second << ";";
      }

      for (const auto &p : paths.front()) {
        file_paths_first << p.first << "," << p.second << ";";
      }
    }
    file_paths_first << "\n";
    file_paths_last << "\n";

    bag.close();
    printf("Closed bag.\n");
  }

  return 1;
}
