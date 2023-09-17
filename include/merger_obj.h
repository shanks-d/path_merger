#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "path_merger/msg/path.hpp"

using std::placeholders::_1;
typedef path_merger::msg::Path Path;
typedef geometry_msgs::msg::Point Point;
using namespace std::chrono_literals;

// @brief Main class that merges the inputs paths and publishes to a new topic
class Merger : public rclcpp::Node
{
  public:
    // @brief Constructor for the Merger class
    Merger();

  private:
    // @brief Subscriber callback to receive path1 information
    void path1Callback(const Path::SharedPtr msg);

    // @brief Subscriber callback to receive path2 information
    void path2Callback(const Path::SharedPtr msg);

    // @brief Publishes the computed merged path to 'merged_path' topic
    void publishPath();

    // @brief Returns the euclidean distance between two points
    // @param p1 Point 1
    // @param p2 Point 2
    float calcDist(const Point& p1, const Point& p2);

    // @brief Returns the index of path waypoint closest to the target point
    // @param path Path to be search for the closest waypoint index to target
    // @param point Target point
    int findTransitionIndex(const Path& path, const Point& point);

    // @brief Returns the result of the path validity checks
    // @param path1 Input path 1
    // @param path2 Input path 2
    bool isInputValid(const Path& path1, const Path& path2);

    // @brief Merges the input paths using weighted vector addition
    void mergePaths();

    Path path1_;                                            // stores path1 information
    Path path2_;                                            // stores path2 information
    Path mergedPath_;                                       // stores merged path information
    rclcpp::TimerBase::SharedPtr timer_;                    // timer to continuously call for merging of input paths
    rclcpp::Publisher<Path>::SharedPtr pubMergedPath_;      // publishes merged path information
    rclcpp::Subscription<Path>::SharedPtr subPath1_;        // subscribes path1 information
    rclcpp::Subscription<Path>::SharedPtr subPath2_;        // subscribes path2 information
    bool receivedPath1_ = false;                            // indicates subscription of path1 information
    bool receivedPath2_ = false;                            // indicates subscription of path2 information
    const float distThreshold_ = 0.75;                      // maximum distance between two waypoints
    const int minPathLength_ = 20;                          // minimum number of waypoints in a path
    float alpha_;                                           // weight of the path1 component
    float rho_;                                             // weight of the inertial component
    float delta_;                                           // weight of the differential component
    float beta_;                                            // fixed decrement of alpha at every iteration
    float gamma_;                                           // derivative value to update alpha according to the change in theta 
    float kTheta_;                                          // constant of proportionality between gamma and the change in theta
    float currTheta_;                                       // current theta angle between resultant and path2 component
    float prevTheta_;                                       // previous angle between resultant and path2 component
};