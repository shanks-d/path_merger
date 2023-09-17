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

class Merger : public rclcpp::Node
{
  public:
    Merger();

  private:
    void path1Callback(const Path::SharedPtr msg);

    void path2Callback(const Path::SharedPtr msg);

    void publishPath();

    // calculate the euclidean distance between two points
    float calcDist(const Point& p1, const Point& p2);

    // find the point index of path1 closest to the start of path2
    int findTransitionIndex();

    bool isInputValid();

    void mergePaths();

    Path path1_;
    Path path2_;
    Path mergedPath_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Path>::SharedPtr pubMergedPath_;
    rclcpp::Subscription<Path>::SharedPtr subPath1_;
    rclcpp::Subscription<Path>::SharedPtr subPath2_;
    // debouncing flags
    bool receivedPath1_ = false;
    bool receivedPath2_ = false;
    // constants
    const float distThreshold_ = 0.75;
    const int minPathLength_ = 20;
    // weights
    float alpha_ ;
    float rho_;
    float delta_;
    // parameters
    float beta_;
    float kTheta_;
    float gamma_;
    float currTheta_;
    float prevTheta_;
};