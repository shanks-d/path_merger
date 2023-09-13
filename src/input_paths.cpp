#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "path_merger/msg/path.hpp"

using Path = path_merger::msg::Path;
using Point = geometry_msgs::msg::Point;
using namespace std::chrono_literals;

const std::vector<std::pair<float, float>> p1 {{1,0}, {2,0}, {3,0}, {4,0}, {5,0}, {6,0}, {7,0}, {8,0}, {9,0}};
const std::vector<std::pair<float, float>> p2 {{5,-0.5}, {6,-1.5}, {7,-2}, {8,-3}, {9,-5}};

class PathPublisher : public rclcpp::Node
{
  public:
    PathPublisher()
    : Node("Input_Paths_publisher"){
        initializePaths();

        pub1_ = this->create_publisher<Path>("path1", 10);
        timer1_ = this->create_wall_timer(1s, std::bind(&PathPublisher::pub1Callback, this));
      
        pub2_ = this->create_publisher<Path>("path2", 10);
        timer2_ = this->create_wall_timer(1s, std::bind(&PathPublisher::pub2Callback, this));
    }

  private:
    void initializePaths(){
        // Path1
        rclcpp::Time now = this->get_clock()->now();
        path1_.header.stamp = now - rclcpp::Duration(500000000);  //time gap of 500ms
        path1_.header.frame_id = "path1";
        for(auto p: p1){
            Point point;
            point.x = p.first;
            point.y = p.second;
            path1_.points.push_back(point);
        }

        // Path2
        path2_.header.stamp = now;
        path2_.header.frame_id = "path2";
        for(auto p: p2){
            Point point;
            point.x = p.first;
            point.y = p.second;
            path2_.points.push_back(point);
        }
    }
    void pub1Callback(){
        pub1_->publish(path1_);
        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing input path1");
    }

    void pub2Callback(){
        pub2_->publish(path2_);
        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing input path2");
    }

    Path path1_;
    Path path2_;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Publisher<Path>::SharedPtr pub1_;
    rclcpp::Publisher<Path>::SharedPtr pub2_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}