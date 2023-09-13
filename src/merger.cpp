#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "path_merger/msg/path.hpp"

using std::placeholders::_1;
using Path = path_merger::msg::Path;
using Point = geometry_msgs::msg::Point;
using namespace std::chrono_literals;

class Merger : public rclcpp::Node
{
  public:
    Merger()
    : Node("path_merger"){
        subPath1_ = this->create_subscription<Path>("path1", 10,
        std::bind(&Merger::path1Callback, this, _1));
        subPath2_ = this->create_subscription<Path>("path2", 10,
        std::bind(&Merger::path2Callback, this, _1));

        pubMergedPath_ = this->create_publisher<Path>("merged_path", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&Merger::mergePaths, this));
    }

  private:
    void path1Callback(const Path::SharedPtr msg){
        if(!receivedPath1_){
            path1_.points = msg->points;
            receivedPath1_ = true;
            RCLCPP_INFO(this->get_logger(), "Subscribed to path1");
        }
    }

    void path2Callback(const Path::SharedPtr msg){
        if(!receivedPath2_){
            path2_.points = msg->points;
            receivedPath2_ = true;
            RCLCPP_INFO(this->get_logger(), "Subscribed to path2");
        }
    }

    void publishPath(){
        static rclcpp::Time now = this->get_clock()->now();
        mergedPath_.header.stamp = now;
        mergedPath_.header.frame_id = "mergedPath";
        pubMergedPath_->publish(mergedPath_);
        publishedPath_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "Merged path published!");

        // uncomment this to allow resubscription of new input paths
        // receivedPath1_ = receivedPath2_ = false;
    }

    void mergePaths(){
        if(receivedPath1_ && receivedPath2_){
            // consider path2 as the merged path
            mergedPath_.points = path2_.points;
            
            // publish the merged path
            publishPath();
        }
        RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for the input paths to be subscribed...");

        if(publishedPath_){
            // publish the merged path
            publishPath();
        }
    }

    Path path1_;
    Path path2_;
    Path mergedPath_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<path_merger::msg::Path>::SharedPtr pubMergedPath_;
    rclcpp::Subscription<path_merger::msg::Path>::SharedPtr subPath1_;
    rclcpp::Subscription<path_merger::msg::Path>::SharedPtr subPath2_;
    bool receivedPath1_ = false;
    bool receivedPath2_ = false;
    bool publishedPath_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Merger>());
    rclcpp::shutdown();
    return 0;
}