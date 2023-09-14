#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "path_merger/msg/path.hpp"

using std::placeholders::_1;
using Path = path_merger::msg::Path;
using Marker = visualization_msgs::msg::Marker;
using namespace std::chrono_literals;

class Visualize : public rclcpp::Node
{
  public:
    Visualize()
    : Node("visualize_paths"){
        subPath1_ = this->create_subscription<Path>("path1", 10,
        std::bind(&Visualize::path1Callback, this, _1));
        subPath2_ = this->create_subscription<Path>("path2", 10,
        std::bind(&Visualize::path2Callback, this, _1));
        subMergedPath_ = this->create_subscription<Path>("merged_path", 10,
        std::bind(&Visualize::mergedPathCallback, this, _1));

        pubPath_ = this->create_publisher<Marker>("path_markers", 10);

        visualizePaths();
        timer_ = this->create_wall_timer(
        500ms, std::bind(&Visualize::visualizePaths, this));
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

    void mergedPathCallback(const Path::SharedPtr msg){
        if(!receivedMergedPath_){
            mergedPath_.points = msg->points;
            receivedMergedPath_ = true;
            RCLCPP_INFO(this->get_logger(), "Subscribed to merged_path");
        }
    }

    void visualizePaths(){
        if(receivedPath1_ && receivedPath2_){
            Marker lineStrip1, lineStrip2, lineStripMerged;
            lineStrip1.header.frame_id = lineStrip2.header.frame_id = lineStripMerged.header.frame_id = "/visualize";
            lineStrip1.header.stamp = lineStrip2.header.stamp = lineStripMerged.header.stamp = this->get_clock()->now();
            
            lineStrip1.ns = "path1";
            lineStrip2.ns = "path2";
            lineStripMerged.ns = "merged_path";

            lineStrip1.action = lineStrip2.action = lineStripMerged.action = Marker::ADD;
            lineStrip1.pose.orientation.w = lineStrip2.pose.orientation.w = lineStripMerged.pose.orientation.w = 1.0;

            // lineStripMerged.id = 0;
            // lineStrip1.id = 1;
            // lineStrip2.id = 2;

            lineStrip1.type = Marker::LINE_STRIP;
            lineStrip2.type = Marker::LINE_STRIP;
            lineStripMerged.type = Marker::LINE_STRIP;

            lineStrip1.scale.x = lineStrip2.scale.x = lineStripMerged.scale.x = 0.1;

            // Path1 is red
            lineStrip1.color.r = 1.0;
            lineStrip1.color.a = 1.0;

            // Path2 is green
            lineStrip2.color.g = 1.0;
            lineStrip2.color.a = 1.0;

            // Merged Path is blue
            lineStripMerged.color.b = 1.0;
            lineStripMerged.color.a = 1.0;

            lineStrip1.points = path1_.points;
            lineStrip2.points = path2_.points;

            pubPath_->publish(lineStrip1);
            pubPath_->publish(lineStrip2);
            
            if(receivedMergedPath_){
                lineStripMerged.points = mergedPath_.points;
                pubPath_->publish(lineStripMerged);
            }
        }
    }

    Path path1_;
    Path path2_;
    Path mergedPath_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Marker>::SharedPtr pubPath_;
    rclcpp::Subscription<Path>::SharedPtr subPath1_;
    rclcpp::Subscription<Path>::SharedPtr subPath2_;
    rclcpp::Subscription<Path>::SharedPtr subMergedPath_;
    bool receivedPath1_ = false;
    bool receivedPath2_ = false;
    bool receivedMergedPath_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualize>());
    rclcpp::shutdown();
    return 0;
}