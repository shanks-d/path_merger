#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "path_merger/msg/path.hpp"

using std::placeholders::_1;
typedef path_merger::msg::Path Path;
typedef visualization_msgs::msg::Marker Marker;
using namespace std::chrono_literals;

// @brief Main class to subscribe and visualize paths 
class Visualize : public rclcpp::Node
{
  public:
  // @brief Constructor for the Visualize class
    Visualize()
    : Node("visualize_paths"){
        // create subscribers for 'path1', 'path2', and 'merged_path' topics
        subPath1_ = this->create_subscription<Path>("path1", 10, std::bind(&Visualize::path1Callback, this, _1));
        subPath2_ = this->create_subscription<Path>("path2", 10, std::bind(&Visualize::path2Callback, this, _1));
        subMergedPath_ = this->create_subscription<Path>("merged_path", 10, std::bind(&Visualize::mergedPathCallback, this, _1));
        // create publisher of visual markers of paths in rviz
        pubPath_ = this->create_publisher<Marker>("path_markers", 10);

        // initialize marker descriptions
        initializeMarkers();

        // create a timer event to publish the path markers while the node is active
        timer_ = this->create_wall_timer(500ms, std::bind(&Visualize::visualizePaths, this));
    }

  private:
    // @brief Subscriber callback to receive path1 information 
    void path1Callback(const Path::SharedPtr msg){
        // check if a new path scenario is received
        if(path1_.scenario != msg->scenario){
            // update the path information
            path1_.points = msg->points;
            path1_.scenario = msg->scenario;
            // set flag as received
            receivedPath1_ = true;
            // clear old merged path in case of new inputs
            mergedPath_.points.clear();
        }
    }

    // @brief Subscriber callback to receive path2 information
    void path2Callback(const Path::SharedPtr msg){
        // check if a new path scenario is received
        if(path2_.scenario != msg->scenario){
            // update the path information
            path2_.points = msg->points;
            path2_.scenario = msg->scenario;
            // set flag as received
            receivedPath2_ = true;
            // clear old merged path in case of new inputs
            mergedPath_.points.clear();
        }
    }

    // @brief Subscriber callback to receive merged_path information
    void mergedPathCallback(const Path::SharedPtr msg){
        // no checks for new infomation as merged_path is only published once
        // update the path information
        mergedPath_.points = msg->points;
        // set flag as received
        receivedMergedPath_ = true;
    }

    // @brief Sets the configuration of the visual markers
    void initializeMarkers(){
        lineStrip1_.header.frame_id = lineStrip2_.header.frame_id = lineStripMerged_.header.frame_id = "/visualize";
        linePoints1_.header.frame_id = linePoints2_.header.frame_id = linePointsMerged_.header.frame_id = "/visualize";

        lineStrip1_.ns = "path1";
        lineStrip2_.ns = "path2";
        lineStripMerged_.ns = "merged_path";
        
        linePoints1_.ns = "waypoints1";
        linePoints2_.ns = "waypoints2";
        linePointsMerged_.ns = "waypoints_merged";

        lineStrip1_.action = lineStrip2_.action = lineStripMerged_.action = Marker::ADD;
        linePoints1_.action = linePoints2_.action = linePointsMerged_.action = Marker::ADD;
        lineStrip1_.pose.orientation.w = lineStrip2_.pose.orientation.w = lineStripMerged_.pose.orientation.w = 1.0;
        linePoints1_.pose.orientation.w = linePoints2_.pose.orientation.w = linePointsMerged_.pose.orientation.w = 1.0;

        lineStrip1_.id = 0;
        lineStrip2_.id = 1;
        lineStripMerged_.id = 2;
        
        linePoints1_.id = 3;
        linePoints2_.id = 4;
        linePointsMerged_.id = 5;

        lineStrip1_.type = Marker::LINE_STRIP;
        lineStrip2_.type = Marker::LINE_STRIP;
        lineStripMerged_.type = Marker::LINE_STRIP;

        linePoints1_.type = Marker::POINTS;
        linePoints2_.type = Marker::POINTS;
        linePointsMerged_.type = Marker::POINTS;

        lineStrip1_.scale.x = lineStrip2_.scale.x = lineStripMerged_.scale.x = 0.1;

        linePoints1_.scale.x = linePoints2_.scale.x = linePointsMerged_.scale.x = 0.15;
        linePoints1_.scale.y = linePoints2_.scale.y = linePointsMerged_.scale.y = 0.15;

        // Path1 is red
        lineStrip1_.color.r = 0.8;
        lineStrip1_.color.a = 0.8;
        linePoints1_.color.r = 1.0;
        linePoints1_.color.a = 1.0;

        // Path2 is green
        lineStrip2_.color.g = 0.8;
        lineStrip2_.color.a = 0.8;
        linePoints2_.color.g = 1.0;
        linePoints2_.color.a = 1.0;

        // Merged Path is blue
        lineStripMerged_.color.b = 0.85;
        lineStripMerged_.color.a = 0.85;
        linePointsMerged_.color.b = 1.0;
        linePointsMerged_.color.a = 1.0;
    }

    // @brief Updates the markers with path data and publishes for visualization
    void visualizePaths(){
        // get current time
        rclcpp::Time now = this->get_clock()->now();

        // first visualize the input paths
        if(receivedPath1_ && receivedPath2_){
            lineStrip1_.header.stamp = lineStrip2_.header.stamp = now;
            linePoints1_.header.stamp = linePoints2_.header.stamp = now;

            lineStrip1_.points = path1_.points;
            linePoints1_.points = path1_.points;
            lineStrip2_.points = path2_.points;
            linePoints2_.points = path2_.points;

            pubPath_->publish(lineStrip1_);
            pubPath_->publish(linePoints1_);
            pubPath_->publish(lineStrip2_);
            pubPath_->publish(linePoints2_);
        }
        
        // now show the merged path
        if(receivedMergedPath_){
            lineStripMerged_.header.stamp = now;
            linePointsMerged_.header.stamp = now;

            lineStripMerged_.points = mergedPath_.points;
            linePointsMerged_.points = mergedPath_.points;
            
            pubPath_->publish(lineStripMerged_);
            pubPath_->publish(linePointsMerged_);
        }
    }

    Path path1_;                                                // stores path1 information
    Path path2_;                                                // stores path2 information
    Path mergedPath_;                                           // stores merged path information
    rclcpp::TimerBase::SharedPtr timer_;                        // timer to continuously publish visual markers of paths
    rclcpp::Publisher<Marker>::SharedPtr pubPath_;              // publishes visual markers of paths
    rclcpp::Subscription<Path>::SharedPtr subPath1_;            // subscribes path1 information
    rclcpp::Subscription<Path>::SharedPtr subPath2_;            // subscribes path2 information
    rclcpp::Subscription<Path>::SharedPtr subMergedPath_;       // subscribes merged_path information
    Marker lineStrip1_, lineStrip2_, lineStripMerged_;          // markers representing path segments 
    Marker linePoints1_, linePoints2_, linePointsMerged_;       // markers representing path waypoints
    bool receivedPath1_ = false;                                // indicates subscription of path1 information
    bool receivedPath2_ = false;                                // indicates subscription of path2 information
    bool receivedMergedPath_ = false;                           // indicates subscription of merged_path information
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualize>());
    rclcpp::shutdown();
    return 0;
}