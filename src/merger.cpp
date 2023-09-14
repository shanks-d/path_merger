#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
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
    }

    // calculate the euclidean distance between two points
    float calcDist(const Point& p1, const Point& p2){
        return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
    }

    // find the point index of path1 closest to the start of path2
    int findTransitionIndex(){
        int index = 0;
        float minDist = std::numeric_limits<float>::max();
        for(int p = 0; p < (int)path1_.points.size(); p++){
            float dist = calcDist(path1_.points[p], path2_.points[0]);
            if(dist < minDist){
                index = p;
                minDist = dist;
            }
        }
        return index;
    }

    void mergePaths(){
        // only performing merging operation once
        if(!publishedPath_){
            if(receivedPath1_ && receivedPath2_){
                int idx1 = findTransitionIndex();
                int idx2 = 0;
                // check if path1 is relevant
                if(calcDist(path1_.points[idx1], path2_.points[idx2]) >= distThreshold_){
                    RCLCPP_WARN(this->get_logger(), "Lost track of the old path");
                    mergedPath_.points = path2_.points;
                }
                else{
                    // copy all the points on path1 until the transition index
                    for(int i = 0; i < idx1; i++){
                        mergedPath_.points.push_back(path1_.points[i]);
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "Transition begins");

                    // iterate over the remaining path1 points
                    for(int i = idx1; i < (int)path1_.points.size(); i++){
                        // current ego pose
                        Point curr = mergedPath_.points.back();
                    
                        // calculate the path components
                        Point component1, component2;
                        component1.x = path1_.points[idx1].x - curr.x;
                        component1.y = path1_.points[idx1].y - curr.y;
                        component2.x = path2_.points[idx2].x - curr.x;
                        component2.y = path2_.points[idx2].y - curr.y;

                        // assign the weights
                        alpha_ -= beta_;

                        if(alpha_ > 1.0){
                            alpha_ = 1.0;
                        }
                        else if(alpha_ < 0.0){
                            alpha_ = 0.0;
                        }

                        // compute the resultant
                        Point resultant;
                        resultant.x = component1.x*alpha_ + component2.x*(1.0 - alpha_);
                        resultant.y = component1.y*alpha_ + component2.y*(1.0 - alpha_);

                        // debug logs
                        RCLCPP_INFO(this->get_logger(), "idx1: %d, idx2: %d, alpha: %f, beta: %f", idx1, idx2, alpha_, beta_);

                        // add the resultant to the merged_path
                        resultant.x += curr.x;
                        resultant.y += curr.y;
                        mergedPath_.points.push_back(resultant);

                        // adjust the weights
                        // beta_ = beta_;

                        // increment path indices
                        idx1++;
                        idx2++;
                    }

                    // adjust the weights
                    // beta_ = beta_;

                    // iterate over the rest of the path2
                    for(int i = idx2; i < (int)path2_.points.size(); i++){
                        // current ego pose
                        Point curr = mergedPath_.points.back();

                        // calculate the path components
                        Point component2, component3;
                        component2.x = path2_.points[idx2].x - curr.x;
                        component2.y = path2_.points[idx2].y - curr.y;
                        component3.x = path2_.points[idx2].x - path2_.points[idx2-1].x;
                        component3.y = path2_.points[idx2].y - path2_.points[idx2-1].y;

                        // assign the weights
                        alpha_ -= beta_;

                        if(alpha_ > 1.0){
                            alpha_ = 1.0;
                        }
                        else if(alpha_ < 0.0){
                            alpha_ = 0.0;
                        }

                        // compute the resultant
                        Point resultant;
                        resultant.x = component3.x*alpha_ + component2.x*(1.0 - alpha_);
                        resultant.y = component3.y*alpha_ + component2.y*(1.0 - alpha_);

                        // debug logs
                        RCLCPP_INFO(this->get_logger(), "idx2: %d, alpha: %f, beta: %f", idx2, alpha_, beta_);

                        // add the resultant to the merged_path
                        resultant.x += curr.x;
                        resultant.y += curr.y;
                        mergedPath_.points.push_back(resultant);

                        // adjust the weights
                        // beta_ = beta_;

                        // increment path indices
                        idx2++;
                    }
                }
                // publish the merged path
                publishPath();
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for the input paths to be subscribed...");
            }
        }
        else{
            // publish the merged path
            publishPath();
        }
    }

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
    bool publishedPath_ = false;
    // parameters
    const float distThreshold_ = 0.2;
    // weights
    float alpha_ = 1.0;
    float beta_ = 0.1;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Merger>());
    rclcpp::shutdown();
    return 0;
}