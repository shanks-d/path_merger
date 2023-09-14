#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "path_merger/msg/path.hpp"

using Path = path_merger::msg::Path;
using Point = geometry_msgs::msg::Point;
using namespace std::chrono_literals;

// system path to the path_merger package
#define packageDir "/home/shanks/ros2_ws/src/path_merger"

// const std::vector<std::pair<float, float>> p1 {{1,0}, {2,0}, {3,0}, {4,0}, {5,0}, {6,0}, {7,0}, {8,0}, {9,0}};
// const std::vector<std::pair<float, float>> p2 {{5,-0.5}, {6,-1.5}, {7,-2}, {8,-3}, {9,-5}};

const std::unordered_map<int, std::string> scenarioNames {{1, "Lane Change"}};

class PathPublisher : public rclcpp::Node
{
  public:
    PathPublisher()
    : Node("input_paths_publisher"){
        this->declare_parameter("test_scenario", 1);    //default scenario 1

        // initializePaths();
        readPathData();

        pub1_ = this->create_publisher<Path>("path1", 10);
        timer1_ = this->create_wall_timer(1s, std::bind(&PathPublisher::pub1Callback, this));
      
        pub2_ = this->create_publisher<Path>("path2", 10);
        timer2_ = this->create_wall_timer(1s, std::bind(&PathPublisher::pub2Callback, this));
    }

  private:
    void readPathData(){
        
        int scenario = this->get_parameter("test_scenario").as_int();
        auto itr = scenarioNames.find(scenario);
        if(itr == scenarioNames.end()){
            throw std::runtime_error("Invalid scenario number");
        }

        RCLCPP_INFO(this->get_logger(), "Scenario %d selected: %s!", scenario, itr->second.c_str());
        std::string filename(packageDir);
        filename.append("/data/scenario" + std::to_string(scenario) + ".csv");

        // create an input filestream
        std::ifstream pathFile(filename);

        // make sure the file is open
        if(!pathFile.is_open()){
            throw std::runtime_error("Could not open file");
        }

        std::string line;

        // read and ignore the column names 
        std::getline(pathFile, line);

        // read data, line by line
        while(std::getline(pathFile, line)){
            // create a stringstream of the current line
            std::stringstream ss(line);
            
            // keep track of the index to populate the correct path
            int idx = 0;
            float val;
            Point p1, p2;
            
            // extract each integer
            while(ss >> val){                
                if(idx == 0){
                    p1.x = val;
                }
                else if(idx == 1){
                    p1.y = val;
                }
                else if(idx == 2){
                    p2.x = val;
                }
                else{
                    p2.y = val;
                }
                
                // ignore comma and move on
                if(ss.peek() == ',') ss.ignore();
                
                // increment the index
                idx++;
            }

            path1_.points.push_back(p1);
            path2_.points.push_back(p2);
        }

        // close file
        pathFile.close();

        // path headers
        rclcpp::Time now = this->get_clock()->now();
        path1_.header.stamp = now - rclcpp::Duration(500000000);  //time gap of 500ms
        path1_.header.frame_id = "path1";
        path2_.header.stamp = now;
        path2_.header.frame_id = "path2";
    }

    // void initializePaths(){
    //     // Path1
    //     rclcpp::Time now = this->get_clock()->now();
    //     path1_.header.stamp = now - rclcpp::Duration(500000000);  //time gap of 500ms
    //     path1_.header.frame_id = "path1";
    //     for(auto p: p1){
    //         Point point;
    //         point.x = p.first;
    //         point.y = p.second;
    //         path1_.points.push_back(point);
    //     }

    //     // Path2
    //     path2_.header.stamp = now;
    //     path2_.header.frame_id = "path2";
    //     for(auto p: p2){
    //         Point point;
    //         point.x = p.first;
    //         point.y = p.second;
    //         path2_.points.push_back(point);
    //     }
    // }

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