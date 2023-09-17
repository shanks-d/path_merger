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

typedef path_merger::msg::Path Path;
typedef geometry_msgs::msg::Point Point;
using namespace std::chrono_literals;

// system path to the path_merger package
#define packageDir "/home/shanks/ros2_ws/src/path_merger"

// mapping scenario numbers to scenario names
const std::unordered_map<int, std::string> scenarioNames {{1, "Lane Change"}, 
                                                          {2, "Right Turn"}, 
                                                          {3, "U Turn"},
                                                          {4, "Quick Overtake"}, 
                                                          {5, "Stop Sign"},
                                                          {6, "Two Turns"},
                                                          {7, "Failed Continuity"},
                                                          {8, "Lost Track"}};

// @brief Main class to load test data and publish to the path topics
class PathPublisher : public rclcpp::Node
{
  public:
    // @brief Constructor for the publisher class
    PathPublisher()
    : Node("input_paths_publisher"){
        // create a ros parameter to input scenario when running the node
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter selects which test data to publish to the path topics (default: 1)";
        this->declare_parameter("test_scenario", 1);    //default scenario 1

        RCLCPP_INFO(this->get_logger(), "Reading path data...");
        readPathData();
        RCLCPP_INFO(this->get_logger(), "Input paths loaded");

        // create publishers for 'path1' and 'path2' topics 
        pub1_ = this->create_publisher<Path>("path1", 10);
        pub2_ = this->create_publisher<Path>("path2", 10);
        // create a timer event to publish to the path topics while the node is active
        timer_ = this->create_wall_timer(500ms, std::bind(&PathPublisher::pubCallback, this));
    }

  private:
    // @brief Opens the selected data file to read path waypoints
    void readPathData(){
        // get the entered test scenario number        
        int test_scenario = this->get_parameter("test_scenario").as_int();
        
        // make sure the entered test scenario number is a valid input
        auto scenario = scenarioNames.find(test_scenario);
        if(scenario == scenarioNames.end()){
            throw "ArgumentError: Invalid scenario number entered, select between 1 and 7";
        }
        RCLCPP_INFO(this->get_logger(), "Scenario %d selected: %s!", test_scenario, scenario->second.c_str());
        
        // creating system path to the data file in the package
        std::string filename(packageDir);
        filename.append("/data/scenario" + std::to_string(test_scenario) + ".csv");

        // create an input filestream
        std::ifstream pathFile(filename);

        // make sure the file is open
        if(!pathFile.is_open()){
            throw "PathError: Could not open file, make sure package directory is correct";
        }

        // variable to store streamed information 
        std::string line;

        // read and ignore the data in the first line (column names)
        std::getline(pathFile, line);

        // read data, line by line
        while(std::getline(pathFile, line)){
            // create a stringstream of the current line
            std::stringstream ss(line);
            
            // keep track of the index to populate the correct path
            int idx = 0;
            // store element-wise data of the current line streamed
            float val;
            // construct coordinates as waypoints
            Point p1, p2;
            
            // extract each element of the current line
            while(ss >> val){
                // data in each line represents 2D coordinates: x1, y1, x2, y2 (in-order)
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

            // add the waypoints to the path variables
            path1_.points.push_back(p1);
            path2_.points.push_back(p2);
        }

        // close file
        pathFile.close();

        // add path metadata to path headers
        rclcpp::Time now = this->get_clock()->now();
        path1_.header.stamp = now - rclcpp::Duration(500000000);  // assuming a time gap of 500ms between the two plans
        path1_.header.frame_id = "path1";
        path2_.header.stamp = now;
        path2_.header.frame_id = "path2";
        path1_.scenario = path2_.scenario = scenario->second;
    }
    
    // @brief Publishes the loaded path data to their respective path topics
    void pubCallback(){
        pub1_->publish(path1_);
        pub2_->publish(path2_);
        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing input paths...");
    }

    Path path1_;                                    // stores path1 information
    Path path2_;                                    // stores path2 information
    rclcpp::TimerBase::SharedPtr timer_;            // timer to continuously publish data to the path topics
    rclcpp::Publisher<Path>::SharedPtr pub1_;       // publishes path1 information
    rclcpp::Publisher<Path>::SharedPtr pub2_;       // publishes path2 information
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try{
        rclcpp::spin(std::make_shared<PathPublisher>());
    }
    catch(char const* error){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), error);
    }
    
    rclcpp::shutdown();
    return 0;
}