#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <string>
#include <cmath>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#define LANE_CHANGE 1
#define RIGHT_TURN 2
#define U_TURN 3

std::vector<float> path1_x, path1_y;
std::vector<float> path2_x, path2_y;

const float resolution = 0.25;   // meters per pixel

void laneChange(){
    // path1
    // straight segment 
    float y = -1.0;
    float x = 0.0;
    float x1 = -9.0, x2 = 2.0;
    int size = (x2 - x1)/resolution;
    for(int i = 0; i <= size; i++){
        x = x1 + i*resolution;
        path1_x.push_back(x);
        path1_y.push_back(y);
    }
    
    // path2
    // straight segment
    y = -1.0;
    x1 = -6.0;
    x2 = -5.0;
    size = (x2 - x1)/resolution;
    for(int i = 0; i < size; i++){
        x = x1 + i*resolution;
        path2_x.push_back(x);
        path2_y.push_back(y);
    }
    // curved segment
    x1 = -5.0;
    x2 = 5.0;
    size = (x2 - x1)/resolution;
    for(int i = 0; i <= size; i++){
        x = x1 + i*resolution;
        y = sin(0.1*M_PI*x);
        path2_x.push_back(x);
        path2_y.push_back(y);
    }

    // check if paths are of equal lengths 
    if(path1_x.size() != path2_x.size()){
        throw std::runtime_error("Unequal path lengths");
    }
}

void rightTurn(){
    // path1
    // straight segment 
    float y = 0.0;
    float x = 0.0;
    float x1 = -7.0, x2 = 4.0;
    int size = (x2 - x1)/resolution;
    for(int i = 0; i <= size; i++){
        x = x1 + i*resolution;
        path1_x.push_back(x);
        path1_y.push_back(y);
    }
    
    // path2
    // straight segment
    y = 0.0;
    x1 = -4.0;
    x2 = -3.0;
    size = (x2 - x1)/resolution;
    for(int i = 0; i < size; i++){
        x = x1 + i*resolution;
        path2_x.push_back(x);
        path2_y.push_back(y);
    }
    // curved segment
    x1 = -3.0;
    x2 = -0.5;
    size = (x2 - x1)/resolution;
    for(int i = 0; i < size; i++){
        x = x1 + i*resolution;
        y = 0.3/x + 0.1;
        path2_x.push_back(x);
        path2_y.push_back(y);
    }
    // curved segment
    float y1 = -0.5;
    float y2 = -3.0;
    size = (y1 - y2)/resolution;
    for(int i = 0; i < size; i++){
        y = y1 - i*resolution;
        x = 0.3/(y - 0.1);
        path2_x.push_back(x);
        path2_y.push_back(y);
    }
    // straight segment
    x = -0.1;
    y1 = -3.0;
    y2 = -8.0;
    size = (y1 - y2)/resolution;
    for(int i = 0; i <= size; i++){
        y = y1 - i*resolution;
        path2_x.push_back(x);
        path2_y.push_back(y);
    }

    // check if paths are of equal lengths 
    if(path1_x.size() != path2_x.size()){
        throw std::runtime_error("Unequal path lengths");
    }
}

void uTurn(){
    // path1
    // straight segment
    float y = 1.0;
    float x = 0.0;
    float x1 = -6.0, x2 = 3.0;
    int size = (x2 - x1)/resolution;
    for(int i = 0; i <= size; i++){
        x = x1 + i*resolution;
        path1_x.push_back(x);
        path1_y.push_back(y);
    }
    
    // path2
    // straight segment
    y = 1.0;
    x1 = -3.0;
    x2 = -2.0;
    size = (x2 - x1)/resolution;
    for(int i = 0; i < size; i++){
        x = x1 + i*resolution;
        path2_x.push_back(x);
        path2_y.push_back(y);
    }
    // curved segment
    float t = 0.0;
    float dt = 0.1;
    size = 10;
    for(int i = 0; i < size; i++){
        float ti = t + i*dt;
        x = 1.0*sin(M_PI*ti) - 2.0;
        y = 1.0*cos(M_PI*ti);
        path2_x.push_back(x);
        path2_y.push_back(y);
    }
    // straight segment
    y = -1.0;
    x1 = -2.0;
    x2 = -7.5;
    size = (x1 - x2)/resolution;
    for(int i = 0; i <= size; i++){
        x = x1 - i*resolution;
        path2_x.push_back(x);
        path2_y.push_back(y);
    }

    // check if paths are of equal lengths 
    if(path1_x.size() != path2_x.size()){
        throw std::runtime_error("Unequal path lengths");
    }
}

// shift path2 along Y axis 
void shift2InY(float h){
    for(int i = 0; i < (int)path2_y.size(); i++){
        path2_y[i] += h;
    }
}

// compute waypoints for different scenarios
void computePoints(int scenario){
    switch(scenario){
        case 1:
            laneChange();
            shift2InY(0.25);
            break;
        case 2:
            rightTurn();
            shift2InY(0.25);
            break;
        case 3:
            uTurn();
            shift2InY(-0.25);
            break;
        default:
            throw std::runtime_error("Invalid scenario");
            break;
    }
}

void writePathData(std::string filename){
    // create an output filestream object
    std::ofstream pathFile(filename);
    
    // send column names to the stream
    pathFile << "x1,y1,x2,y2" << std::endl;
    
    // send data to the stream
    int length = path1_x.size();
    for(int i = 0; i < length; i++){
        pathFile << path1_x[i] << "," << path1_y[i] << "," << path2_x[i] << "," << path2_y[i] << std::endl;
    }
    
    // close the file
    pathFile.close();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // scenarios: LANE_CHANGE, RIGHT_TURN U_TURN
    computePoints(U_TURN);
    
    // write the vector to CSV
    writePathData("scenario3.csv");

    rclcpp::shutdown();
    return 0;
}