#include "merger_obj.h"

Merger::Merger()
    : Node("path_merger"){
        subPath1_ = this->create_subscription<Path>("path1", 10,
        std::bind(&Merger::path1Callback, this, _1));
        subPath2_ = this->create_subscription<Path>("path2", 10,
        std::bind(&Merger::path2Callback, this, _1));

        // creating tuning variables as rosparams
        this->declare_parameter("beta", 0.1);
        this->declare_parameter("k", 0.6);
        this->declare_parameter("rho", 1.0);
        this->declare_parameter("delta", 0.2);

        pubMergedPath_ = this->create_publisher<Path>("merged_path", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&Merger::mergePaths, this));
}

void Merger::path1Callback(const Path::SharedPtr msg){
    if(!receivedPath1_ && path1_.scenario != msg->scenario){
        path1_.points = msg->points;
        path1_.scenario = msg->scenario;
        receivedPath1_ = true;
        RCLCPP_INFO(this->get_logger(), "Subscribed to path1");
    }
}

void Merger::path2Callback(const Path::SharedPtr msg){
    if(!receivedPath2_ && path2_.scenario != msg->scenario){
        path2_.points = msg->points;
        path2_.scenario = msg->scenario;
        receivedPath2_ = true;
        RCLCPP_INFO(this->get_logger(), "Subscribed to path2");
    }
}

void Merger::publishPath(){
    mergedPath_.header.stamp = this->get_clock()->now();
    mergedPath_.header.frame_id = "mergedPath";
    mergedPath_.scenario = path1_.scenario;
    pubMergedPath_->publish(mergedPath_);
    RCLCPP_INFO(this->get_logger(), "Merged path published!");
}

// calculate the euclidean distance between two points
float Merger::calcDist(const Point& p1, const Point& p2){
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

// find the point index of path1 closest to the start of path2
int Merger::findTransitionIndex(){
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

bool Merger::isInputValid(){
    bool isValid = true;

    // checking for path lengths
    RCLCPP_INFO(this->get_logger(), "Checking for path length requirements...");
    if(path1_.points.size() != path2_.points.size()){
        RCLCPP_WARN(this->get_logger(), "Unequal path lengths, path1: %d, path2: %d", path1_.points.size(), path2_.points.size());
    }
    if((int)path1_.points.size() < minPathLength_){
        RCLCPP_ERROR(this->get_logger(), "Path1 has %d waypoints but minimum required is %d", path1_.points.size(), minPathLength_);
        isValid = false;
    }
    if((int)path2_.points.size() < minPathLength_){
        RCLCPP_ERROR(this->get_logger(), "Path2 has %d waypoints but minimum required is %d", path2_.points.size(), minPathLength_);
        isValid = false;
    }

    // checking for continuity
    RCLCPP_INFO(this->get_logger(), "Checking for path continuity...");
    for(int i = 1; i < (int)path1_.points.size(); i++){
        if(calcDist(path1_.points[i], path1_.points[i-1]) >= distThreshold_){
            RCLCPP_ERROR_ONCE(this->get_logger(), "Path1 is inconsistent or not continuous");
            isValid = false;
        }
    }
    for(int i = 1; i < (int)path2_.points.size(); i++){
        if(calcDist(path2_.points[i], path2_.points[i-1]) >= distThreshold_){
            RCLCPP_ERROR_ONCE(this->get_logger(), "Path2 is inconsistent or not continuous");
            isValid = false;
        }
    }

    return isValid;
}

void Merger::mergePaths(){
    // perform merging when the input paths are subcribed
    if(receivedPath1_ && receivedPath2_){
        // check for path validity before merging
        if(isInputValid()){
            RCLCPP_INFO(this->get_logger(), "Input paths are valid!");

            int idx1 = findTransitionIndex();
            // check if path1 is relevant
            if(calcDist(path1_.points[idx1], path2_.points[0]) >= distThreshold_){
                RCLCPP_ERROR(this->get_logger(), "Lost track of the old path, considering new path as merged path");
                mergedPath_.points = path2_.points;
            }
            else{
                beta_ = this->get_parameter("beta").as_double();
                kTheta_ = this->get_parameter("k").as_double();
                rho_ = this->get_parameter("rho").as_double();
                delta_ = this->get_parameter("delta").as_double();
                RCLCPP_INFO(this->get_logger(), "Begin merging of the input paths");

                // initializing weights and parameter
                alpha_ = 1.0;
                gamma_ = 0.0;

                // copy all the points on path1 until the transition index (included)
                for(int i = 0; i <= idx1; i++){
                    mergedPath_.points.push_back(path1_.points[i]);
                }
                
                // initialize prevTheta
                float thetaC1 = atan2(path1_.points[idx1].y - path1_.points[idx1-1].y, path1_.points[idx1].x - path1_.points[idx1-1].x);
                float thetaC2 = atan2(path2_.points[0].y - path1_.points[idx1-1].y, path2_.points[0].x - path1_.points[idx1-1].x);
                prevTheta_ = thetaC2 - thetaC1;

                // reset path indices
                idx1++;         // merging from the next index of path1's transition index 
                int idx2 = 1;   // begin path2 from second point such that component2 can be computed at the first loop iteration
                
                // iterate over the remaining path1 points
                for(int i = idx1; i < (int)path1_.points.size(); i++){
                    // calculate input path components
                    Point component1, component2;
                    component1.x = path1_.points[idx1].x - path1_.points[idx1-1].x;
                    component1.y = path1_.points[idx1].y - path1_.points[idx1-1].y;
                    component2.x = path2_.points[idx2].x - path2_.points[idx2-1].x;
                    component2.y = path2_.points[idx2].y - path2_.points[idx2-1].y;
                    float thetaC2 = atan2(component2.y, component2.x);
                    
                    // calculate inertia component
                    Point curr = mergedPath_.points.back();
                    Point prev = mergedPath_.points[(int)mergedPath_.points.size()-2];
                    Point inertia;
                    inertia.x = curr.x - prev.x;
                    inertia.y = curr.y - prev.y;

                    // calculate difference compoment
                    Point component3;
                    component3.x = path2_.points[idx2].x - curr.x;
                    component3.y = path2_.points[idx2].y - curr.y;

                    // assign the weights if non-zero
                    if(alpha_ > 0.0){
                        alpha_ = alpha_ - beta_ - gamma_;
                        // making sure alpha is within [0, 1]
                        if(alpha_ > 1.0){
                            alpha_ = 1.0;
                        }
                        else if(alpha_ < 0.0){
                            alpha_ = 0.0;
                        }
                    }

                    // compute the resultant with normalized weights
                    Point resultant;
                    resultant.x = (component1.x*alpha_ + component2.x*(1.0 - alpha_) + component3.x*delta_ + inertia.x*rho_) / (1 + delta_ + rho_);     // normalized using (1+delta+rho)
                    resultant.y = (component1.y*alpha_ + component2.y*(1.0 - alpha_) + component3.y*delta_ + inertia.y*rho_) / (1 + delta_ + rho_);
                    float thetaR = atan2(resultant.y, resultant.x);

                    // add the resultant to the merged_path
                    resultant.x += curr.x;
                    resultant.y += curr.y;
                    mergedPath_.points.push_back(resultant);
                    
                    // adjust the parameters
                    currTheta_ = thetaC2 - thetaR;
                    float dTheta = currTheta_ - prevTheta_;
                    int dirFactor = (prevTheta_ < 0.0 || currTheta_ < 0.0) ? -1 : 1;    // inverts dTheta when Theta is -ve
                    gamma_ = kTheta_ * dTheta * dirFactor;
                    
                    // update variables for next iteration
                    prevTheta_ = currTheta_;
                    idx1++;
                    idx2++;
                }

                // iterate over the rest of the path2
                for(int i = idx2; i < (int)path2_.points.size(); i++){
                    // calculate input path component
                    Point component2;
                    component2.x = path2_.points[idx2].x - path2_.points[idx2-1].x;
                    component2.y = path2_.points[idx2].y - path2_.points[idx2-1].y;

                    // calculate inertia component
                    Point curr = mergedPath_.points.back();
                    Point prev = mergedPath_.points[(int)mergedPath_.points.size()-2];
                    Point inertia;
                    inertia.x = curr.x - prev.x;
                    inertia.y = curr.y - prev.y;

                    // calculate difference compoment
                    Point component3;
                    component3.x = path2_.points[idx2].x - curr.x;
                    component3.y = path2_.points[idx2].y - curr.y;

                    // compute the resultant with normalized weights
                    Point resultant;
                    resultant.x = (component2.x + component3.x*delta_ + inertia.x*rho_) / (1 + delta_ + rho_);  // normalized using (1+delta+rho)
                    resultant.y = (component2.y + component3.y*delta_ + inertia.y*rho_) / (1 + delta_ + rho_);

                    // add the resultant to the merged_path
                    resultant.x += curr.x;
                    resultant.y += curr.y;
                    mergedPath_.points.push_back(resultant);

                    // update variables for next iteration
                    idx2++;
                }
            }
            // publish the merged path
            publishPath();
            
            // clear old merged path
            mergedPath_.points.clear();
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Validity test failed, ignoring subscribed input paths");
        }

        // resetting flags to receive new input paths
        receivedPath1_ = false;
        receivedPath2_ = false;

        RCLCPP_INFO(this->get_logger(), "Ready to receive new input paths for merging...");
    }
    else{
        RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for the input paths to be subscribed...");
    }
}
