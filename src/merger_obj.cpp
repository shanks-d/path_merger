#include "merger_obj.h"

// @brief Main class that merges the inputs paths and publishes to a new topic
Merger::Merger()
    : Node("path_merger"){
        // create subscribers for 'path1' and 'path2' topics
        subPath1_ = this->create_subscription<Path>("path1", 10,
        std::bind(&Merger::path1Callback, this, _1));
        subPath2_ = this->create_subscription<Path>("path2", 10,
        std::bind(&Merger::path2Callback, this, _1));

        // creating tunable variables as rosparams
        this->declare_parameter("beta", 0.1);
        this->declare_parameter("k", 0.6);
        this->declare_parameter("rho", 1.0);
        this->declare_parameter("delta", 0.2);

        // create publisher of merged path to 'merged_path' topic
        pubMergedPath_ = this->create_publisher<Path>("merged_path", 10);
        // create a timer event to call for merging of input paths while the node is active
        timer_ = this->create_wall_timer(500ms, std::bind(&Merger::mergePaths, this));
}

// @brief Subscriber callback to receive path1 information
void Merger::path1Callback(const Path::SharedPtr msg){
    // check if a new path scenario is received
    if(!receivedPath1_ && path1_.scenario != msg->scenario){
        // update the path information
        path1_.points = msg->points;
        path1_.header = msg->header;
        path1_.scenario = msg->scenario;
        // set flag as received
        receivedPath1_ = true;
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s", path1_.header.frame_id.c_str());
    }
}

// @brief Subscriber callback to receive path2 information
void Merger::path2Callback(const Path::SharedPtr msg){
    // check if a new path scenario is received
    if(!receivedPath2_ && path2_.scenario != msg->scenario){
        // update the path information
        path2_.points = msg->points;
        path2_.header = msg->header;
        path2_.scenario = msg->scenario;
        // set the flag as received
        receivedPath2_ = true;
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s", path2_.header.frame_id.c_str());
    }
}

// @brief Publishes the computed merged path to 'merged_path' topic
void Merger::publishPath(){
    // add path metadata to the merged path headers
    mergedPath_.header.stamp = this->get_clock()->now();
    mergedPath_.header.frame_id = "mergedPath";
    mergedPath_.scenario = path1_.scenario;     // since scenario is the same for both the paths
    pubMergedPath_->publish(mergedPath_);
    RCLCPP_INFO(this->get_logger(), "Merged path published!");
}

// @brief Returns the euclidean distance between two points
// @param p1 Point 1
// @param p2 Point 2
float Merger::calcDist(const Point& p1, const Point& p2){
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

// @brief Returns the index of path waypoint closest to the target point
// @param path Path to be search for the closest waypoint index to target
// @param point Target point
int Merger::findTransitionIndex(const Path& path, const Point& point){
    int index = 0;
    float minDist = std::numeric_limits<float>::max();
    int length = path.points.size();
    // iterate over all the points of the path
    for(int p = 0; p < length; p++){
        // compute the distance to the target point
        float dist = calcDist(path.points[p], point);
        // update the minimum index
        if(dist < minDist){
            index = p;
            minDist = dist;
        }
    }
    return index;
}

// @brief Returns the result of the validity checks for both the input paths
// @param path1 Input path 1
// @param path2 Input path 2
bool Merger::isInputValid(const Path& path1, const Path& path2){
    bool isValid = true;

    // checking for path lengths
    RCLCPP_INFO(this->get_logger(), "Checking for path length requirements...");
    int length1 = path1.points.size();
    int length2 = path2.points.size();
    // unequal path lengths warrants a warning but we can continue to merge if no other issues are detected
    if(length1 != length2){
        RCLCPP_WARN(this->get_logger(), "Paths are of unequal length, %s: %d, %s: %d", path1.header.frame_id.c_str(), length1, path2.header.frame_id.c_str(), length2);
    }
    // the weights and other parameters of the merging algorithm would have to be re-tuned for shorter paths lengths
    if(length1 < minPathLength_){
        RCLCPP_ERROR(this->get_logger(), "%s has %d waypoints but minimum required is %d", path1.header.frame_id.c_str(), length1, minPathLength_);
        isValid = false;
    }
    if(length2 < minPathLength_){
        RCLCPP_ERROR(this->get_logger(), "%s has %d waypoints but minimum required is %d", path2.header.frame_id.c_str(), length2, minPathLength_);
        isValid = false;
    }

    // checking for consistency and continuity of the paths
    RCLCPP_INFO(this->get_logger(), "Checking for continuity of the input paths...");
    for(int i = 1; i < length1; i++){
        if(calcDist(path1.points[i], path1.points[i-1]) >= distThreshold_){
            RCLCPP_ERROR_ONCE(this->get_logger(), "%s is not continuous", path1.header.frame_id.c_str());
            isValid = false;
            break;
        }
    }
    for(int i = 1; i < length2; i++){
        if(calcDist(path2.points[i], path2.points[i-1]) >= distThreshold_){
            RCLCPP_ERROR_ONCE(this->get_logger(), "%s is not continuous", path2.header.frame_id.c_str());
            isValid = false;
            break;
        }
    }

    // checking for path scenarios
    RCLCPP_INFO(this->get_logger(), "Checking for consistency of the input paths...");
    if(path1.scenario != path2.scenario){
        RCLCPP_ERROR(this->get_logger(), "Scenario names corresponding to the input paths are inconsistent");
        isValid = false;
    } 
    return isValid;
}

// @brief Merges the input paths using weighted vector addition
void Merger::mergePaths(){
    // check if the input paths are subscribed to begin with the merging operation, else wait and do nothing
    if(receivedPath1_ && receivedPath2_){
        // check for path validity before merging
        if(isInputValid(path1_, path2_)){
            RCLCPP_INFO(this->get_logger(), "Input paths are valid!");

            // find the closest point in path1 from the start of path2, which marks as the transition point
            int idx1 = findTransitionIndex(path1_, path2_.points[0]);
            // if the distance at the transition point is greater than the maximum distance then regard path1 as irrelevant
            if(calcDist(path1_.points[idx1], path2_.points[0]) >= distThreshold_){
                RCLCPP_ERROR(this->get_logger(), "Lost track of the old path, considering new path as merged path");
                // in such case, output the new path (path2) as the merged path
                mergedPath_.points = path2_.points;
            }
            // Merging algorithm
            else{
                RCLCPP_INFO(this->get_logger(), "Begin merging of the input paths");
                
                // retrieve the value of the parameters for merging
                beta_ = this->get_parameter("beta").as_double();
                kTheta_ = this->get_parameter("k").as_double();
                rho_ = this->get_parameter("rho").as_double();
                delta_ = this->get_parameter("delta").as_double();

                // initializing weights and parameter
                alpha_ = 1.0;
                gamma_ = 0.0;

                // copy all the points on path1 to the merged path until the transition point index (included)
                for(int i = 0; i <= idx1; i++){
                    mergedPath_.points.push_back(path1_.points[i]);
                }
                
                // reset path indices
                idx1++;         // merging from the next index of path1's transition index 
                int idx2 = 1;   // begin path2 from second point such that component2 can be computed at the first loop iteration
                
                // initialize previous Theta as the angle between the path1 and path2 components
                // setting prevTheta as zero results in elevated derivative part for the first iteration 
                float thetaC1 = atan2(path1_.points[idx1].y - path1_.points[idx1-1].y, path1_.points[idx1].x - path1_.points[idx1-1].x);
                float thetaC2 = atan2(path2_.points[idx2].y - path2_.points[idx2-1].y, path2_.points[idx2].x - path2_.points[idx2-1].x);
                prevTheta_ = thetaC2 - thetaC1;
                
                // iterate over the remaining path1 points while performing the merging operation
                int length1 = path1_.points.size();
                for(int i = idx1; i < length1; i++){
                    // calculate input path components
                    Point component1, component2;
                    component1.x = path1_.points[idx1].x - path1_.points[idx1-1].x;
                    component1.y = path1_.points[idx1].y - path1_.points[idx1-1].y;
                    component2.x = path2_.points[idx2].x - path2_.points[idx2-1].x;
                    component2.y = path2_.points[idx2].y - path2_.points[idx2-1].y;
                    float thetaC2 = atan2(component2.y, component2.x);
                    
                    // calculate inertial component
                    Point curr = mergedPath_.points.back();
                    Point prev = mergedPath_.points[(int)mergedPath_.points.size()-2];
                    Point inertia;
                    inertia.x = curr.x - prev.x;
                    inertia.y = curr.y - prev.y;

                    // calculate differential compoment
                    Point component3;
                    component3.x = path2_.points[idx2].x - curr.x;
                    component3.y = path2_.points[idx2].y - curr.y;

                    // compute new value of the weight alpha only if non-zero, to avoid revival of alpha due to the derivative part
                    if(alpha_ > 0.0){
                        // alpha update step, starting with 1.0 should gradually diminish to zero
                        alpha_ = alpha_ - beta_ - gamma_;
                        // making sure alpha is within [0, 1]
                        if(alpha_ > 1.0){
                            alpha_ = 1.0;
                        }
                        else if(alpha_ < 0.0){
                            alpha_ = 0.0;
                        }
                    }

                    // compute the resultant as weighted vector addition, where the weights are normalized by (1 + delta + rho)
                    Point resultant;
                    resultant.x = (component1.x*alpha_ + component2.x*(1.0 - alpha_) + component3.x*delta_ + inertia.x*rho_) / (1 + delta_ + rho_);
                    resultant.y = (component1.y*alpha_ + component2.y*(1.0 - alpha_) + component3.y*delta_ + inertia.y*rho_) / (1 + delta_ + rho_);
                    float thetaR = atan2(resultant.y, resultant.x);

                    // add the resultant component to the current point and append it to the merged_path
                    resultant.x += curr.x;
                    resultant.y += curr.y;
                    mergedPath_.points.push_back(resultant);
                    
                    // find the change in theta to calculate the derivative part of the alpha update step 
                    currTheta_ = thetaC2 - thetaR;
                    float dTheta = currTheta_ - prevTheta_;
                    // inverts dTheta when Theta is -ve such that dTheta is -ve if converging and +ve if diverging 
                    int dirFactor = (prevTheta_ < 0.0 || currTheta_ < 0.0) ? -1 : 1;
                    // update the derivative part for next iteration
                    gamma_ = kTheta_ * dTheta * dirFactor;
                    
                    // update variables for next iteration
                    prevTheta_ = currTheta_;
                    idx1++;
                    idx2++;
                }

                // iterate over the rest of the path2 to complete the merging operation
                for(int i = idx2; i < (int)path2_.points.size(); i++){
                    // calculate input path component
                    Point component2;
                    component2.x = path2_.points[idx2].x - path2_.points[idx2-1].x;
                    component2.y = path2_.points[idx2].y - path2_.points[idx2-1].y;

                    // calculate inertial component
                    Point curr = mergedPath_.points.back();
                    Point prev = mergedPath_.points[(int)mergedPath_.points.size()-2];
                    Point inertia;
                    inertia.x = curr.x - prev.x;
                    inertia.y = curr.y - prev.y;

                    // calculate differential compoment
                    Point component3;
                    component3.x = path2_.points[idx2].x - curr.x;
                    component3.y = path2_.points[idx2].y - curr.y;

                    // compute the resultant as weighted vector addition, where the weights are normalized by (1 + delta + rho)
                    // notice here alpha is not used since it is the weight of the path1 component and there are no more waypoints of path1 left
                    Point resultant;
                    resultant.x = (component2.x + component3.x*delta_ + inertia.x*rho_) / (1 + delta_ + rho_);
                    resultant.y = (component2.y + component3.y*delta_ + inertia.y*rho_) / (1 + delta_ + rho_);

                    // add the resultant component to the current point and append it to the merged_path
                    resultant.x += curr.x;
                    resultant.y += curr.y;
                    mergedPath_.points.push_back(resultant);

                    // update variables for next iteration
                    idx2++;
                }
            }
            // publish the merged path
            publishPath();
            
            // clear the merged path in preparation for running merging operation again with new input paths
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
