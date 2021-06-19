/*
 * scan_matching_localizer_node.cpp
 *
 *  Created on: Apr 30, 2021
 *      Author: Daegyu Lee
 */

// headers in ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>
#include <float.h>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <bits/stdc++.h>
#include <mutex>
#include <thread>

#include <fstream>

class TrackExtractor
{
    public:
        TrackExtractor(ros::NodeHandle& nh);        
        ~TrackExtractor();
        void PathRead(const geometry_msgs::PoseStamped&);
        
        ros::NodeHandle nh_;
        ros::Subscriber subPath;
        std::ofstream writeFile;
};

TrackExtractor::TrackExtractor(ros::NodeHandle& nh) : nh_(nh)
{
    subPath = nh_.subscribe("slam_out_pose",100, &TrackExtractor::PathRead, this);
    writeFile = std::ofstream("/home/group6/catkin_ws/src/turtlesim_ferrari/path.csv");
};

TrackExtractor::~TrackExtractor() 
{    
    ROS_INFO("TrackExtractor destructor.");
    writeFile.close();
}

void TrackExtractor::PathRead(const geometry_msgs::PoseStamped& msg)
{
    if (writeFile.is_open()) {
	    std::cout << "Writting...." << std::endl;
        double x, y;

        x = msg.pose.position.x;
        y = msg.pose.position.y;
        
        writeFile << x << "," << y << std::endl;   
    }
}

int main(int argc, char** argv)
{    
    // node name initialization
    ros::init(argc, argv, "track_extractor");

    ros::NodeHandle nh;
    TrackExtractor localizer(nh);
    std::cout << "Start Logging Path" << std::endl;

    ros::spin();
    return 0;
}

