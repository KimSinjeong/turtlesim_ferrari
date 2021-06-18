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
#include <nav_msgs/Path.h>
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

bool write = false;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

class TrackExtractor
{
    public:
        TrackExtractor(ros::NodeHandle& nh);        
        ~TrackExtractor();
        void PathRead(const nav_msgs::Path& msg);
        
        ros::NodeHandle nh_;
        ros::Subscriber subPath;
};

TrackExtractor::TrackExtractor(ros::NodeHandle& nh) : nh_(nh)
{
    subPath = nh_.subscribe("/trajectory",1, &TrackExtractor::PathRead, this);
};

TrackExtractor::~TrackExtractor() 
{    
    ROS_INFO("TrackExtractor destructor.");
}

void TrackExtractor::PathRead(const nav_msgs::Path& msg)
{
    if (!write)
        return;

    std::ofstream writeFile("path.csv");
    geoemtry_msgs::Pose pose;
    double x, y;

    if (writeFile.is_open()) {
        for (int i = 0; i < msg.poses.size(); i++) {
            pose = msg.poses.at(i);
            x = pose.position.x;
            y = pose.position.y;
            
            writeFile << x << "," << y << std::endl;
        }
        openFile.close();
    }
    write = false;
}

int main(int argc, char** argv)
{    
    // node name initialization
    ros::init(argc, argv, "track_extractor");

    ros::NodeHandle nh;
    TrackExtractor localizer(nh);

    while (ros::ok())
    {
        int c = getch();   // call your non-blocking input function
        write = true;
    }

    return 0;
}

