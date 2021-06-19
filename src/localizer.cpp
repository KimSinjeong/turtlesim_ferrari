/*
 * scan_matching_localizer_node.cpp
 *
 *  Created on: Apr 30, 2021
 *      Author: Daegyu Lee
 */

// NOTE
// 1. Please submit a report discussing the result of registration algorithm,
// 2. Please feel the code where I wrote as "TODO:#".
// 3. Import 2-D occupancy grid map using map_server(please refer to launch file.) 
// 4. You should give a initial pose using RVIZ 2D Pose Estimate tool.
// 5. If you have query things, feel free to send me a mail : "lee.dk@kaist.ac.kr"
// 6. Run a bag file and compare the result.


// headers in ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
#include <stdlib.h>

// PCL Libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pclomp/ndt_omp.h>

class ScanLocalizer
{
    public:
        ScanLocalizer(ros::NodeHandle& nh);        
        ~ScanLocalizer();
        void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void OccupancyGrid2DMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
        void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void NDTMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData);
        
        ros::NodeHandle nh_;
        ros::Subscriber subScan;
        ros::Subscriber subOccupancyGrid;
        ros::Subscriber subInitPose;

        ros::Publisher pubTransformedCloud;
        ros::Publisher pubOdometry;


        nav_msgs::Odometry m_odomScan;
    
        pcl::PointCloud<pcl::PointXYZI>::Ptr m_StaticMap_ptr;
        bool bRvizInit;
        Eigen::Matrix4f prev_guess, init_guess;
};

ScanLocalizer::ScanLocalizer(ros::NodeHandle& nh) : nh_(nh), bRvizInit(false)
{
    subScan = nh_.subscribe("/scan",1, &ScanLocalizer::LaserScanCallback, this);
    subOccupancyGrid = nh_.subscribe("/map",1, &ScanLocalizer::OccupancyGrid2DMapCallback, this);
    subInitPose = nh.subscribe("/initialpose", 1, &ScanLocalizer::InitPoseCallback, this);

    pubTransformedCloud = nh_.advertise<sensor_msgs::PointCloud2>("/registered_points", 1, true);
    pubOdometry = nh_.advertise<nav_msgs::Odometry>("/odom", 1, true);

    prev_guess = Eigen::Matrix4f::Identity();
    init_guess = Eigen::Matrix4f::Identity();
};

ScanLocalizer::~ScanLocalizer() 
{    
    ROS_INFO("ScanLocalizer destructor.");
}

void ScanLocalizer::InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(msg.pose.pose.orientation.w, 
                                              msg.pose.pose.orientation.x, 
                                              msg.pose.pose.orientation.y, 
                                              msg.pose.pose.orientation.z).toRotationMatrix();
    prev_guess.block(0,0,3,3) = mat3;
    prev_guess(0,3) = msg.pose.pose.position.x;
    prev_guess(1,3) = msg.pose.pose.position.y;
    bRvizInit = true;

    // Initialize parameter to proceed
    if (nh_.hasParam("/initialized")) {
        nh_.setParam("/initialized", true);
    }

    std::cout << prev_guess << std::endl;
}

void ScanLocalizer::LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    if(msg->ranges.empty())
    {
        ROS_ERROR("Empty scan data");
        return;
    }

    // Feel the code to convert the laser scan data to the pointcloud type(pcl::PointCloud<pcl::PointXYZI>)
    for(int i = 0; i < msg->ranges.size(); i++)
    {
        pcl::PointXYZI pointBuf;
        //TODO:#1
        // feel the code here
        if (msg->range_min < msg->ranges[i] && msg->ranges[i] < msg->range_max) {
            float angle_temp = msg->angle_min + i*msg->angle_increment;

            pointBuf.x = msg->ranges[i]*cos(angle_temp);
            pointBuf.y = msg->ranges[i]*sin(angle_temp);
            pointBuf.z = 0;
            pointBuf.intensity = 1.f;
            
            cloud_in_ptr->points.push_back(pointBuf);
        }
    }


    /*Scan matching algorithm*/
    if(cloud_in_ptr->points.empty() || m_StaticMap_ptr->points.empty())
        return;
    

    // Implement below two algorithm and compare the result.
    // You can switch the registration algorithm between ICP and NDT_OMP
    // 2. NDT_OMP(multi-thread)
    NDTMatching(cloud_in_ptr, m_StaticMap_ptr);
}

void ScanLocalizer::OccupancyGrid2DMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    // Feel the code to convert the laser scan data to the pointcloud type(pcl::PointCloud<pcl::PointXYZI>)
    Eigen::Matrix3f rot = Eigen::Quaternionf(msg->info.origin.orientation.w, 
                                              msg->info.origin.orientation.x, 
                                              msg->info.origin.orientation.y, 
                                              msg->info.origin.orientation.z).toRotationMatrix();
    
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            tf(i, j) = rot(i, j);
    
    tf(0, 3) = msg->info.origin.position.x;
    tf(1, 3) = msg->info.origin.position.y;

    m_StaticMap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    for (unsigned int width = 0; width < msg->info.width; width++) {
        for (unsigned int height = 0; height < msg->info.height; height++) {
            //TODO:#3
            // feel the code here.
            //Convert occupied grid to the x-y coordinates to put the target(pcl::PointCloud<pcl::PointXYZI>)
            if (msg->data[width + height*msg->info.width] > 80) {
                pcl::PointXYZI pointBuf;
                pointBuf.x = width*msg->info.resolution + msg->info.resolution/2;
                pointBuf.y = height*msg->info.resolution + msg->info.resolution/2;
                pointBuf.z = 0;
                pointBuf.intensity = 1.f;
                m_StaticMap_ptr->push_back(pointBuf);
            }
        }
    }
    pcl::transformPointCloud (*m_StaticMap_ptr, *m_StaticMap_ptr, tf);

    ROS_INFO("MAP IS LOADED");  
}

void ScanLocalizer::NDTMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData)
{
    if (!bRvizInit)
    {
        init_guess = Eigen::Matrix4f::Identity();
    }
    else
    {
        init_guess = prev_guess;
    }

    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setInputSource(InputData);
    ndt->setInputTarget(TargetData);    
    ndt->setTransformationEpsilon(0.01); 
    ndt->setMaximumIterations(32); 
    ndt->setResolution(1.0);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setNumThreads(5);

    //TODO:#7
    //Feel the code here
    // 1. Run registration algorithm(align)
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
    ndt->align(*result, init_guess);

    // 2. Evaluate the registration algorithm using threshold(score)
    if (ndt->hasConverged())
    {
        std::cout << "converged." << std::endl
                << "The score is " << ndt->getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << ndt->getFinalTransformation() << std::endl;
    }
    else 
    {   
        return;
    }

    // 3. Convert a Final transformation matrix to the init_guess in order to run a align function.
    init_guess.block<3, 3>(0, 0) = ndt->getFinalTransformation().block<3, 3>(0, 0);
    init_guess.block<3, 1>(0, 3) = ndt->getFinalTransformation().block<3, 1>(0, 3);
    
    // 4. Publish registered cloud
    //Convert transformed(registered) pointcloud using NDT algorithm.
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    //Convert pcl pointcloud to the ros msg.  
    sensor_msgs::PointCloud2 PointCloud2Msg;
    pcl::toROSMsg(*result, PointCloud2Msg);
    PointCloud2Msg.header.frame_id = "map";
    pubTransformedCloud.publish(PointCloud2Msg); // topic name: "/registered_points"

    // 5. Publish Odometry
    m_odomScan.header.frame_id = "map";
    //ROS_INFO("stamp time: %f\n", ros::Time::now().toSec());
    m_odomScan.header.stamp = ros::Time::now();
    m_odomScan.pose.pose.position.x = init_guess(0, 3);
    m_odomScan.pose.pose.position.y = init_guess(1, 3);
    m_odomScan.pose.pose.position.z = init_guess(2, 3);

    Eigen::Quaternionf qrot(init_guess.block<3, 3>(0, 0));
    m_odomScan.pose.pose.orientation.x = qrot.x();
    m_odomScan.pose.pose.orientation.y = qrot.y();
    m_odomScan.pose.pose.orientation.z = qrot.z();
    m_odomScan.pose.pose.orientation.w = qrot.w();

    pubOdometry.publish(m_odomScan); //topic name: "/odom"

    prev_guess = init_guess;
}

int main(int argc, char** argv)
{    
    // node name initialization
    ros::init(argc, argv, "scan_matcher");
    sleep(10);

    ros::NodeHandle nh;
    ScanLocalizer localizer(nh);

    ros::spin();

    return 0;
}

