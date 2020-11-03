
// This is created by Haeyeon Gim 2020.11.3 ...
//
// 1. SUBSCRIBE POINTCLOUD TOPIC (CREATED BY DEPTH IMAGE) <- published by depth_image_proc (http://wiki.ros.org/depth_image_proc)
// 2. SUBSCRIBE ODOMETRY TOPIC
// 3. OPTIMIZATION INITIALIZ WITH ESTIMATED ODOMETRY
// 4. RECONSTRUCT THE DEPTH POINT


#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

double MINIMUM_RANGE = 0.1;
double MAXIMUM_RANGE = 5.0;

ros::Publisher pubObstacleCloud;
tf::StampedTransform transform;
        
geometry_msgs::PoseWithCovarianceStamped estimatedPose;
PointCloud::Ptr pointCloudStack (new PointCloud); 
PointCloud::Ptr prevPointCloud (new PointCloud);
std::mutex prevPclMutex;
std::mutex poseMutex;
std::mutex obstacleMutex;

bool sysInit = false;

void removeClosedPointCloud(const PointCloud &cloud_in,
                              PointCloud &cloud_out, float min_thres, float max_thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x 
                + cloud_in.points[i].y * cloud_in.points[i].y 
                + cloud_in.points[i].z * cloud_in.points[i].z > min_thres * min_thres &&
                cloud_in.points[i].x * cloud_in.points[i].x 
                + cloud_in.points[i].y * cloud_in.points[i].y 
                + cloud_in.points[i].z * cloud_in.points[i].z < max_thres * max_thres)
            {
                cloud_out.points[j] = cloud_in.points[i];
                j++;
            }
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void transformToMap(const PointCloud &cloud_in, PointCloud &cloud_out) 
{
    std::cout << "transform map" << std::endl;
        
    poseMutex.lock();
    geometry_msgs::Point pos = estimatedPose.pose.pose.position;
    geometry_msgs::Quaternion ori = estimatedPose.pose.pose.orientation;
     
    Eigen::Quaterniond q_w_curr(ori.x, ori.y, ori.z, ori.w);
    Eigen::Vector3d t_w_curr(pos.x, pos.y, pos.z);
    poseMutex.unlock();
	pcl::PointXYZ pi, po;
    cloud_out.clear();

    for (int i=0; i<cloud_in.size(); i++) {
        pi = cloud_in[i];
        Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        
        po.x = point_w.x();
        po.y = point_w.y();
        po.z = point_w.z();
        //po.intensity = pi.intensity;
        cloud_out.push_back(po);
    }    
}
void emptyCache() {
    std::cout << "cache empty" << std::endl;
        
    sysInit = false;
    prevPointCloud->clear();
}
void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &depthCloudMsg) {
    std::cout << "pointcloud handler called" << std::endl;
        
    // change data format
    PointCloudPtr depthCloudIn (new PointCloud);
    PointCloudPtr depthCloudInMapCoord (new PointCloud);
    pcl::fromROSMsg(*depthCloudMsg, *depthCloudIn);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(*depthCloudIn, *depthCloudIn, indices);
    removeClosedPointCloud(*depthCloudIn, *depthCloudIn, MINIMUM_RANGE, MAXIMUM_RANGE); // -> remove outboundary depth
    std::cout << "number of depthcloud point: " <<depthCloudIn->size()<< std::endl;
        
    // if(!depthCloudIn->empty())
    //     {
    //         emptyCache();
    //         return;
    //     }
    transformToMap(*depthCloudIn, *depthCloudInMapCoord); // initialized according to the odometry
    std::cout << "number of depthcloud point after transform: " << depthCloudIn->size()<< std::endl;
        
    if (!sysInit)
    {
        prevPointCloud = depthCloudInMapCoord;  
        for (int i=0; i<depthCloudInMapCoord->size(); i++) {
            pointCloudStack->push_back((*depthCloudInMapCoord)[i]);        
        }      
        sysInit = true;
        return;
    }

    else {
        // OPTIMIZATION //
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(depthCloudInMapCoord);        
        icp.setInputTarget(prevPointCloud);
        icp.align(*depthCloudInMapCoord);
        
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
  
        if (icp.hasConverged()) {
            
            obstacleMutex.lock();
            for (int i=0; i<depthCloudInMapCoord->size(); i++) {
                pointCloudStack->push_back((*depthCloudInMapCoord)[i]);        
            }
            obstacleMutex.unlock();

            prevPclMutex.lock();
            prevPointCloud = depthCloudInMapCoord;        
            prevPclMutex.unlock();
            
        }
    }
    std::cout << "point cloud stack point number: "<<pointCloudStack->size() << std::endl;
    
    sensor_msgs::PointCloud2 obstacle_cloud;
    pcl::toROSMsg(*pointCloudStack, obstacle_cloud);
    obstacle_cloud.header.frame_id = "/world";
    pubObstacleCloud.publish(obstacle_cloud);
}


void poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr &estimatedPoseMsg) {
    poseMutex.lock();
    estimatedPose.header = estimatedPoseMsg->header;
    estimatedPose.pose.covariance = estimatedPoseMsg->pose.covariance;
    estimatedPose.pose.pose.position.x = estimatedPoseMsg->pose.pose.position.x;
    estimatedPose.pose.pose.position.y = estimatedPoseMsg->pose.pose.position.y;
    estimatedPose.pose.pose.position.z = estimatedPoseMsg->pose.pose.position.z;
    estimatedPose.pose.pose.orientation.x = estimatedPoseMsg->pose.pose.orientation.x;
    estimatedPose.pose.pose.orientation.y = estimatedPoseMsg->pose.pose.orientation.y;
    estimatedPose.pose.pose.orientation.z = estimatedPoseMsg->pose.pose.orientation.z;
    estimatedPose.pose.pose.orientation.w = estimatedPoseMsg->pose.pose.orientation.w;
    poseMutex.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh;
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.0);
    nh.param<double>("maximum_range", MAXIMUM_RANGE, 100.0);
    tf::TransformListener tfListener;
    ros::Subscriber subdepthCloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 100, pointCloudHandler); 
    pubObstacleCloud = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_cloud", 100);

    while(ros::ok()) {
        try{
            listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
        prevPclMutex.unlock();
        poseMutex.unlock();
        obstacleMutex.unlock();
    }
    return 0;
}