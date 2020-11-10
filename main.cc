
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
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <nav_msgs/Path.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

float MINIMUM_RANGE = 0.1;
float MAXIMUM_RANGE = 5.0;

ros::Publisher pubObstacleCloud;
tf::StampedTransform transform;
bool read_pose = false;
geometry_msgs::PoseStamped estimatedPose;
PointCloud::Ptr pointCloudStack (new PointCloud); 
PointCloud::Ptr prevPointCloud (new PointCloud);
std::mutex obstacleMutex;
std::mutex prevPclMutex;
std::mutex poseMutex;
bool sysInit = false;

class ObstacleDetection
{
public:
    ObstacleDetection() {

        subPose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("poseupdate", 100, &ObstacleDetection::poseHandler,this); 
        subdepthCloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 100, &ObstacleDetection::pointCloudHandler,this); 
        //subtrajectory = nh.subscribe<nav_msgs::Path>("/trajectory", 100, &ObstacleDetection::trajectoryHandler,this); 
        pubObstacleCloud = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 100);
    };

    void trajectoryHandler(const nav_msgs::Path::ConstPtr &pathMsg) {
        std::cout << "path handler called" << std::endl;
    }

    void poseHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &poseMsg) {
        std::cout << "pose handler called" << std::endl;
        read_pose = true;
        poseMutex.lock();
        //estimatedPose.header.frame_id = poseMsg->header;
        estimatedPose.pose.position.x = poseMsg->pose.pose.position.x;
        estimatedPose.pose.position.y = poseMsg->pose.pose.position.y;
        estimatedPose.pose.position.z = poseMsg->pose.pose.position.z;
        estimatedPose.pose.orientation.x = poseMsg->pose.pose.orientation.x;
        estimatedPose.pose.orientation.y = poseMsg->pose.pose.orientation.y;
        estimatedPose.pose.orientation.z = poseMsg->pose.pose.orientation.z;
        estimatedPose.pose.orientation.w = poseMsg->pose.pose.orientation.w;
        poseMutex.unlock();
    }

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
        std::cout << "transform to map" << std::endl;
        poseMutex.lock();
        geometry_msgs::Point pos = estimatedPose.pose.position;
        geometry_msgs::Quaternion ori = estimatedPose.pose.orientation;
        poseMutex.unlock();

        Eigen::Quaterniond q_w_curr(ori.x, ori.y, ori.z, ori.w);
        Eigen::Vector3d t_w_curr(pos.x, pos.y, pos.z);
        pcl::PointXYZ pi, po;
        cloud_out.clear();
        std::cout<< pos.x <<','<< pos.y <<','<< pos.z <<std::endl;
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
    void transformToBase(const PointCloud &cloud_in, PointCloud &cloud_out) 
    {
        std::cout << "transform to base_link" << std::endl;
        poseMutex.lock();
        geometry_msgs::Point pos = estimatedPose.pose.position;
        geometry_msgs::Quaternion ori = estimatedPose.pose.orientation;
        poseMutex.unlock();

        Eigen::Quaterniond q_w_curr(ori.x, ori.y, ori.z, ori.w);
        Eigen::Vector3d t_w_curr(pos.x, pos.y, pos.z);

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

    void pointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &depthCloudMsg) {
        std::cout << "pointcloud handler called" << std::endl;
            
        // change data format
        PointCloudPtr depthCloudIn (new PointCloud);
        PointCloudPtr depthCloudInMapCoord (new PointCloud);
        PointCloudPtr depthCloudInBaseCoord (new PointCloud);
        pcl::fromROSMsg(*depthCloudMsg, *depthCloudIn);
        std::vector<int> indices;
        
        pcl::removeNaNFromPointCloud(*depthCloudIn, *depthCloudIn, indices);
        this->removeClosedPointCloud(*depthCloudIn, *depthCloudIn, MINIMUM_RANGE, MAXIMUM_RANGE); // -> remove outboundary depth
            

        this->transformToMap(*depthCloudIn, *depthCloudInMapCoord); // initialized according to the odometry
        if (!depthCloudInMapCoord->empty()){   
            if (!sysInit)
            {
                prevPointCloud = depthCloudInMapCoord;  
                this->transformToBase(*depthCloudInMapCoord, *depthCloudInBaseCoord);    
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
        
                //if (icp.hasConverged()) {
                    
                    obstacleMutex.lock();
                    this->transformToBase(*depthCloudInMapCoord, *depthCloudInBaseCoord); 
                    for (int i=0; i<depthCloudInMapCoord->size(); i++) {
                        pointCloudStack->push_back((*depthCloudInBaseCoord)[i]);        
                    }
                    obstacleMutex.unlock();
                    prevPclMutex.lock();
                    prevPointCloud = depthCloudInMapCoord;        
                    prevPclMutex.unlock();
                    
                //} 
                // else 
                // {
                //     emptyCache();
                //     return;
                // }
            }
        }
        if (read_pose) 
        {
            sensor_msgs::PointCloud2 obstacle_cloud;
            pcl::toROSMsg(*pointCloudStack, obstacle_cloud);
            obstacle_cloud.header.frame_id = "/base_link";
            pubObstacleCloud.publish(obstacle_cloud);
        }
    }
    ~ObstacleDetection(){
    }
    private:
        ros::NodeHandle nh;
        ros::Subscriber subPose;
        ros::Subscriber subdepthCloud;
        ros::Subscriber subtrajectory;
        ros::Publisher pubObstacleCloud;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh;
    if (argv[1]) {
        MAXIMUM_RANGE = atof(argv[1]);
    }
    if (argv[2]) {
        MINIMUM_RANGE = atof(argv[2]);
    }
    ObstacleDetection ObstacleDetection;
    
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}