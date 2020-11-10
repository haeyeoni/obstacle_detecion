
// This is created by Haeyeon Gim 2020.11.3 ...
//
// 1. SUBSCRIBE POINTCLOUD TOPIC (CREATED BY DEPTH IMAGE) <- published by depth_image_proc (http://wiki.ros.org/depth_image_proc)
// 2. SUBSCRIBE ODOMETRY TOPIC
// //3. OPTIMIZATION INITIALIZ WITH ESTIMATED ODOMETRY
// 4. RECONSTRUCT THE DEPTH POINT

#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/PoseStamped.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

float MINIMUM_RANGE = 0.1;
float MAXIMUM_RANGE = 5.0;
float VOXEL_SIZE = 0.05;

ros::Publisher pubObstacleCloud;
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
        subPose = nh.subscribe<geometry_msgs::PoseStamped>("/slam_out_pose", 100, &ObstacleDetection::poseHandler,this); 
        subdepthCloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 100, &ObstacleDetection::pointCloudHandler,this); 
        
        pubObstacleCloud = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 100);
    };

    void poseHandler(const geometry_msgs::PoseStamped::ConstPtr &poseMsg) {
        std::cout << "****pose handler called" << std::endl;
        read_pose = true;
        poseMutex.lock();
        estimatedPose.pose.position.x = poseMsg->pose.position.x;
        estimatedPose.pose.position.y = poseMsg->pose.position.y;
        estimatedPose.pose.position.z = 0;
        estimatedPose.pose.orientation.x = 0;
        estimatedPose.pose.orientation.y = 0;
        estimatedPose.pose.orientation.z = poseMsg->pose.orientation.z;
        estimatedPose.pose.orientation.w = poseMsg->pose.orientation.w;
        poseMutex.unlock();
    }

    void pointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &depthCloudMsg) {
        std::cout << "pointcloud handler called" << std::endl;
            
        // change data format
        PointCloudPtr depthCloudIn (new PointCloud);
        PointCloudPtr depthCloudVoxelized (new PointCloud);
        PointCloudPtr depthCloudInMapCoord (new PointCloud);
        PointCloudPtr depthCloudInBaseCoord (new PointCloud);

        pcl::fromROSMsg(*depthCloudMsg, *depthCloudIn);
        std::vector<int> indices;
        
        
        if (!depthCloudIn->empty()){   
            pcl::removeNaNFromPointCloud(*depthCloudIn, *depthCloudIn, indices);
            this->removeClosedPointCloud(*depthCloudIn, *depthCloudIn, MINIMUM_RANGE, MAXIMUM_RANGE); // -> remove outboundary depth
            std::cout << "before voxelized cloud size:" <<depthCloudIn->size()<< std::endl;
            
            this->voxelize(depthCloudIn, depthCloudVoxelized, VOXEL_SIZE);
            std::cout << "voxelized cloud size:" <<depthCloudVoxelized->size()<< std::endl;
            
            this->transformToMap(*depthCloudVoxelized, *depthCloudInMapCoord); // initialized according to the odometry
            std::cout << "voxelized cloud size:" <<depthCloudVoxelized->size()<< std::endl;
            
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
                // TODO: OPTIMIZATION
                obstacleMutex.lock();
                for (int i=0; i<depthCloudInMapCoord->size(); i++) {
                    pointCloudStack->push_back((*depthCloudInMapCoord)[i]);        
                }
                obstacleMutex.unlock();
                prevPclMutex.lock();
                prevPointCloud = depthCloudInMapCoord;        
                prevPclMutex.unlock();
            }

            if (read_pose) 
            {
                sensor_msgs::PointCloud2 obstacle_cloud;
                pcl::toROSMsg(*pointCloudStack, obstacle_cloud);
                obstacle_cloud.header.frame_id = "/map";
                pubObstacleCloud.publish(obstacle_cloud);
            }
        }
    }

    void voxelize(const PointCloudPtr cloud_in, PointCloudPtr cloud_out, float voxel_size)
    {
        static pcl::VoxelGrid<PointCloud2> voxel_filter;

        PointCloud2Ptr cloud (new PointCloud2());
        PointCloud2Ptr cloud_filtered (new PointCloud2());
        
        pcl::toPCLPointCloud2(*cloud_in, *cloud);
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.filter(*cloud_filtered);
        pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_out);
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
            cloud_out.push_back(po);
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
    // read argument

    if (argv[1]) {
        MAXIMUM_RANGE = atof(argv[1]);
    }
    if (argv[2]) {
        MINIMUM_RANGE = atof(argv[2]);
    }
    if (argv[3]) {
        VOXEL_SIZE = atof(argv[3]);
    }
    ObstacleDetection ObstacleDetection;
    
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}