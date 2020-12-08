
// This is created by Haeyeon Gim 2020.11.3 ...
//
// 1. SUBSCRIBE POINTCLOUD TOPIC (CREATED BY DEPTH IMAGE) <- published by depth_image_proc (http://wiki.ros.org/depth_image_proc)
// 2. SUBSCRIBE ODOMETRY TOPIC
// //3. OPTIMIZATION INITIALIZ WITH ESTIMATED ODOMETRY
// 4. RECONSTRUCT THE DEPTH POINT

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

	
// variables
bool read_pose = false;
bool sysInit = false;
geometry_msgs::PoseWithCovarianceStamped estimatedPose;
PointCloud::Ptr pointCloudStack (new PointCloud); 
PointCloud::Ptr prevPointCloud (new PointCloud);
std::mutex obstacleMutex;
std::mutex prevPclMutex;
std::mutex poseMutex;

class ObstacleDetection
{
public:
    ObstacleDetection(ros::NodeHandle nh, ros::NodeHandle local_nh) {
	local_nh.getParam("minimum_range", MINIMUM_RANGE);
	local_nh.getParam("maximum_range", MAXIMUM_RANGE);
	local_nh.getParam("voxel_size", VOXEL_SIZE);
	
        subPose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 100, &ObstacleDetection::poseHandler,this); 
        subdepthCloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 100, &ObstacleDetection::pointCloudHandler,this);   
        pubObstacleCloud = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 100);
    };

    void poseHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &poseMsg) {
	read_pose = true;
	double qw = poseMsg->pose.pose.orientation.w;
	double qx = poseMsg->pose.pose.orientation.x;
	double qy = poseMsg->pose.pose.orientation.y;
	double qz = poseMsg->pose.pose.orientation.z;
	
	// yaw (z-axis rotation)
	double siny_cosp = 2 * (qw * qz + qx * qy);
	double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
	double yaw = std::atan2(siny_cosp, cosy_cosp);

	poseMutex.lock();
        estimatedPose.pose.pose.position.x = poseMsg->pose.pose.position.x;
        estimatedPose.pose.pose.position.y = poseMsg->pose.pose.position.y;
        estimatedPose.pose.pose.position.z = poseMsg->pose.pose.position.z;
        estimatedPose.pose.pose.orientation.x = poseMsg->pose.pose.orientation.x; //0;
        estimatedPose.pose.pose.orientation.y = poseMsg->pose.pose.orientation.y; //0;
        estimatedPose.pose.pose.orientation.z = poseMsg->pose.pose.orientation.z; //sin(yaw * 0.5);
        estimatedPose.pose.pose.orientation.w = poseMsg->pose.pose.orientation.w; //cos(yaw * 0.5);

        poseMutex.unlock();
    }

    void pointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &depthCloudMsg) {
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
            
            this->voxelize(depthCloudIn, depthCloudVoxelized, VOXEL_SIZE);
            
            this->transformToMap(*depthCloudVoxelized, *depthCloudInMapCoord); // initialized according to the odometry
            
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
                obstacle_cloud.header.frame_id = "map";
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
        poseMutex.lock();
        geometry_msgs::Point pos = estimatedPose.pose.pose.position;
        geometry_msgs::Quaternion ori = estimatedPose.pose.pose.orientation;
        poseMutex.unlock();
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	Eigen::Quaternionf q_w_curr (ori.w, ori.x, ori.y, ori.z);
	Eigen::Matrix3f R(q_w_curr);
	float cosine = R(0,0);
	float sine = R(1,0);

	transform(0,0) = sine; transform(0,1) = 0; transform(0,2)= cosine;
	transform(1,0) = -cosine; transform(1,1) = 0; transform(1,2)= sine;
	transform(2,0) = 0; transform(2,1) = -1; transform(2,2)= 0;
	transform(0,3) = pos.x;
        transform(1,3) = pos.y;
        transform(2,3) = pos.z;
	
	cloud_out.clear();
	pcl::transformPointCloud(cloud_in, cloud_out, transform);
   
    }

    ~ObstacleDetection(){
    }
    private:
        ros::Subscriber subPose;
        ros::Subscriber subdepthCloud;
        ros::Subscriber subtrajectory;
        ros::Publisher pubObstacleCloud;
	
	// Parameters
	float MINIMUM_RANGE = 0.0f;
	float MAXIMUM_RANGE = 2.0f;
	float VOXEL_SIZE = 0.05f;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh; 
    ros::NodeHandle local_nh("~");
    ObstacleDetection ObstacleDetection(nh, local_nh);
    
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
