
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
#include <pcl/registration/icp.h>
#include <pcl/filters/conditional_removal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

	
// variables
bool read_pose = false;
bool sysInit = false;
geometry_msgs::PoseWithCovarianceStamped estimatedPose;
PointCloudPtr pointCloudStack (new PointCloud); 
PointCloudPtr prevPointCloud (new PointCloud);
std::mutex obstacleMutex;
std::mutex prevPclMutex;
std::mutex poseMutex;
std::mutex tfMutex;
tf::StampedTransform tf_base_to_map;

class ObstacleDetection
{
public:
    ObstacleDetection(ros::NodeHandle nh, ros::NodeHandle local_nh) {
		local_nh.getParam("minimum_range", MINIMUM_RANGE);
		local_nh.getParam("maximum_range", MAXIMUM_RANGE);
		local_nh.getParam("minimum_height", MINIMUM_HEIGHT);
		local_nh.getParam("maximum_height", MAXIMUM_HEIGHT);
				
		local_nh.getParam("voxel_size", VOXEL_SIZE);
		local_nh.getParam("max_iteration", MAX_ITERATION);
	
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
        PointCloudPtr depthCloudInMapCoord (new PointCloud);
        PointCloudPtr depthCloudVoxelized (new PointCloud);
        
        pcl::fromROSMsg(*depthCloudMsg, *depthCloudIn);
        std::vector<int> indices;
        
        
        if (!depthCloudIn->empty()){   
            pcl::removeNaNFromPointCloud(*depthCloudIn, *depthCloudIn, indices);
            this->removeClosedPointCloud(depthCloudIn, depthCloudIn); // -> remove outboundary depth         
            this->voxelize(depthCloudIn, depthCloudVoxelized, VOXEL_SIZE);
		    this->transformToMap(*depthCloudVoxelized, *depthCloudInMapCoord); // initialized according to the odometry
            if (!depthCloudInMapCoord->empty()){
		        if (!sysInit)
		        {					
					(*pointCloudStack).clear(); //added
					prevPointCloud = depthCloudInMapCoord;  
					for (int i=0; i<depthCloudInMapCoord->size(); i++) {
					    pointCloudStack->push_back((*depthCloudInMapCoord)[i]);        
					}
					sysInit = true;
			        return;
		        }
		        else {

					pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
					icp.setInputSource(depthCloudInMapCoord);
					icp.setInputTarget(prevPointCloud);
					icp.setMaximumIterations(MAX_ITERATION);
					icp.align(*depthCloudInMapCoord);
		        
		            obstacleMutex.lock();
					(*pointCloudStack).clear(); //added
					for (int i=0; i<depthCloudInMapCoord->size(); i++) {
			            pointCloudStack->push_back((*depthCloudInMapCoord)[i]);        
			        }			
		            obstacleMutex.unlock();
		            prevPclMutex.lock();
		            prevPointCloud = depthCloudInMapCoord;        
		            prevPclMutex.unlock();
		        }
			} else { sysInit = false;}

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

    void removeClosedPointCloud(const PointCloudPtr cloud_in, PointCloudPtr cloud_out)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out->header = cloud_in->header;
            cloud_out->points.resize(cloud_in->points.size());
        }

        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
		// set condition
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -MAXIMUM_HEIGHT)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, -MINIMUM_HEIGHT)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, MINIMUM_RANGE)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, MAXIMUM_RANGE)));
        // conditional removal
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setInputCloud(cloud_in);
		condrem.setCondition(range_condition);
		condrem.setKeepOrganized(true);
		condrem.filter(*cloud_out);

        size_t j = cloud_out->size();
		cloud_out->points.resize(j);

        cloud_out->height = 1;
        cloud_out->width = static_cast<uint32_t>(j);
        cloud_out->is_dense = true;        
    }

    void transformToMap(const PointCloud &cloud_in, PointCloud &cloud_out) 
    {
		// 1. modify axis
		Eigen::Matrix3f rot_x_90;
		Eigen::Matrix3f rot_y_90;
		Eigen::Matrix3f rot_z_90; 
		rot_x_90 << 1,0,0, 0,0,-1, 0,1,0;
		rot_y_90 << 0,0,-1, 0,1,0, 1,0,0;
		rot_z_90 << 0,-1,0, 1,0,0, 0,0,1; 
	
		// 2. move to amcl pose (map -> amcl)
        poseMutex.lock();
        geometry_msgs::Point translation2 = estimatedPose.pose.pose.position;
        geometry_msgs::Quaternion rotation2 = estimatedPose.pose.pose.orientation;
        poseMutex.unlock();
		Eigen::Quaternionf quater2 (rotation2.w, rotation2.x, rotation2.y, rotation2.z);
		Eigen::Matrix3f rot_from_quat2 = quater2.toRotationMatrix();
		
		// calculate total transform and rotation
		Eigen::Matrix4f total_transform = Eigen::Matrix4f::Identity();
		Eigen::Matrix3f total_rotation = rot_z_90 * rot_x_90 * rot_z_90 *rot_z_90 * rot_from_quat2;
		
		total_transform<<total_rotation(0,0), total_rotation(0,1), total_rotation(0,2), translation2.x,
						total_rotation(1,0), total_rotation(1,1), total_rotation(1,2), translation2.y,
						total_rotation(2,0), total_rotation(2,1), total_rotation(2,2), translation2.z,
						0,0,0,1;
		pcl::transformPointCloud(cloud_in, cloud_out, total_transform);			   
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
		float MINIMUM_HEIGHT = -0.5f;
		float MAXIMUM_HEIGHT = 0.5f;
		float VOXEL_SIZE = 0.05f;
		float MAX_ITERATION = 10;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh; 
    ros::NodeHandle local_nh("~");
    ObstacleDetection ObstacleDetection(nh, local_nh);
    
	tf::TransformListener listener;
    while(ros::ok()) {
        ros::spinOnce();
		/*try{
      		listener.lookupTransform("/base_link", "/map", ros::Time(0), tf_base_to_map);
    	}
    	catch (tf::TransformException ex){
      		ROS_ERROR("%s",ex.what());
      		ros::Duration(1.0).sleep();
    	}*/
    }
    return 0;
}
