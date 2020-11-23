
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <message_filters/sync_policies/approximate_time.h>

// image
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// open3d
#include <open3d/geometry/Image.h>
#include <open3d/geometry/Geometry2D.h>
#include <open3d/geometry/Image.h>
using namespace message_filters;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

void poseHandler(const geometry_msgs::PoseStamped::ConstPtr &poseMsg);
void pointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &depthCloudMsg);

void voxelize(const PointCloudPtr cloud_in, PointCloudPtr cloud_out, float voxel_size);

void removeClosedPointCloud(const PointCloud &cloud_in,
                            PointCloud &cloud_out, float min_thres, float max_thres);

void transformToMap(const PointCloud &cloud_in, PointCloud &cloud_out);
void imageHandler(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& depth);