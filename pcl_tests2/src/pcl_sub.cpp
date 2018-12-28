#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher op_pub;

void callback(const PointCloud::ConstPtr& msg)
{
  
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (msg);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);
  
  printf ("Original Cloud: width = %d, height = %d\n", msg->width, msg->height);
  printf ("Filtered Cloud: width = %d, height = %d\n", cloud_filtered->width, cloud_filtered->height);
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  
  //publish the filtered pointcloud to a new topic
  op_pub.publish(cloud_filtered);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_node_pcl");
  ros::NodeHandle nh;
  
  //Subscribe to the default topic that converts pcd file to point cloud. In the callback, perform downsampling
  ros::Subscriber sub = nh.subscribe<PointCloud>("cloud_pcd", 1, callback);
  
  //publish the output topic with filtered pointcloud
  op_pub = nh.advertise<sensor_msgs::PointCloud2> ("op_cloud", 1);
  ros::spin();
}
