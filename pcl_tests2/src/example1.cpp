#include <ros/ros.h>

// PCL specific includes
//#include <sensor_msgs/PointCloud2.h> //pcl_ros handles direct conversion between the PointCloud<T> object to sensor_msgs/PointClud2 conversion

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//pcl:PointXYZ is a struct reference : http://docs.pointclouds.org/1.5.1/structpcl_1_1_point_x_y_z.html
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1); //create a publisher

  //attributes of the PointCloud object : http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html
  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
