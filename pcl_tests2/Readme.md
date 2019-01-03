## [](#header-1)Sample Pipeline workflow

!["Workflow"](https://github.com/jadhavhninad/RosCodeRepo/blob/master/pcl_tests2/assets/pcl_test2_workflow.png)

### [](#header-3) Running a publisher:
To run a default point cloud publisher, run the following command:
rosrun pcl\_ros pcd\_to\_pointcloud _filename_ _frequency_

Eg:
rosrun pcl_ros pcd_to_pointcloud src/RosCodeRepo/pcl_tests2/src/table_scene_lms400.pcd 0.5
