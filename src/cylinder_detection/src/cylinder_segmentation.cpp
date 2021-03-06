#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include <cstdlib> //For rand() and srand()
#include <color_recognition/Int2dArray.h>
#include <color_recognition/IntList.h>
#include <color_recognition/ObjectColor.h>
#include <cylinder_detection/CylinderPoseColor.h>

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;
ros::Publisher cylinder_color_pub;
ros::ServiceClient cylinder_color_client;

tf2_ros::Buffer tf2_buffer;

//typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointA;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
  // All the objects needed

  ros::Time time_rec, time_test;
  time_rec = ros::Time::now();

  pcl::PassThrough<PointT> pass;
  pcl::PassThrough<PointT> pass1;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  Eigen::Vector4f centroid;

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::PCLPointCloud2 cloud_blob_filtered;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointA>::Ptr cloud_filtered3(new pcl::PointCloud<PointA>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_blob_filtered);

  // Read in the cloud data
  pcl::fromPCLPointCloud2(cloud_blob_filtered, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;



  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.5);
  pass.filter(*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_filtered);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud_filtered);
  seg.setInputNormals(cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment(*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
  extract.filter(*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

  pcl::PCLPointCloud2 outcloud_plane;
  pcl::toPCLPointCloud2(*cloud_plane, outcloud_plane);
  pubx.publish(outcloud_plane);

  pass1.setInputCloud(cloud);
  pass1.setFilterFieldName("y");
  pass1.setFilterLimits(-0.2, 0.1);
  pass1.filter(*cloud_filtered2);

  // Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*cloud_filtered2);
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals2);

  std::cerr << "PointCloud after filtering has: " << cloud_filtered2->points.size() << " data points." << std::endl;
  
  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);
  seg.setRadiusLimits(0.09, 0.15);
  seg.setInputCloud(cloud_filtered2);
  seg.setInputNormals(cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud(cloud_filtered2);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
  extract.filter(*cloud_cylinder);
  if (cloud_cylinder->points.empty())
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else if (cloud_cylinder->points.size() > 1000)
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;

    pcl::compute3DCentroid(*cloud_cylinder, centroid);
    std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << centroid[3] << std::endl;

    ROS_INFO("num of points: %lu", cloud_cylinder->points.size());

    //ROS_INFO("colors r: %d, g: %d, b: %d", cloud_cylinder->points[10000].r,cloud_cylinder->points[10000].g,cloud_cylinder->points[10000].b);

    color_recognition::ObjectColor msg;
    std::vector<color_recognition::IntList> lists(1000);

    int counter = 0;
    int s = cloud_cylinder->points.size();
    // get 1000 random points' colors
    for (counter = 0; counter < 1000; counter++)
    {
      int ind = rand() % s;
      int rgb[3] = {cloud_cylinder->points[ind].r, cloud_cylinder->points[ind].g, cloud_cylinder->points[ind].b};
      color_recognition::IntList list;
      std::vector<int> rgb_arr(3);
      rgb_arr[0] = cloud_cylinder->points[ind].r;
      rgb_arr[1] = cloud_cylinder->points[ind].g;
      rgb_arr[2] = cloud_cylinder->points[ind].b;
      list.elements = rgb_arr;
      // put array into our 2d array custom msg
      lists[counter] = list;
    }
    // custom msg

    msg.request.data.lists = lists;

    std::string color = "white";

    if (cylinder_color_client.call(msg))
    {
      color = msg.response.color;
    }
    else
    {
      ROS_INFO("ERROR: error when getting color");
    }

    ROS_INFO("COLOR: %s", color.c_str());

    //Create a point in the "camera_rgb_optical_frame"
    geometry_msgs::PointStamped point_camera;
    geometry_msgs::PointStamped point_map;
    visualization_msgs::Marker marker;
    geometry_msgs::TransformStamped tss;

    point_camera.header.frame_id = "camera_rgb_optical_frame";
    point_camera.header.stamp = ros::Time::now();

    point_map.header.frame_id = "map";
    point_map.header.stamp = ros::Time::now();

    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

    try
    {
      time_test = ros::Time::now();

      std::cerr << time_rec << std::endl;
      std::cerr << time_test << std::endl;
      tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
      //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Transform warning: %s\n", ex.what());
    }

    //std::cerr << tss ;

    tf2::doTransform(point_camera, point_map, tss);

    // get marker pose
    geometry_msgs::Pose pose_marker;
    cylinder_detection::CylinderPoseColor cylinder_color_msg;
    pose_marker.position.x = point_map.point.x;
    pose_marker.position.y = point_map.point.y;
    pose_marker.position.z = point_map.point.z;
    // put pose into custom msg
    cylinder_color_msg.pose = pose_marker;
    cylinder_color_msg.color = color;
    // publish marker pose with 1000 RGB color info of points
    cylinder_color_pub.publish(cylinder_color_msg);

    std::cerr << "point_camera: " << point_camera.point.x << " " << point_camera.point.y << " " << point_camera.point.z << std::endl;

    std::cerr << "point_map: " << point_map.point.x << " " << point_map.point.y << " " << point_map.point.z << std::endl;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "cylinder";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = point_map.point.x;
    marker.pose.position.y = point_map.point.y;
    marker.pose.position.z = point_map.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    //pubm.publish (marker);
    pcl::PCLPointCloud2 outcloud_cylinder;
    pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
    puby.publish(outcloud_cylinder);
  }
  else
  {
    ROS_INFO("not enough points but a cylinder found: %lu", cloud_cylinder->points.size());
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cylinder_segment");
  ros::NodeHandle nh;
  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2>("planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2>("cylinder", 1);

  cylinder_color_pub = nh.advertise<cylinder_detection::CylinderPoseColor>("cylinder_pose_color", 100);
  cylinder_color_client = nh.serviceClient<color_recognition::ObjectColor>("cylinder_color");
  // Spin
  ROS_INFO("Node initilized");
  ros::spin();
}
