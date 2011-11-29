#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include "pcl/visualization/pcl_visualizer.h"

int
 main (int argc, char** argv)
{
  sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2), cloud_filtered_blob (new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
//  reader.read ("data/table_scene_lms400.pcd", *cloud_blob);
  reader.read (argv[1], *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  //seg.setModelType (pcl::SACMODEL_PARALLEL_LINES);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  std::vector<std::string> cloud_names;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    cloud_p = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
//    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
//    pcl::PointCloud<pcl::PointXYZ> new_cloud(*cloud_p);
//    clouds.push_back(*pcl::PointCloud<pcl::PointXYZ>::Ptr(new_cloud));
    clouds.push_back(cloud_p);
    cloud_names.push_back(ss.str());

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    i++;
  }

  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.initCameraParameters ();
  int v1(0);
  viewer.createViewPort(0.0,0.0,0.5,1.0,v1);
  viewer.setBackgroundColor (0, 0, 0, v1);
  viewer.addText ("Original Voxel Representation", 10, 10, "v1 label", v1);
  viewer.addPointCloud<pcl::PointXYZ> (cloud_filtered, pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ>(cloud_filtered), "Left Over Voxels", v1);
  
  int v2(0);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  viewer.setBackgroundColor (0, 0, 0, v2);
  viewer.addText ("Segmented Planes", 10, 10, "v2 label", v2);
  for (int i = 0; i < clouds.size(); ++i) {
    std::cout << cloud_names[i] << std::endl;
    viewer.addPointCloud<pcl::PointXYZ> (clouds[i], pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ>(clouds[i]), cloud_names[i], v2);
  }
  viewer.addCoordinateSystem (1.0);
  viewer.spin();


  return (0);
}
