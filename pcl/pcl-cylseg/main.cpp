#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

int main (int argc, char** argv)
{
	// All the objects needed
	pcl::PCDReader reader;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

	// Read in the cloud data
	reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.5);
	pass.filter (*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_filtered);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight (0.1);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.03);
	seg.setInputCloud (cloud_filtered);
	seg.setInputNormals (cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment (*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud (cloud_filtered);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);

	// Write the planar inliers to disk
	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
	extract.filter (*cloud_plane);
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
	//writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud_filtered2);
	extract_normals.setNegative (true);
	extract_normals.setInputCloud (cloud_normals);
	extract_normals.setIndices (inliers_plane);
	extract_normals.filter (*cloud_normals2);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.05);
	seg.setRadiusLimits (0, 0.1);
	seg.setInputCloud (cloud_filtered2);
	seg.setInputNormals (cloud_normals2);

	// Obtain the cylinder inliers and coefficients
	seg.segment (*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	extract.setInputCloud (cloud_filtered2);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
	extract.filter (*cloud_cylinder);
	if (cloud_cylinder->points.empty ()) 
	std::cerr << "Can't find the cylindrical component." << std::endl;
	else {
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
		writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
	}

	// Visualization
	pcl::visualization::PCLVisualizer viewer("Viewer Window");
	// Create viewport
	int vp(0);
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, vp);

	// The color we will be using
	float bckgr_gray_level = 1.0;  // Black:=0.0
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud, cloud_in_color_h, "cloud_in", vp);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_plane_color_h(cloud_plane, 20, 180, 20);
	viewer.addPointCloud(cloud_plane, cloud_plane_color_h, "cloud_plane", vp);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cylinder_color_h(cloud_cylinder, 180, 20, 20);
	viewer.addPointCloud(cloud_cylinder, cloud_cylinder_color_h, "cloud_cylinder", vp);

	// Adding text descriptions in each viewport
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", vp);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}

	return (0);
}