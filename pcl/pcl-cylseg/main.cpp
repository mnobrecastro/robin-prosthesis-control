#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

#define PLANE_MODEL 0

// Define the input camera: REALSENSE_D435 / PICO_FLEXX
#define INPUT_CAMERA REALSENSE_D435
enum cameras {
	REALSENSE_D435,
	PICO_FLEXX
};

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

float dotProduct(pcl::PointXYZ, pcl::PointXYZ);
float normPointT(pcl::PointXYZ);
std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<PointT>&, pcl::PointXYZ, pcl::PointXYZ);
std::array<float, 6> getPointCloudBoundaries(const pcl::PointCloud<PointT>&);
void correctCylShape(pcl::ModelCoefficients&, const pcl::ModelCoefficients&, const pcl::PointCloud<PointT>&);

void pp_callback(const pcl::visualization::PointPickingEvent&, void*);


int main (int argc, char** argv)
{
	// All the objects needed
	////pcl::PCDReader reader;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	////pcl::PCDWriter writer;
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

	// Checking program arguments
	if (argc < 1) {
		printf("Usage :\n");
		printf("\t\tfile.ply\n");
		PCL_ERROR("Provide one ply file.\n");
		return (-1);
	}

	// Read in the cloud data
	pcl::console::TicToc time;
	time.tic();
	if (pcl::io::loadPLYFile(argv[1], *cloud) < 0) {
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return (-1);
	}
	std::cout << "\nLoaded file " << argv[1] << " (" << cloud->size() << " points) in " << time.toc() << " ms\n" << std::endl;

	// Build a passthrough filter to remove unwated points
#ifdef INPUT_CAMERA
	std::array<float, 6> filter_lims;
	switch (INPUT_CAMERA) {
		case REALSENSE_D435:
			filter_lims = { -0.075, 0.075, -0.100, 0.100, -0.300, -0.110 }; // realsense depth neg z-axis (MinZ 0.110m)
		case PICO_FLEXX:
			filter_lims = { -0.075, 0.075, -0.100, 0.100, -0.300, -0.110 }; // picoflexx depth ?-axis (Min ? m)
		default:
			std::cout << "Only REALSENSE_D435 or PICO_FLEXX can be defined as INPUT_CAMERA." << std::endl;
			return 1;
	}
#else
	std::cout << "Please define an INPUT_CAMERA (REALSENSE_D435 or PICO_FLEXX)." << std::endl;
	return 1;
#endif

	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(filter_lims[0], filter_lims[1]);
	pass.filter(*cloud_filtered);
	//
	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(filter_lims[2], filter_lims[3]);
	pass.filter(*cloud_filtered);
	//
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (filter_lims[4], filter_lims[5]);
	pass.filter (*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_filtered);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

#if PLANE_MODEL

	time.tic();
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
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points (in " << time.toc() << " ms)." << std::endl;

	// Obtain the plane cloud boundaries
	std::array<float, 6> bounds_plane(getPointCloudBoundaries(*cloud_plane));
	std::cerr << "\nPlane boundaries: "
		<< "\n\tx: " << "[" << bounds_plane[0] << "," << bounds_plane[1] << "]"
		<< "\n\ty: " << "[" << bounds_plane[2] << "," << bounds_plane[3] << "]"
		<< "\n\tz: " << "[" << bounds_plane[4] << "," << bounds_plane[5] << "]"
		<< std::endl;

	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud_filtered2);
	extract_normals.setNegative (true);
	extract_normals.setInputCloud (cloud_normals);
	extract_normals.setIndices (inliers_plane);
	extract_normals.filter (*cloud_normals2);

#else

	cloud_filtered2 = cloud_filtered;
	cloud_normals2 = cloud_normals;

#endif

	time.tic();
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (100); //10000
	seg.setDistanceThreshold (0.05);
	seg.setRadiusLimits (0, 0.040);
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
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points (in " << time.toc() << " ms)." << std::endl;
	}

	// Obtain the cylinder cloud boundaries
	std::array<float, 6> bounds_cylinder(getPointCloudBoundaries(*cloud_cylinder));
	std::cerr << "\nCylinder boundaries: "
		<< "\n\tx: " << "[" << bounds_cylinder[0] << "," << bounds_cylinder[1] << "]"
		<< "\n\ty: " << "[" << bounds_cylinder[2] << "," << bounds_cylinder[3] << "]"
		<< "\n\tz: " << "[" << bounds_cylinder[4] << "," << bounds_cylinder[5] << "]"
		<< std::endl;

	// Visualization
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
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

#if PLANE_MODEL

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_plane_color_h(cloud_plane, 20, 180, 20);
	viewer.addPointCloud(cloud_plane, cloud_plane_color_h, "cloud_plane", vp);

#else

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered2_color_h(cloud_filtered2, 20, 180, 20);
	viewer.addPointCloud(cloud_filtered2, cloud_filtered2_color_h, "cloud_filtered2", vp);

#endif

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cylinder_color_h(cloud_cylinder, 180, 20, 20);
	viewer.addPointCloud(cloud_cylinder, cloud_cylinder_color_h, "cloud_cylinder", vp);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);

	// Add global reference frame (on-camera)
	viewer.addCoordinateSystem(0.25);

	// Plot cylinder shape
	pcl::ModelCoefficients::Ptr corrected_coefs_cylinder(new pcl::ModelCoefficients);
	correctCylShape(*corrected_coefs_cylinder, *coefficients_cylinder, *cloud_cylinder);
	viewer.addCylinder(*corrected_coefs_cylinder, "cylinder");

	// Plot cylinder longitudinal axis //PointT
	PointT point_on_axis( (*coefficients_cylinder).values[0], (*coefficients_cylinder).values[1], (*coefficients_cylinder).values[2] );
	PointT axis_direction( point_on_axis.x + (*coefficients_cylinder).values[3], point_on_axis.y + (*coefficients_cylinder).values[4], point_on_axis.z + (*coefficients_cylinder).values[5] );
	PointT cam_origin(0.0, 0.0, 0.0);
	PointT axis_projection((*coefficients_cylinder).values[3], (*coefficients_cylinder).values[4], 0.0);
	viewer.addLine(cam_origin, axis_projection, "line");

	// Calculate the angular difference
	float dTheta(M_PI - std::atan2(axis_projection.y, axis_projection.x));
	std::string action;
	if (std::abs(dTheta) < 5 * M_PI/ 180)
		action = "grab";
	else if(dTheta > 0)
		action = "rotate_right";
	else
		action = "rotate_left";
	std::cout << "\nCurrent angle: " << dTheta * 180/M_PI << "\tAction: " << action;

	// Adding text descriptions in each viewport
	viewer.addText("White: Original point cloud\nRed: RANSAC point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", vp);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	viewer.registerPointPickingCallback(pp_callback, (void*)& viewer);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}

	return (0);
}




////////////////////////////////////////

float dotProduct(pcl::PointXYZ a, pcl::PointXYZ b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float normPointT(pcl::PointXYZ c)
{
	return std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
}

std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<PointT>& cloud, pcl::PointXYZ center, pcl::PointXYZ direction)
{
	std::array<float, 2> arr = {1000.0, -1000.0};
	pcl::PointXYZ vec;
	float scalar_proj;
	for (size_t i = 0; i < cloud.points.size(); ++i) { 
		vec.x = cloud.points[i].x - center.x;
		vec.y = cloud.points[i].y - center.y;
		vec.z = cloud.points[i].z - center.z;
		scalar_proj = dotProduct(direction, vec) / normPointT(direction);
		if (scalar_proj < arr[0])
			arr[0] = scalar_proj;
		if (scalar_proj > arr[1])
			arr[1] = scalar_proj;
	}
	return arr;
}

std::array<float, 6> getPointCloudBoundaries(const pcl::PointCloud<PointT>& cloud)
{
	std::array<float, 6> arr = { 1000.0, -1000.0, 1000.0, -1000.0, 1000.0, -1000.0};
	for (auto point : cloud.points) {
		if (point.x < arr[0])
			arr[0] = point.x;
		if (point.x > arr[1])
			arr[1] = point.x;
		if (point.y < arr[2])
			arr[2] = point.y;
		if (point.y > arr[3])
			arr[3] = point.y;
		if (point.z < arr[4])
			arr[4] = point.z;
		if (point.z > arr[5])
			arr[5] = point.z;
	}
	return arr;
}

void correctCylShape(pcl::ModelCoefficients& cyl, const pcl::ModelCoefficients& coefficients, const pcl::PointCloud<PointT>& cloud)
{
	pcl::PointXYZ point_on_axis(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
	pcl::PointXYZ axis_direction(coefficients.values[3], coefficients.values[4], coefficients.values[5]);	
	std::array<float, 2> arr(getPointCloudExtremes(cloud, point_on_axis, axis_direction));

	pcl::PointXYZ point_bottom;
	point_bottom.x = point_on_axis.x + arr[0] * axis_direction.x / normPointT(axis_direction);
	point_bottom.y = point_on_axis.y + arr[0] * axis_direction.y / normPointT(axis_direction);
	point_bottom.z = point_on_axis.z + arr[0] * axis_direction.z / normPointT(axis_direction);
	pcl::PointXYZ bottom_top_direction;
	bottom_top_direction.x = (-arr[0] + arr[1]) * axis_direction.x / normPointT(axis_direction);
	bottom_top_direction.y = (-arr[0] + arr[1]) * axis_direction.y / normPointT(axis_direction);
	bottom_top_direction.z = (-arr[0] + arr[1]) * axis_direction.z / normPointT(axis_direction);

	cyl.values.push_back(point_bottom.x);
	cyl.values.push_back(point_bottom.y);
	cyl.values.push_back(point_bottom.z);
	cyl.values.push_back(bottom_top_direction.x);
	cyl.values.push_back(bottom_top_direction.y);
	cyl.values.push_back(bottom_top_direction.z);
	cyl.values.push_back(coefficients.values[6]);
}


////////////////////////////////////////

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	std::cout << "Picking event active" << std::endl;
	if (event.getPointIndex() != -1) {
		float x, y, z;
		event.getPoint(x, y, z);
		std::cout << x << ";" << y << ";" << z << std::endl;
	}
}