#include <ctime>
#include <thread>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

//#define DEBUG

// Define the input camera: REALSENSE_D435 / PICO_FLEXX
#define REALSENSE_D435 0
#define	PICO_FLEXX 1
#define INPUT_CAMERA REALSENSE_D435

// Include PLANE_MODEL in the RANSAC pipeline.
#define PLANE_MODEL 0
#define CYLINDER_MODEL 1

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points&);
float dotProduct(pcl::PointXYZ, pcl::PointXYZ);
float normPointT(pcl::PointXYZ);
std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<PointT>&, pcl::PointXYZ, pcl::PointXYZ);
std::array<float, 6> getPointCloudBoundaries(const pcl::PointCloud<PointT>&);
void correctCylShape(pcl::ModelCoefficients&, const pcl::ModelCoefficients&, const pcl::PointCloud<PointT>&);

namespace michelangelo {
	enum Camera {
		_REALSENSE_D435,
		_PICO_FLEXX
	};
	void printCamInfo(rs2::device& dev);

	void correctAngle(pcl::PointXYZ&, float);
	pcl::PointXYZ otherAngle(const pcl::PointXYZ&, float);
	float readAngle(pcl::PointXYZ);
	std::string setAction(float);	
}

void pp_callback(const pcl::visualization::PointPickingEvent&, void*);


int main (int argc, char** argv)
{	
	//  Visualiser initiallization
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	int vp(0); // Default viewport
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, vp);
	viewer.setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
	viewer.setSize(800, 600); 
	float bckgr_gray_level = 1.0;  // Black:=0.0
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
	viewer.addCoordinateSystem(0.25); // Global reference frame (on-camera)
	viewer.addText("White: Original point cloud\nRed: RANSAC point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", vp);
	
	//viewer.registerPointPickingCallback(pp_callback, (void*)&viewer);
	
#ifdef INPUT_CAMERA
	std::array<float, 6> filter_lims;

#if INPUT_CAMERA == REALSENSE_D435

	rs2::device dev = [] {
		rs2::context ctx;
		std::cout << "Waiting for device..." << std::endl;
		while (true) {
			for (auto&& dev : ctx.query_devices())
				return dev;
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}();
	michelangelo::printCamInfo(dev);

	bool disparity(true);

	rs2::pipeline pipe;
	rs2::config cfg;
	std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	std::cout << "Opening pipeline for " << serial_number << std::endl;
	cfg.enable_device(serial_number);	
	if (!disparity) {
		cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 90);
		//cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
		////cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
	} else {
		//cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
		cfg.enable_stream(RS2_STREAM_DEPTH, 848, 100, RS2_FORMAT_Z16, 100); // USB3.0 only!
	}
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

	auto advanced_dev = dev.as<rs400::advanced_mode>();
	STDepthTableControl depth_table = advanced_dev.get_depth_table();	
	if (!disparity) {
		depth_table.depthUnits = 1000; // 0.001m
		depth_table.depthClampMin = 0;
		depth_table.depthClampMax = 200; // mm
		depth_table.disparityShift = 0;		
	} else {
		depth_table.depthUnits = 1000;  // 0.001m
		depth_table.depthClampMin = 0;
		depth_table.depthClampMax = 200; // mm
		depth_table.disparityShift = 145; // 145@30 or [125-175]@100
	}
	advanced_dev.set_depth_table(depth_table);

	rs2::pipeline_profile profile = pipe.start(cfg);

	if (!disparity) {
		filter_lims = { -0.100, 0.100, -0.100, 0.100, 0.000, 0.300 }; // realsense depth neg z-axis (MinZ 0.110m)
	} else {
		filter_lims = { -0.050, 0.050, -0.050, 0.050, 0.050, 0.200 }; // realsense depth neg z-axis (MinZ 0.110m)
	}
	std::cout << "Using the input camera REALSENSE_D435...\n" << std::endl;
#endif
#if INPUT_CAMERA == PICO_FLEXX
	filter_lims = {-0.075, 0.075, -0.100, 0.100, -0.300, -0.110 }; // picoflexx depth ?-axis (Min ? m)
	std::cout << "Using the input camera PICO_FLEXX...\n" << std::endl;		
#endif
#if INPUT_CAMERA != REALSENSE_D435 && INPUT_CAMERA != PICO_FLEXX
	std::cout << "ERROR: Only REALSENSE_D435 or PICO_FLEXX can be defined as INPUT_CAMERA.\n" << std::endl;
	return -1;
#endif
#else
	std::cout << "ERROR: Please define an INPUT_CAMERA (REALSENSE_D435 or PICO_FLEXX).\n" << std::endl;
	return -1;
#endif // INPUT_CAMERA
		
	// Pointcloud objects
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);

	// PCL objects
	pcl::PassThrough<PointT> pass(true);
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
		
	pcl::console::TicToc time;
	pcl::console::TicToc tloop;

	while (!viewer.wasStopped()) {

		tloop.tic();

		viewer.removeAllShapes();
		viewer.removeAllPointClouds();


#if INPUT_CAMERA == REALSENSE_D435

		time.tic();
		// RealSense2 pointcloud, points and pipeline objects
		rs2::pointcloud pc;
		rs2::points points;

		// Wait for the next set of frames from the camera
		rs2::frameset frames(pipe.wait_for_frames());
		rs2::depth_frame depth(frames.get_depth_frame());

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		// Transform rs2::pointcloud into pcl::PointCloud<PointT>::Ptr
		cloud = points_to_pcl(points);
		std::cout << "\nRead pointcloud from " << cloud->size() << " data points (in " << time.toc() << " ms)." << std::endl;
#endif
#if INPUT_CAMERA == PICO_FLEXX

		// Transform rs2::pointcloud into pcl::PointCloud<PointT>::Ptr
		cloud = points_to_pcl(points);
		std::cout << "\nRead pointcloud from (" << cloud->size() << " data points (in " << time.toc() << " ms)." << std::endl;
#endif

#ifdef DEBUG
		// Draw raw pointcloud
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
			(int)255 * txt_gray_lvl);
		viewer.addPointCloud(cloud, cloud_in_color_h, "cloud_in", vp);
#endif //DEBUG

		// PCL objects
		pcl::PassThrough<PointT> pass(true);
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
		pcl::ExtractIndices<PointT> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

		pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);


		time.tic();
		// Build a passthrough filter to remove unwated points
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
		pass.setInputCloud(cloud_filtered);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(filter_lims[4], filter_lims[5]);
		pass.filter(*cloud_filtered);
		std::cerr << "PointCloud after filtering: " << cloud_filtered->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;

		time.tic();
		// Downsampling the filtered point cloud		
		pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
		dsfilt.setInputCloud(cloud_filtered);
		if (!disparity) {
			dsfilt.setLeafSize(0.005f, 0.005f, 0.005f); //0.01f
		} else {
			dsfilt.setLeafSize(0.002f, 0.002f, 0.002f); //0.01f
		}
		dsfilt.filter(*cloud_filtered);
		std::cerr << "PointCloud after downsampling: " << cloud_filtered->width * cloud_filtered->height << "=" << cloud_filtered->points.size()
			<< " data points (in " << time.toc() << " ms)." << std::endl; //pcl::getFieldsList(*cloud_filtered)

#ifndef DEBUG
		// Draw filtered PointCloud
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_in_color_h(cloud_filtered, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
			(int)255 * txt_gray_lvl);
		viewer.addPointCloud(cloud_filtered, cloud_filtered_in_color_h, "cloud_filtered_in", vp);
#endif //DEBUG

		if (cloud_filtered->points.size() < 100) {
			std::cerr << "* Not enough points to perform segmentation." << std::endl;
			std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
			continue;
		}

		// Estimate point normals
		ne.setSearchMethod(tree);
		ne.setInputCloud(cloud_filtered);
		ne.setKSearch(50);
		ne.compute(*cloud_normals);

#if PLANE_MODEL

		time.tic();
		// Create the segmentation object for the planar model and set all the parameters
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
		seg.setNormalDistanceWeight(0.1);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.03);
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
		std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;

		// Obtain the plane cloud boundaries
		std::array<float, 6> bounds_plane(getPointCloudBoundaries(*cloud_plane));
		std::cerr << "\nPlane boundaries: "
			<< "\n\tx: " << "[" << bounds_plane[0] << "," << bounds_plane[1] << "]"
			<< "\n\ty: " << "[" << bounds_plane[2] << "," << bounds_plane[3] << "]"
			<< "\n\tz: " << "[" << bounds_plane[4] << "," << bounds_plane[5] << "]"
			<< std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_filtered2);
		extract_normals.setNegative(true);
		extract_normals.setInputCloud(cloud_normals);
		extract_normals.setIndices(inliers_plane);
		extract_normals.filter(*cloud_normals2);

#ifndef DEBUG
		// Transformed point cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_plane_color_h(cloud_plane, 20, 180, 20);
		viewer.addPointCloud(cloud_plane, cloud_plane_color_h, "cloud_plane", vp);
#endif //DEBUG
#else
		/*
#ifndef DEBUG
		// Transformed point cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered2_color_h(cloud_filtered2, 20, 180, 20);
		viewer.addPointCloud(cloud_filtered2, cloud_filtered2_color_h, "cloud_filtered2", vp);
#endif //DEBUG
		*/
#endif //PLANE_MODEL

#if CYLINDER_MODEL

		time.tic();
		// Create the segmentation object for cylinder segmentation and set all the parameters
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight(0.1);
		seg.setMaxIterations(10000);
		seg.setDistanceThreshold(0.05);
		seg.setRadiusLimits(0.005, 0.040);
#if PLANE_MODEL
		seg.setInputCloud(cloud_filtered2);
		seg.setInputNormals(cloud_normals2);
#else	
		seg.setInputCloud(cloud_filtered);
		seg.setInputNormals(cloud_normals);
#endif //PLANE_MODEL

		// Obtain the cylinder inliers and coefficients
		seg.segment(*inliers_cylinder, *coefficients_cylinder);
		//std::cerr << "Cylinder inliers: " << *inliers_cylinder << std::endl;
		//std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

		// Save the cylinder inliers
#if PLANE_MODEL
		extract.setInputCloud(cloud_filtered2);
#else
		extract.setInputCloud(cloud_filtered);
#endif //PLANE_MODEL
		extract.setIndices(inliers_cylinder);
		extract.setNegative(false);
		pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
		extract.filter(*cloud_cylinder);

		if (cloud_cylinder->points.empty()) {
			std::cerr << "\tCan't find the cylindrical component." << std::endl;
			std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
			continue;
		} else {
			std::cerr << "PointCloud CYLINDER: " << cloud_cylinder->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;
		}

		// Obtain the cylinder cloud boundaries
		std::array<float, 6> bounds_cylinder(getPointCloudBoundaries(*cloud_cylinder));
		/*std::cerr << "\nCylinder boundaries: "
			<< "\n\tx: " << "[" << bounds_cylinder[0] << "," << bounds_cylinder[1] << "]"
			<< "\n\ty: " << "[" << bounds_cylinder[2] << "," << bounds_cylinder[3] << "]"
			<< "\n\tz: " << "[" << bounds_cylinder[4] << "," << bounds_cylinder[5] << "]"
			<< std::endl;*/

#ifndef DEBUG
		// ICP aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cylinder_color_h(cloud_cylinder, 180, 20, 20);
		viewer.addPointCloud(cloud_cylinder, cloud_cylinder_color_h, "cloud_cylinder", vp);

		// Plot cylinder shape
		pcl::ModelCoefficients::Ptr corrected_coefs_cylinder(new pcl::ModelCoefficients);
		correctCylShape(*corrected_coefs_cylinder, *coefficients_cylinder, *cloud_cylinder);
		viewer.addCylinder(*corrected_coefs_cylinder, "cylinder");
#endif //DEBUG

		// Plot cylinder longitudinal axis //PointT
		PointT point_on_axis((*coefficients_cylinder).values[0], (*coefficients_cylinder).values[1], (*coefficients_cylinder).values[2]);
		PointT axis_direction(point_on_axis.x + (*coefficients_cylinder).values[3], point_on_axis.y + (*coefficients_cylinder).values[4], point_on_axis.z + (*coefficients_cylinder).values[5]);
		PointT cam_origin(0.0, 0.0, 0.0);
		PointT axis_projection((*coefficients_cylinder).values[3], (*coefficients_cylinder).values[4], 0.0);

		float camAngle(90.0 * M_PI / 180);
		michelangelo::correctAngle(axis_projection, camAngle);
#ifndef DEBUG
		viewer.addLine(cam_origin, axis_projection, "line");
#endif //DEBUG

		// Calculate the angular difference			
		//float dTheta(M_PI - std::atan2(axis_projection.y, axis_projection.x));
		float dTheta(michelangelo::readAngle(axis_projection));
		std::string action(michelangelo::setAction(dTheta));
		std::cout << "* Current angle: " << dTheta * 180 / M_PI << "\tAction: " << action << std::endl;

#endif	// CYLINDER_MODEL

		viewer.spinOnce(1, true);
		std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
	}

	pipe.stop();

	return (0);
}




////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points) {
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}

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

void michelangelo::correctAngle(pcl::PointXYZ& axis_projection, float camAngle)
{
	float ang(michelangelo::readAngle(axis_projection) + camAngle);
	if (ang > M_PI) {
		ang -= 2 * M_PI;
	}

	if (ang < 0.0) {
		axis_projection.x = -axis_projection.x;
		axis_projection.y = -axis_projection.y;
	}
	
	pcl::PointXYZ proj1(axis_projection);
	pcl::PointXYZ proj2(proj1);
	proj2.x = -proj2.x;
	proj2.y = -proj2.y;
}

pcl::PointXYZ michelangelo::otherAngle(const pcl::PointXYZ& axis_projection, float camAngle)
{
	pcl::PointXYZ other(axis_projection);
	other.x = -other.x;
	other.y = -other.y;
	return other;
}


float michelangelo::readAngle(pcl::PointXYZ axis_projection)
{
	float angle(std::atan2(axis_projection.y, axis_projection.x));
	return angle;
}

std::string michelangelo::setAction(float angle)
{
	std::string action;
	if (std::abs(angle) < 5*M_PI/180)
		action = "grab";
	else if (angle > 0)
		action = "rotate_right";
	else
		action = "rotate_left";
	return action;
}

void michelangelo::printCamInfo(rs2::device& dev) {
	std::cout << "Device found:" << std::endl;
	std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << " "
		<< dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "
		<< dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;

	auto sensors = dev.query_sensors();
	for (rs2::sensor& sensor : sensors) {
		std::cout << "Sensor " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
		for (rs2::stream_profile& profile : sensor.get_stream_profiles()) {
			if (profile.is<rs2::video_stream_profile>() && profile.stream_name() == "Depth") {
				rs2::video_stream_profile video_stream_profile = profile.as<rs2::video_stream_profile>();
				std::cout << " Video stream: " << video_stream_profile.format() << " " <<
					video_stream_profile.width() << "x" << video_stream_profile.height() << " @" << video_stream_profile.fps() << "Hz" << std::endl;
			}
			if (profile.is<rs2::motion_stream_profile>() && profile.stream_name() == "Accel") {
				rs2::motion_stream_profile motion_stream_profile = profile.as<rs2::motion_stream_profile>();
				std::cout << " Motion stream: " << motion_stream_profile.format() << " " <<
					motion_stream_profile.stream_type() << " @" << motion_stream_profile.fps() << "Hz" << std::endl;
			}
			if (profile.is<rs2::pose_stream_profile>() && profile.stream_name() == "Gyro") {
				rs2::pose_stream_profile pose_stream_profile = profile.as<rs2::pose_stream_profile>();
				std::cout << " Pose stream: " << pose_stream_profile.format() << " " <<
					pose_stream_profile.stream_type() << " @" << pose_stream_profile.fps() << "Hz" << std::endl;
			}
			�//std::cout << "  stream " << profile.stream_name() << " " << profile.stream_type() << " " << profile.format() << " " << " " << profile.fps() << std::endl;
		}
	}
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