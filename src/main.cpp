#include <ctime>
#include <array>
#include <thread>
#include <omp.h>

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
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

//#define DEBUG

// Define the input camera: REALSENSE_D435 / PICO_FLEXX
#define REALSENSE_D435 0
#define	PICO_FLEXX 1
#define INPUT_CAMERA REALSENSE_D435

// Include PLANE_SEGMENTATION in the RANSAC pipeline.
#define PLANE_SEGMENTATION 0
#define PRIMITIVE_MODEL 1

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points&);
float dotProduct(pcl::PointXYZ, pcl::PointXYZ);
float normPointT(pcl::PointXYZ);
std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<PointT>&, pcl::PointXYZ, pcl::PointXYZ);
std::array<float, 6> getPointCloudBoundaries(const pcl::PointCloud<PointT>&);
void correctCylShape(pcl::ModelCoefficients&, const pcl::ModelCoefficients&, const pcl::PointCloud<PointT>&);

namespace michelangelo {
	enum Primitive {
		PRIMITIVE_PLANE,
		PRIMITIVE_CYLINDER,
		PRIMITIVE_SPHERE
	};

	enum Segmentation {
		SEG_RANSAC,
		SEG_LCCP
	};

	enum Camera {
		_REALSENSE_D435,
		_PICO_FLEXX
	};
	void printCamInfo(rs2::device& dev);

	void correctAngle(pcl::PointXYZ&, float);
	pcl::PointXYZ otherAngle(const pcl::PointXYZ&, float);
	float readAngle(pcl::PointXYZ);
	std::string setAction(float);

	class PrimitiveModel
	{
	private:
		pcl::ModelCoefficients coefficients; //(new pcl::ModelCoefficients);
		pcl::PointIndices inliers; //(new pcl::PointIndices);
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

	public:
		PrimitiveModel(michelangelo::Primitive prim)
		{
			//this->coefficients = new pcl::ModelCoefficients;				
			switch (prim) {
			case michelangelo::PRIMITIVE_PLANE:
				seg.setModelType(pcl::SACMODEL_PLANE);
				break;
			case michelangelo::PRIMITIVE_SPHERE:
				seg.setModelType(pcl::SACMODEL_SPHERE);
				break;
			case michelangelo::PRIMITIVE_CYLINDER:
				seg.setModelType(pcl::SACMODEL_CYLINDER);
				break;
			}
		}

		void segment(pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
		{
			// Create the segmentation object for primitive segmentation and set all the parameters
			seg.setOptimizeCoefficients(true);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setNormalDistanceWeight(0.1);
			seg.setMaxIterations(10000);
			seg.setDistanceThreshold(0.01);
			seg.setRadiusLimits(0.005, 0.050);
#if PLANE_SEGMENTATION
			seg.setInputCloud(cloud_filtered2);
			seg.setInputNormals(cloud_normals2);
#else	
			seg.setInputCloud(cloud_filtered);
			seg.setInputNormals(cloud_normals);
#endif //PLANE_SEGMENTATION

			// Obtain the primitive inliers and coefficients
			seg.segment(this->inliers, this->coefficients);
		}
	};
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
	float bckgr_gray_level = 0.0;  // Black:=0.0
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

	bool disparity(false);

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
	//cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

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
	pcl::PointCloud<PointT>::Ptr cloud_primitive(new pcl::PointCloud<PointT>());

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h((int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_in_color_h((int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_plane_color_h(20, 180, 20);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_primitive_color_h(180, 20, 20);

	// PCL objects
	pcl::PassThrough<PointT> pass(true);
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_primitive(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_primitive(new pcl::PointIndices);

	// PCL Primitive
	std::array<michelangelo::Primitive, 3> primitives = { michelangelo::PRIMITIVE_PLANE, michelangelo::PRIMITIVE_SPHERE, michelangelo::PRIMITIVE_CYLINDER };
	michelangelo::Primitive prim(primitives[2]);	

	// LCCP objects
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud;
	pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud;
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud;
	pcl::LCCPSegmentation<pcl::PointXYZRGBA> lccp;
	
	///  Default values of parameters before parsing
	// Supervoxel Parameters
	float voxel_resolution = 0.0075f;
	float seed_resolution = 0.03f;
	float color_importance = 0.0f;
	float spatial_importance = 1.0f;
	float normal_importance = 4.0f;
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

	// LCCPSegmentation Parameters
	float concavity_tolerance_threshold = 10;
	float smoothness_threshold = 0.1;
	std::uint32_t min_segment_size = 0;
	bool use_extended_convexity = false;
	bool use_sanity_criterion = false;
	
	float normals_scale;
	normals_scale = seed_resolution / 2.0;
	unsigned int k_factor = 0;
	if (use_extended_convexity)
		k_factor = 1;
	pcl::SupervoxelClustering<pcl::PointXYZRGBA> supervox(voxel_resolution, seed_resolution);
	

	michelangelo::Segmentation segmethod(michelangelo::SEG_LCCP);

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
		cloud_in_color_h.setInputCloud(cloud);
		viewer.addPointCloud(cloud, cloud_in_color_h, "cloud_in", vp);
#endif //DEBUG

		bool FILTER(true);
		if (FILTER) {
			time.tic();
			// Build a passthrough filter to remove unwated points
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(filter_lims[0], filter_lims[1]);
			pass.filter(*cloud_filtered);
			//
			if (!disparity) {
				pass.setInputCloud(cloud_filtered);
				pass.setFilterFieldName("y");
				pass.setFilterLimits(filter_lims[2], filter_lims[3]);
				pass.filter(*cloud_filtered);
				//
				pass.setInputCloud(cloud_filtered);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(filter_lims[4], filter_lims[5]);
				pass.filter(*cloud_filtered);
			}
			std::cerr << "PointCloud after filtering: " << cloud_filtered->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;
		} else {
			pcl::copyPointCloud(*cloud, *cloud_filtered);
		}

		bool DOWNSAMPLING(true);
		if (DOWNSAMPLING) {
			time.tic();
			// Downsampling the filtered point cloud		
			pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
			dsfilt.setInputCloud(cloud_filtered);
			if (!disparity) {
				dsfilt.setLeafSize(0.002f, 0.002f, 0.002f); //0.005f //0.01f
			} else {
				dsfilt.setLeafSize(0.002f, 0.002f, 0.002f); //0.01f
			}
			dsfilt.filter(*cloud_filtered);
			std::cerr << "PointCloud after downsampling: " << cloud_filtered->width * cloud_filtered->height << "=" << cloud_filtered->points.size()
				<< " data points (in " << time.toc() << " ms)." << std::endl; //pcl::getFieldsList(*cloud_filtered)
		}

#ifndef DEBUG
		// Draw filtered PointCloud
		cloud_filtered_in_color_h.setInputCloud(cloud_filtered);
		viewer.addPointCloud(cloud_filtered, cloud_filtered_in_color_h, "cloud_filtered_in", vp);
#endif //DEBUG

		if (cloud_filtered->points.size() < 100) {
			std::cerr << "* Not enough points to perform segmentation." << std::endl;
			std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
			continue;
		}

		switch (segmethod) {
			case michelangelo::SEG_RANSAC:

				// Estimate point normals
				std::cout << "Computing normals...";
				ne.setSearchMethod(tree);
				ne.setInputCloud(cloud_filtered);
				ne.setKSearch(50);
				ne.compute(*cloud_normals);
				std::cout << " done." << std::endl;

#if PLANE_SEGMENTATION

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
				cloud_plane_color_h.setInputCloud(cloud_plane);
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
	#endif //PLANE_SEGMENTATION

	#if PRIMITIVE_MODEL
							   
				//std::array<michelangelo::Primitive, 3> prims = { michelangelo::PRIMITIVE_PLANE, michelangelo::PRIMITIVE_SPHERE, michelangelo::PRIMITIVE_CYLINDER };
				//std::array<michelangelo::PrimitiveModel, 3> prim_models();

				/*#pragma omp parallel num_threads(3){
					// This code will be executed by three threads.

					// Chunks of this loop will be divided amongst
					// the (three) threads of the current team.
				#pragma omp for
					for (int n = 0; n < 10; ++n)
						printf(" %d", n);
				}*/

				time.tic();
				// Create the segmentation object for primitive segmentation and set all the parameters
				seg.setOptimizeCoefficients(true);
				seg.setMethodType(pcl::SAC_RANSAC);
				switch (prim) {
				case michelangelo::PRIMITIVE_PLANE:
					seg.setModelType(pcl::SACMODEL_PLANE);
					break;
				case michelangelo::PRIMITIVE_SPHERE:
					seg.setModelType(pcl::SACMODEL_SPHERE);
					break;
				case michelangelo::PRIMITIVE_CYLINDER:
					seg.setModelType(pcl::SACMODEL_CYLINDER);
					break;
				}
				seg.setNormalDistanceWeight(0.1);
				seg.setMaxIterations(10000);
				seg.setDistanceThreshold(0.01); //0.05
				seg.setRadiusLimits(0.005, 0.050);
	#if PLANE_SEGMENTATION
				seg.setInputCloud(cloud_filtered2);
				seg.setInputNormals(cloud_normals2);
	#else	
				seg.setInputCloud(cloud_filtered);
				seg.setInputNormals(cloud_normals);
	#endif //PLANE_SEGMENTATION

				// Obtain the primitive inliers and coefficients
				seg.segment(*inliers_primitive, *coefficients_primitive);
				//std::cerr << "primitive inliers: " << *inliers_primitive << std::endl;
				//std::cerr << "primitive coefficients: " << *coefficients_primitive << std::endl;

				// Save the primitive inliers
	#if PLANE_SEGMENTATION
				extract.setInputCloud(cloud_filtered2);
	#else
				extract.setInputCloud(cloud_filtered);
	#endif //PLANE_SEGMENTATION
				extract.setIndices(inliers_primitive);
				extract.setNegative(false);
				//pcl::PointCloud<PointT>::Ptr cloud_primitive(new pcl::PointCloud<PointT>());
				extract.filter(*cloud_primitive);

				if (cloud_primitive->points.empty()) {
					std::cerr << "\tCan't find the cylindrical component." << std::endl;
					std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
					continue;
				}
				else {
					std::cerr << "PointCloud PRIMITIVE: " << cloud_primitive->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;
				}

				// Obtain the primitive cloud boundaries
				std::array<float, 6> bounds_primitive(getPointCloudBoundaries(*cloud_primitive));
				/*std::cerr << "\nCylinder boundaries: "
					<< "\n\tx: " << "[" << bounds_cylinder[0] << "," << bounds_cylinder[1] << "]"
					<< "\n\ty: " << "[" << bounds_cylinder[2] << "," << bounds_cylinder[3] << "]"
					<< "\n\tz: " << "[" << bounds_cylinder[4] << "," << bounds_cylinder[5] << "]"
					<< std::endl;*/

	#ifndef DEBUG
				// ICP aligned point cloud is red
				cloud_primitive_color_h.setInputCloud(cloud_primitive);
				viewer.addPointCloud(cloud_primitive, cloud_primitive_color_h, "cloud_primitive", vp);

				// Plot primitive shape
				switch (prim) {
				case michelangelo::PRIMITIVE_PLANE:
					//viewer.addCylinder(*corrected_coefs_cylinder, "cylinder");
					break;
				case michelangelo::PRIMITIVE_SPHERE:
					viewer.addSphere(*coefficients_primitive, "sphere");
					break;
				case michelangelo::PRIMITIVE_CYLINDER:
					pcl::ModelCoefficients::Ptr corrected_coefs_primitive(new pcl::ModelCoefficients);
					correctCylShape(*corrected_coefs_primitive, *coefficients_primitive, *cloud_primitive);
					viewer.addCylinder(*corrected_coefs_primitive, "cylinder");

					// Plot cylinder longitudinal axis //PointT
					PointT point_on_axis((*coefficients_primitive).values[0], (*coefficients_primitive).values[1], (*coefficients_primitive).values[2]);
					PointT axis_direction(point_on_axis.x + (*coefficients_primitive).values[3], point_on_axis.y + (*coefficients_primitive).values[4], point_on_axis.z + (*coefficients_primitive).values[5]);
					PointT cam_origin(0.0, 0.0, 0.0);
					PointT axis_projection((*coefficients_primitive).values[3], (*coefficients_primitive).values[4], 0.0);

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

	#endif	// PRIMITIVE_MODEL
	#endif //DEBUG
					break;
				}
				break;

			case michelangelo::SEG_LCCP:

				// Convert the filtered cloud XYZ to XYZRGBA
				pcl::copyPointCloud(*cloud_filtered, *cloud_filtered_rgba);				

				/// Preparation of Input: Supervoxel Oversegmentation
				pcl::SupervoxelClustering<pcl::PointXYZRGBA> supervox(voxel_resolution, seed_resolution);
				supervox.setUseSingleCameraTransform(use_single_cam_transform);
				supervox.setInputCloud(cloud_filtered_rgba);
				if (false)
					supervox.setNormalCloud(cloud_normals);
				supervox.setColorImportance(color_importance);
				supervox.setSpatialImportance(spatial_importance);
				supervox.setNormalImportance(normal_importance);
				std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> supervoxel_clusters;

				PCL_INFO("Extracting supervoxels\n");
				supervox.extract(supervoxel_clusters);
				PCL_INFO("FLAG");

				if (use_supervoxel_refinement){
					PCL_INFO("Refining supervoxels\n");
					supervox.refineSupervoxels(2, supervoxel_clusters);
				}
				std::stringstream temp;
				temp << "  Nr. Supervoxels: " << supervoxel_clusters.size() << "\n";
				PCL_INFO(temp.str().c_str());

				PCL_INFO("Getting supervoxel adjacency\n");
				std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
				supervox.getSupervoxelAdjacency(supervoxel_adjacency);

				/// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
				sv_centroid_normal_cloud = pcl::SupervoxelClustering<pcl::PointXYZRGBA>::makeSupervoxelNormalCloud(supervoxel_clusters);

				/// The Main Step: Perform LCCPSegmentation
				PCL_INFO("Starting Segmentation\n");
				lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
				lccp.setSanityCheck(use_sanity_criterion);
				lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
				lccp.setKFactor(k_factor);
				lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
				lccp.setMinSegmentSize(min_segment_size);
				lccp.segment();

				PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
				sv_labeled_cloud = supervox.getLabeledCloud();
				lccp_labeled_cloud = sv_labeled_cloud->makeShared();
				lccp.relabelCloud(*lccp_labeled_cloud);

				viewer.addPointCloud(lccp_labeled_cloud, "maincloud");
				break;
		}
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
			//std::cout << "  stream " << profile.stream_name() << " " << profile.stream_type() << " " << profile.format() << " " << " " << profile.fps() << std::endl;
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