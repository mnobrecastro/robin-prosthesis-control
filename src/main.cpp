#include <ctime>
#include <array>
#include <thread>
#include <omp.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
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
#define FITMODEL 1

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

enum MovAvg { DEFAULT, EXPONENTIAL};

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points&);
float dotProduct(pcl::PointXYZ, pcl::PointXYZ);
float normPointT(pcl::PointXYZ);
int trimPointCloud(const pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&, const std::array<std::array<float, 2>, 3>, char, float);
void downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&, int);
std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<PointT>&, pcl::PointXYZ, pcl::PointXYZ);
std::array<float, 6> getPointCloudBoundaries(const pcl::PointCloud<PointT>&);
std::vector<int> getCentroidsLCCP(const pcl::PointCloud<pcl::PointXYZL>&, std::vector<uint32_t>&, std::vector<std::array<float, 3>>&);
uint32_t selCentroidLCCP(const std::vector<uint32_t>&, const std::vector<std::array<float, 3>>&);
void getLabeledCloudLCCP(const pcl::PointCloud<pcl::PointXYZL>&, pcl::PointCloud<pcl::PointXYZ>&, uint32_t);
float moving_average(float, std::list<float>&, int, MovAvg);
void correctLineShape(pcl::ModelCoefficients&, const pcl::PointCloud<PointT>&);
void correctCircleShape(pcl::ModelCoefficients&);
void correctCylShape(pcl::ModelCoefficients&, const pcl::ModelCoefficients&, const pcl::PointCloud<PointT>&);

namespace michelangelo {
	enum Primitive {
		PRIMITIVE_CUBE,
		PRIMITIVE_CYLINDER,
		PRIMITIVE_SPHERE
	};

	enum Method {
		NONE,
		HEURISTIC,
		SEGMENTATION
	};

	enum Heuristic {
		HEU_NONE,
		HEU_FAN,
		HEU_CROSS,
	};

	enum Segmentation {
		SEG_NONE,
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
			case michelangelo::PRIMITIVE_CUBE:
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
	bool RENDER(true);

	/*RawCross*/
	michelangelo::Method method(michelangelo::HEURISTIC);
	michelangelo::Heuristic heumethod(michelangelo::HEU_CROSS);
	michelangelo::Segmentation segmethod(michelangelo::SEG_NONE);
	unsigned int MIN_POINTS_PROCEED(50);
	bool DISPARITY(false);
	bool FILTER(true);
	bool FILT_DFLT(false);
	char TRIM_TYPE('+');
	float TRIM_WIDTH(0.010);//0.010
	bool DOWNSAMPLING(true); //'false'->rawdata slows down.
	bool DNSP_DFLT(true);
	unsigned int N_SAMPLE(2000);
	float MIN_SAMP_DIST(0.002);//0.002f
	unsigned int RANSAC_MAX_IT(100);
	bool STRIP_MODEL_LINE(true);
	int MOVING_AVG_SIZE(50);

	/*Disparity*/
	/*michelangelo::Segmentation segmethod(michelangelo::SEG_NONE);
	bool DISPARITY(true);
	bool FILTER(true);
	bool FILT_DFLT(false);
	char TRIM_TYPE('o');
	float TRIM_WIDTH(0.010);
	bool DOWNSAMPLING(true);*/

	/*LCCP*/
	/*michelangelo::Method method(michelangelo::SEGMENTATION);
	michelangelo::Heuristic heumethod;
	michelangelo::Segmentation segmethod(michelangelo::SEG_LCCP);
	unsigned int MIN_POINTS_PROCEED(100);
	bool DISPARITY(false);
	bool FILTER(true);
	bool FILT_DFLT(false);
	char TRIM_TYPE('o');
	float TRIM_WIDTH(0.010);
	bool DOWNSAMPLING(true);
	bool DNSP_DFLT(true);
	float MIN_SAMP_DIST(0.002);//0.002f
	unsigned int N_SAMPLE(2000);
	unsigned int RANSAC_MAX_IT(200);
	bool STRIP_MODEL_LINE;*/

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
	std::array<std::array<float, 2>, 3> filter_lims;

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

	rs2::pipeline pipe;
	rs2::config cfg;
	std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	std::cout << "Opening pipeline for " << serial_number << std::endl;
	cfg.enable_device(serial_number);	
	if (!DISPARITY) {
		//cfg.enable_stream(RS2_STREAM_DEPTH, 256, 144, RS2_FORMAT_Z16, 90); //works fine!
		cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 90); //works fine!
		//cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
		////cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
	} else {
		//cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
		cfg.enable_stream(RS2_STREAM_DEPTH, 848, 100, RS2_FORMAT_Z16, 100); // USB3.0 only!
	}
	//cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

	auto advanced_dev = dev.as<rs400::advanced_mode>();
	STDepthTableControl depth_table = advanced_dev.get_depth_table();	
	if (!DISPARITY) {
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

	if (!DISPARITY) {
		filter_lims[0] = { -0.100, 0.100 };
		filter_lims[1] = { -0.100, 0.100 };
		filter_lims[2] = { 0.050, 0.200 }; // realsense depth neg z-axis (MinZ 0.110m)
	} else {
		filter_lims[0] = { -0.050, 0.050 };
		filter_lims[1] = { -0.050, 0.050 };
		filter_lims[2] = { 0.050, 0.200 }; // realsense depth neg z-axis (MinZ 0.110m)
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
		
	/*// Pointcloud objects
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_primitive(new pcl::PointCloud<PointT>());

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h((int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_in_color_h((int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_plane_color_h(20, 180, 20);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_segmented_color_h(20, 180, 20);
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
	pcl::SupervoxelClustering<pcl::PointXYZRGBA> supervox(voxel_resolution, seed_resolution);*/

	pcl::console::TicToc time;
	pcl::console::TicToc tloop;

	while (!viewer.wasStopped()) {

		tloop.tic();

		// -------------------
		// Pointcloud objects
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<PointT>::Ptr cloud_filt_vertical(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_filt_horizontal(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<PointT>::Ptr cloud_primitive(new pcl::PointCloud<PointT>());
		std::array<pcl::PointCloud<PointT>::Ptr,2> arr_cloud_prim_vertical;
		std::array<pcl::PointCloud<PointT>::Ptr,2> arr_cloud_prim_horizontal;
		std::array<pcl::PointCloud<PointT>::Ptr,3> arr_cloud_plane;

		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h((int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_in_color_h((int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_plane_color_h(20, 180, 20);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_segmented_color_h(20, 180, 20);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_primitive_color_h(180, 20, 20);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_prim_vertical_color_h(180, 20, 20);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_prim_horizontal_color_h(180, 20, 20);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> arr_cloud_cube_color_h[3] = {
			pcl::visualization::PointCloudColorHandlerCustom<PointT>(255, 0, 255), 
			pcl::visualization::PointCloudColorHandlerCustom<PointT>(255, 255, 0),
			pcl::visualization::PointCloudColorHandlerCustom<PointT>(0, 255, 255)
		};

		// PCL objects
		pcl::PassThrough<PointT> pass(true);
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		pcl::NormalEstimation<PointT, pcl::Normal> ne_vertical;
		pcl::NormalEstimation<PointT, pcl::Normal> ne_horizontal;
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
		pcl::SACSegmentation<PointT> seg_vertical;
		pcl::SACSegmentation<PointT> seg_horizontal;
		pcl::ExtractIndices<PointT> extract;
		//pcl::ExtractIndices<PointT> extract_vertical;
		//pcl::ExtractIndices<PointT> extract_horizontal;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		pcl::ExtractIndices<pcl::Normal> extract_normals_vertical;
		pcl::ExtractIndices<pcl::Normal> extract_normals_horizontal;
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
		pcl::search::KdTree<PointT>::Ptr tree_vertical(new pcl::search::KdTree<PointT>());
		pcl::search::KdTree<PointT>::Ptr tree_horizontal(new pcl::search::KdTree<PointT>());

		pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_primitive(new pcl::ModelCoefficients);
		std::array<pcl::ModelCoefficients::Ptr, 2> arr_coeffs_prim_vertical;
		std::array<pcl::ModelCoefficients::Ptr, 2> arr_coeffs_prim_horizontal;
		std::array<pcl::ModelCoefficients::Ptr, 3> arr_coeffs_plane;
		pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_primitive(new pcl::PointIndices);
		std::array<pcl::PointIndices::Ptr, 2> arr_inliers_prim_vertical;
		std::array<pcl::PointIndices::Ptr, 2> arr_inliers_prim_horizontal;
		std::array<pcl::PointIndices::Ptr, 3> arr_inls_plane;

		// PCL Primitive
		std::array<michelangelo::Primitive, 3> primitives = { michelangelo::PRIMITIVE_CUBE, michelangelo::PRIMITIVE_SPHERE, michelangelo::PRIMITIVE_CYLINDER };
		michelangelo::Primitive prim(primitives[0]);

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

		// -------------------

		if (RENDER) {
			viewer.removeAllShapes();
			viewer.removeAllPointClouds();
		}

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

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// CROPPING + DOWNSAMPLING
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		if (FILTER) {
			time.tic();
			// Build a passthrough filter to remove unwated points
			if (FILT_DFLT) {
				pass.setInputCloud(cloud);
				pass.setFilterFieldName("x");
				pass.setFilterLimits(filter_lims[0][0], filter_lims[0][1]);
				pass.filter(*cloud_filtered);
				//
				if (!DISPARITY) {
					pass.setInputCloud(cloud_filtered);
					pass.setFilterFieldName("y");
					pass.setFilterLimits(filter_lims[1][0], filter_lims[1][1]);
					pass.filter(*cloud_filtered);
					//
					pass.setInputCloud(cloud_filtered);
					pass.setFilterFieldName("z");
					pass.setFilterLimits(filter_lims[2][0], filter_lims[2][1]);
					pass.filter(*cloud_filtered);
				}
			} else {
				int valid = trimPointCloud(*cloud, *cloud_filtered, filter_lims, TRIM_TYPE, TRIM_WIDTH);
				if (valid == 1) {
					std::cerr << "Invalid ranges for trimming the pointcloud." << std::endl;
					return -1;
				}
			}
			std::cerr << "PointCloud after filtering: " << cloud_filtered->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;
		} else {
			pcl::copyPointCloud(*cloud, *cloud_filtered);
		}
				
		if (DOWNSAMPLING) {
			time.tic();
			// Downsampling the filtered point cloud
			if (DNSP_DFLT) {						
				pcl::ApproximateVoxelGrid<pcl::PointXYZ> dsfilt;
				dsfilt.setInputCloud(cloud_filtered);
				if (!DISPARITY) {
					dsfilt.setLeafSize(0.0025f, 0.0025f, 0.0025f); //0.005f
				} else {
					dsfilt.setLeafSize(MIN_SAMP_DIST, MIN_SAMP_DIST, MIN_SAMP_DIST);
				}
				dsfilt.filter(*cloud_filtered);
			} else {
				//pcl::PointCloud<PointT>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
				//downsamplePointCloud(*cloud_filtered, *cloud_ds, 100);
				//cloud_filtered->clear();
				//pcl::copyPointCloud(*cloud_ds, *cloud_filtered);

				pcl::RandomSample<pcl::PointXYZ> dsfilt;
				dsfilt.setInputCloud(cloud_filtered);
				dsfilt.setSample(N_SAMPLE);
				dsfilt.filter(*cloud_filtered);
			}
			std::cerr << "PointCloud after downsampling: " << cloud_filtered->width * cloud_filtered->height << "=" << cloud_filtered->points.size()
				<< " data points (in " << time.toc() << " ms)." << std::endl; //pcl::getFieldsList(*cloud_filtered)
		}

		if (RENDER) {
			// Render the filtered PointCloud
			cloud_filtered_in_color_h.setInputCloud(cloud_filtered);
			viewer.addPointCloud(cloud_filtered, cloud_filtered_in_color_h, "cloud_filtered_in", vp);
		}

		// Check whether the filtered PointCloud can proceed to the analysis
		if (cloud_filtered->points.size() < MIN_POINTS_PROCEED) {
			std::cerr << "* Not enough points to perform segmentation." << std::endl;
			std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
			continue;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// PRIMITIVE SEGMENTATION
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		size_t k_lines_vertical, k_lines_horizontal;
		int n_pts_vertical, n_pts_horizontal;
		int MAX_LINES_VERTICAL, MAX_LINES_HORIZONTAL;
		if (!STRIP_MODEL_LINE) {
			MAX_LINES_VERTICAL = 1, MAX_LINES_HORIZONTAL = 1;
		} else {
			MAX_LINES_VERTICAL = 2, MAX_LINES_HORIZONTAL = 2;
		}

		int vertical_idx(-1), horizontal_idx(-1);
		std::vector<std::array<float, 2>> bounds_vertical, bounds_horizontal;
		float cube_width(0.050), cube_height(0.050), cube_depth(0.050);
		std::list<float> save_cube_width, save_cube_height, save_cube_depth;

		switch (method) {
		case michelangelo::HEURISTIC:

			for (auto p : cloud_filtered->points) {
				//horizontal strip
				if (filter_lims[0][0] <= p.x && p.x <= filter_lims[0][1] &&
					-TRIM_WIDTH / 2 <= p.y && p.y <= TRIM_WIDTH / 2 &&
					filter_lims[2][0] <= p.z && p.z <= filter_lims[2][1]) {
					cloud_filt_horizontal->push_back(pcl::PointXYZ(p.x, 0.0, p.z));
					//vertical strip
				}
				else if (-TRIM_WIDTH / 2 <= p.x && p.x <= TRIM_WIDTH / 2 &&
					filter_lims[1][0] <= p.y && p.y <= filter_lims[1][1] &&
					filter_lims[2][0] <= p.z && p.z <= filter_lims[2][1]) {
					cloud_filt_vertical->push_back(pcl::PointXYZ(0.0, p.y, p.z));
				}
			}

			time.tic();

			// VERTICAL STRIP

			// Create the segmentation object for primitive segmentation			
			seg_vertical.setMethodType(pcl::SAC_RANSAC); //RMSAC
			seg_vertical.setOptimizeCoefficients(true);
			if (!STRIP_MODEL_LINE) {
				seg_vertical.setModelType(pcl::SACMODEL_CIRCLE3D);
				seg_vertical.setMaxIterations(RANSAC_MAX_IT);
				seg_vertical.setDistanceThreshold(0.001);
				seg_vertical.setRadiusLimits(0.005, 0.050);//0.05
			} else {
				seg_vertical.setModelType(pcl::SACMODEL_LINE);
				seg_vertical.setMaxIterations(RANSAC_MAX_IT);
				seg_vertical.setDistanceThreshold(0.001);
			}

			k_lines_vertical = 0; n_pts_vertical = cloud_filt_vertical->points.size();
			while (cloud_filt_vertical->points.size() > 0.3 * n_pts_vertical && k_lines_vertical < MAX_LINES_VERTICAL) {

				pcl::PointCloud<PointT>::Ptr cloud_prim_line(new pcl::PointCloud<PointT>());
				pcl::PointIndices::Ptr inliers_prim_line(new pcl::PointIndices);
				pcl::ModelCoefficients::Ptr coefficients_prim_line(new pcl::ModelCoefficients);
				
				// Segment the largest line component from the remaining cloud
				seg_vertical.setInputCloud(cloud_filt_vertical);
				seg_vertical.segment(*inliers_prim_line, *coefficients_prim_line);
				if (inliers_prim_line->indices.size() == 0) {
					std::cout << "Could not estimate a line model for the vertical strip." << std::endl;
					break;
				}

				std::cout << "Vertical prim line #" << k_lines_vertical + 1 << ": " << inliers_prim_line->indices.size() << "/" << cloud_filt_vertical->points.size() << " points." << std::endl;

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZ> extract_vertical;
				extract_vertical.setInputCloud(cloud_filt_vertical);
				extract_vertical.setIndices(inliers_prim_line);
				extract_vertical.setNegative(false);
				extract_vertical.filter(*cloud_prim_line);

				// Remove the line inliers, extract the rest
				extract_vertical.setNegative(true);
				extract_vertical.filter(*cloud_filt_vertical);

				arr_cloud_prim_vertical[k_lines_vertical] = cloud_prim_line;
				arr_inliers_prim_vertical[k_lines_vertical] = inliers_prim_line;
				arr_coeffs_prim_vertical[k_lines_vertical] = coefficients_prim_line;

				*cloud_primitive += *cloud_prim_line;

				++k_lines_vertical;
			}

			// HORIZONTAL STRIP

			// Create the segmentation object for primitive segmentation			
			seg_horizontal.setMethodType(pcl::SAC_RANSAC); //RMSAC
			seg_horizontal.setOptimizeCoefficients(true);
			if (!STRIP_MODEL_LINE) {
				seg_horizontal.setModelType(pcl::SACMODEL_CIRCLE3D);
				seg_horizontal.setMaxIterations(RANSAC_MAX_IT);
				seg_horizontal.setDistanceThreshold(0.001);
				seg_horizontal.setRadiusLimits(0.005, 0.05);//0.05
			} else {
				seg_horizontal.setModelType(pcl::SACMODEL_LINE);
				seg_horizontal.setMaxIterations(RANSAC_MAX_IT);
				seg_horizontal.setDistanceThreshold(0.001);
			}

			k_lines_horizontal = 0; n_pts_horizontal = cloud_filt_horizontal->points.size();
			while (cloud_filt_horizontal->points.size() > 0.3 * n_pts_horizontal && k_lines_horizontal < MAX_LINES_HORIZONTAL) {

				pcl::PointCloud<PointT>::Ptr cloud_prim_line(new pcl::PointCloud<PointT>());
				pcl::PointIndices::Ptr inliers_prim_line(new pcl::PointIndices);
				pcl::ModelCoefficients::Ptr coefficients_prim_line(new pcl::ModelCoefficients);

				// Segment the largest line component from the remaining cloud
				seg_horizontal.setInputCloud(cloud_filt_horizontal);
				seg_horizontal.segment(*inliers_prim_line, *coefficients_prim_line);
				if (inliers_prim_line->indices.size() == 0) {
					std::cout << "Could not estimate a line model for the vertical strip." << std::endl;
					break;
				}

				std::cout << "Horizontal prim line #" << k_lines_horizontal + 1 << ": " << inliers_prim_line->indices.size() << "/" << cloud_filt_horizontal->points.size() << " points." << std::endl;

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZ> extract_horizontal;
				extract_horizontal.setInputCloud(cloud_filt_horizontal);
				extract_horizontal.setIndices(inliers_prim_line);
				extract_horizontal.setNegative(false);
				extract_horizontal.filter(*cloud_prim_line);

				// Remove the planar inliers, extract the rest
				extract_horizontal.setNegative(true);
				extract_horizontal.filter(*cloud_filt_horizontal);

				arr_cloud_prim_horizontal[k_lines_horizontal] = cloud_prim_line;
				arr_inliers_prim_horizontal[k_lines_horizontal] = inliers_prim_line;
				arr_coeffs_prim_horizontal[k_lines_horizontal] = coefficients_prim_line;

				*cloud_primitive += *cloud_prim_line;

				++k_lines_horizontal;
			}

			////// RENDERING

			// Check if line primitives were found
			if (k_lines_vertical == 0 || k_lines_horizontal == 0) {	//&&		
				std::cerr << "\tCan't find the primitive." << std::endl;
				std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
				continue;
			}
			else {
				std::cerr << "PointCloud PRIMITIVE: " << cloud_primitive->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;
			}

			// Render the line primitives PointCloud and 3-D shape
			if (RENDER) {
				// prim_vertical point cloud is red
				for (int i(0); i < k_lines_vertical; ++i) {
					cloud_prim_vertical_color_h.setInputCloud(arr_cloud_prim_vertical[i]);
					viewer.addPointCloud(arr_cloud_prim_vertical[i], cloud_prim_vertical_color_h, "cloud_prim_vertical_" + std::to_string(i), vp);

					if (arr_coeffs_prim_vertical[i]->values.size() == 7) {
						correctCircleShape(*arr_coeffs_prim_vertical[i]);
						viewer.addCylinder(*arr_coeffs_prim_vertical[i], "vertical_" + std::to_string(i));
					}
					else {
						correctLineShape(*arr_coeffs_prim_vertical[i], *arr_cloud_prim_vertical[i]);
						viewer.addLine(*arr_coeffs_prim_vertical[i], "vertical_" + std::to_string(i));
					}
				}

				// prim_horizontal point cloud is red
				for (int i(0); i < k_lines_horizontal; ++i) {
					cloud_prim_horizontal_color_h.setInputCloud(arr_cloud_prim_horizontal[i]);
					viewer.addPointCloud(arr_cloud_prim_horizontal[i], cloud_prim_horizontal_color_h, "cloud_prim_horizontal_" + std::to_string(i), vp);

					if (arr_coeffs_prim_horizontal[i]->values.size() == 7) {
						correctCircleShape(*arr_coeffs_prim_horizontal[i]);
						viewer.addCylinder(*arr_coeffs_prim_horizontal[i], "horizontal_" + std::to_string(i));
					}
					else {
						correctLineShape(*arr_coeffs_prim_horizontal[i], *arr_cloud_prim_horizontal[i]);
						viewer.addLine(*arr_coeffs_prim_horizontal[i], "horizontal_" + std::to_string(i));
					}
				}
			}

			// Computing the boundaries of the line primitives
			//   [-]
			//   [-]
			// [-----][--------]
			//   [-]
			//   [-]
			//   [-]
			//std::vector<std::array<float, 2>> bounds_vertical;
			for (int i(0); i < k_lines_vertical; ++i) {
				std::array<float, 6> bounds_temp(getPointCloudBoundaries(*arr_cloud_prim_vertical[i]));
				bounds_vertical.push_back({ bounds_temp[2] ,bounds_temp[3] });
			}
			//std::vector<std::array<float, 2>> bounds_horizontal;
			for (int i(0); i < k_lines_horizontal; ++i) {
				std::array<float, 6> bounds_temp(getPointCloudBoundaries(*arr_cloud_prim_horizontal[i]));
				bounds_horizontal.push_back({ bounds_temp[0] ,bounds_temp[1] });
			}

			// Finding the front cube_face spaned by '+'
			//int vertical_idx(-1), horizontal_idx(-1);
			for (int k1(0); k1 < bounds_vertical.size(); ++k1) {
				for (int k2(0); k2 < bounds_horizontal.size(); ++k2) {
					if (bounds_vertical[k1][0] <= 0.0 && 0.0 < bounds_vertical[k1][1] && bounds_horizontal[k2][0] <= 0.0 && 0.0 < bounds_horizontal[k2][1]) {
						vertical_idx = k1;
						horizontal_idx = k2;
						break;
					}
				}
			}

			//float cube_width(0.050), cube_height(0.050), cube_depth(0.050);
			if (vertical_idx == -1 || horizontal_idx == -1) { //&&
				continue;
			} else {
				std::cout << "vert_idx: " << vertical_idx  << " hori_idx: " << horizontal_idx << std::endl;

				// Find the centroid of the points in the line primitive
				Eigen::Vector3f v_point(
					arr_coeffs_prim_vertical[vertical_idx]->values[0],
					arr_coeffs_prim_vertical[vertical_idx]->values[1],
					arr_coeffs_prim_vertical[vertical_idx]->values[2]
					);
				Eigen::Vector3f v_dir(
					arr_coeffs_prim_vertical[vertical_idx]->values[3],
					arr_coeffs_prim_vertical[vertical_idx]->values[4],
					arr_coeffs_prim_vertical[vertical_idx]->values[5]
				);
				Eigen::Vector3f v_center(v_point.x() + v_dir.x() * 0.5, v_point.y() + v_dir.y() * 0.5, v_point.z() + v_dir.z() * 0.5);
				
				Eigen::Vector3f h_point(
					arr_coeffs_prim_horizontal[horizontal_idx]->values[0],
					arr_coeffs_prim_horizontal[horizontal_idx]->values[1],
					arr_coeffs_prim_horizontal[horizontal_idx]->values[2]
				);
				Eigen::Vector3f h_dir(
					arr_coeffs_prim_horizontal[horizontal_idx]->values[3],
					arr_coeffs_prim_horizontal[horizontal_idx]->values[4],
					arr_coeffs_prim_horizontal[horizontal_idx]->values[5]
				);
				Eigen::Vector3f h_center(h_point.x() + h_dir.x() * 0.5, h_point.y() + h_dir.y() * 0.5, h_point.z() + h_dir.z() * 0.5);

				// Find the cube_face center
				Eigen::Vector3f vec(v_center.x() - h_center.x(), v_center.y() - h_center.y(), v_center.z() - h_center.z());
				Eigen::Vector3f face_center(
					h_center.x() + v_dir.dot(vec) / std::pow(v_dir.norm(), 2) * v_dir.x(),
					h_center.y() + v_dir.dot(vec) / std::pow(v_dir.norm(), 2) * v_dir.y(),
					h_center.z() + v_dir.dot(vec) / std::pow(v_dir.norm(), 2) * v_dir.z()
				);
				Eigen::Vector3f cube_center;

				// Cube primitive parameters
				cube_width = h_dir.norm();
				cube_width = moving_average(cube_width, save_cube_width, MOVING_AVG_SIZE, EXPONENTIAL);
				cube_height = v_dir.norm();
				cube_height = moving_average(cube_height, save_cube_height, MOVING_AVG_SIZE, EXPONENTIAL);

				Eigen::Vector3f face_normal(h_dir.cross(v_dir));
				face_normal.normalize();
				if (face_normal.z() < 0.0) {
					face_normal[0] = -face_normal.x();
					face_normal[1] = -face_normal.y();
					face_normal[2] = -face_normal.z();
				}
					
				if (k_lines_vertical == 1 && k_lines_horizontal == 2) {
					Eigen::Vector3f d_dir(
						arr_coeffs_prim_horizontal[int(1) - horizontal_idx]->values[3],
						arr_coeffs_prim_horizontal[int(1) - horizontal_idx]->values[4],
						arr_coeffs_prim_horizontal[int(1) - horizontal_idx]->values[5]
					);
					cube_depth = d_dir.norm();
				} else if (k_lines_vertical == 2 && k_lines_horizontal == 1) {
					Eigen::Vector3f d_dir(
						arr_coeffs_prim_vertical[int(1) - vertical_idx]->values[3],
						arr_coeffs_prim_vertical[int(1) - vertical_idx]->values[4],
						arr_coeffs_prim_vertical[int(1) - vertical_idx]->values[5]
					);
					cube_depth = d_dir.norm();
				} else if (k_lines_vertical == 2 && k_lines_horizontal == 2) {
					Eigen::Vector3f d_dir( // Has to be reviewed base on weight of the number of points
						arr_coeffs_prim_horizontal[int(1) - horizontal_idx]->values[3],
						arr_coeffs_prim_horizontal[int(1) - horizontal_idx]->values[4],
						arr_coeffs_prim_horizontal[int(1) - horizontal_idx]->values[5]
					);
					cube_depth = d_dir.norm();
				}
				cube_depth = moving_average(cube_depth, save_cube_depth, MOVING_AVG_SIZE, EXPONENTIAL);
				cube_center = face_center + face_normal * cube_depth / 2;

				Eigen::Quaternionf quat;
				quat.setFromTwoVectors(Eigen::Vector3f(0.0, 0.0, 1.0), face_normal);

				//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
				coefficients_primitive->values.push_back(cube_center.x()); //Tx
				coefficients_primitive->values.push_back(cube_center.y()); //Ty
				coefficients_primitive->values.push_back(cube_center.z()); //Tz
				coefficients_primitive->values.push_back(quat.x()); //Qx
				coefficients_primitive->values.push_back(quat.y()); //Qy
				coefficients_primitive->values.push_back(quat.z()); //Qz
				coefficients_primitive->values.push_back(quat.w()); //Qw
				coefficients_primitive->values.push_back(cube_width); //width
				coefficients_primitive->values.push_back(cube_height); //height
				coefficients_primitive->values.push_back(cube_depth); //depth

				if (RENDER) {
					viewer.addCube(*coefficients_primitive, "cube");
				}
				//}

				/*switch (prim) {
				case michelangelo::PRIMITIVE_CUBE:
					if (RENDER) {
						viewer.addCube(*coefficients_primitive, "cube");
					};
					break;
				}*/
			}	
			break;

		case michelangelo::SEGMENTATION:

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);
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
				case michelangelo::PRIMITIVE_CUBE:
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
				seg.setDistanceThreshold(0.05); //0.05
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
					std::cerr << "\tCan't find the solid component." << std::endl;
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

				if (RENDER) {
					// ICP aligned point cloud is red
					cloud_primitive_color_h.setInputCloud(cloud_primitive);
					viewer.addPointCloud(cloud_primitive, cloud_primitive_color_h, "cloud_primitive", vp);
				}

				// Plot primitive shape
				switch (prim) {
				case michelangelo::PRIMITIVE_CUBE:
					if (RENDER) {
						//viewer.addCylinder(*corrected_coefs_cylinder, "cylinder");
					}
					break;
				case michelangelo::PRIMITIVE_SPHERE:
					if (RENDER) {
						viewer.addSphere(*coefficients_primitive, "sphere");
					}
					break;
				case michelangelo::PRIMITIVE_CYLINDER:
					pcl::ModelCoefficients::Ptr corrected_coefs_primitive(new pcl::ModelCoefficients);
					correctCylShape(*corrected_coefs_primitive, *coefficients_primitive, *cloud_primitive);					

					// Plot cylinder longitudinal axis //PointT
					PointT point_on_axis((*coefficients_primitive).values[0], (*coefficients_primitive).values[1], (*coefficients_primitive).values[2]);
					PointT axis_direction(point_on_axis.x + (*coefficients_primitive).values[3], point_on_axis.y + (*coefficients_primitive).values[4], point_on_axis.z + (*coefficients_primitive).values[5]);
					PointT cam_origin(0.0, 0.0, 0.0);
					PointT axis_projection((*coefficients_primitive).values[3], (*coefficients_primitive).values[4], 0.0);

					float camAngle(90.0 * M_PI / 180);
					michelangelo::correctAngle(axis_projection, camAngle);
					
					if (RENDER) {
						viewer.addCylinder(*corrected_coefs_primitive, "cylinder");
						viewer.addLine(cam_origin, axis_projection, "line");
					}

					// Calculate the angular difference			
					//float dTheta(M_PI - std::atan2(axis_projection.y, axis_projection.x));
					float dTheta(michelangelo::readAngle(axis_projection));
					std::string action(michelangelo::setAction(dTheta));
					std::cout << "* Current angle: " << dTheta * 180 / M_PI << "\tAction: " << action << std::endl;

#endif	// PRIMITIVE_MODEL
					break;
				}

				break;

			case michelangelo::SEG_LCCP:

				time.tic();
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

				if (use_supervoxel_refinement) {
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
				//sv_centroid_normal_cloud = pcl::SupervoxelClustering<pcl::PointXYZRGBA>::makeSupervoxelNormalCloud(supervoxel_clusters);

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

				std::vector<int> segments;
				std::vector<uint32_t> labels;
				std::vector<std::array<float, 3>> centroids;
				std::vector<int> label_count = getCentroidsLCCP(*lccp_labeled_cloud, labels, centroids);
				std::cout << "  Nr. Segments: " << labels.size() << std::endl;

				bool PLOT_CENTROIDS(false);
				if (PLOT_CENTROIDS) {
					std::string id;
					for (int i(0); i < centroids.size(); ++i) {
						pcl::PointXYZ pFUL(centroids[i][0] - 0.005, centroids[i][1] - 0.005, centroids[i][2] - 0.005);
						pcl::PointXYZ pFUR(centroids[i][0] + 0.005, centroids[i][1] - 0.005, centroids[i][2] - 0.005);
						pcl::PointXYZ pFDL(centroids[i][0] - 0.005, centroids[i][1] + 0.005, centroids[i][2] - 0.005);
						pcl::PointXYZ pFDR(centroids[i][0] + 0.005, centroids[i][1] + 0.005, centroids[i][2] - 0.005);

						pcl::PointXYZ pBUL(centroids[i][0] - 0.005, centroids[i][1] - 0.005, centroids[i][2] + 0.005);
						pcl::PointXYZ pBUR(centroids[i][0] + 0.005, centroids[i][1] - 0.005, centroids[i][2] + 0.005);
						pcl::PointXYZ pBDL(centroids[i][0] - 0.005, centroids[i][1] + 0.005, centroids[i][2] + 0.005);
						pcl::PointXYZ pBDR(centroids[i][0] + 0.005, centroids[i][1] + 0.005, centroids[i][2] + 0.005);

						viewer.addLine<pcl::PointXYZ>(pFUL, pFUR, int(255) * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "FU", vp);
						viewer.addLine<pcl::PointXYZ>(pFUR, pFDR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "FR", vp);
						viewer.addLine<pcl::PointXYZ>(pFDR, pFDL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "FD", vp);
						viewer.addLine<pcl::PointXYZ>(pFDL, pFUL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "FL", vp);

						viewer.addLine<pcl::PointXYZ>(pBUL, pBUR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "BU", vp);
						viewer.addLine<pcl::PointXYZ>(pBUR, pBDR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "BR", vp);
						viewer.addLine<pcl::PointXYZ>(pBDR, pBDL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "BD", vp);
						viewer.addLine<pcl::PointXYZ>(pBDL, pBUL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "BL", vp);

						viewer.addLine<pcl::PointXYZ>(pBUL, pFUL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "LU", vp);
						viewer.addLine<pcl::PointXYZ>(pBDL, pFDL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "LD", vp);
						viewer.addLine<pcl::PointXYZ>(pBUR, pFUR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "RU", vp);
						viewer.addLine<pcl::PointXYZ>(pBDR, pFDR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "RD", vp);
#ifdef DEBUG
						std::cout << "    l: " << labels[i] <<
							" pts: " << label_count[i] <<
							" x: " << centroids[i][0] <<
							" y: " << centroids[i][1] <<
							" z: " << centroids[i][2] << std::endl;
#endif //DEBUG
					}
				}
				uint32_t lbl = selCentroidLCCP(labels, centroids);
				getLabeledCloudLCCP(*lccp_labeled_cloud, *cloud_segmented, lbl);

				cloud_segmented_color_h.setInputCloud(cloud_segmented);
				viewer.addPointCloud(cloud_segmented, cloud_segmented_color_h, "cloud_segmented", vp);

				cloud_filtered->clear();
				pcl::copyPointCloud(*cloud_segmented, *cloud_filtered);
				std::cout << "PointCloud after LCCP: [" << lbl << "] " << cloud_segmented->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;
				break;
			}
#if FITMODEL
			// Estimate point normals
			std::cout << "Computing normals...";
			ne.setSearchMethod(tree);
			ne.setInputCloud(cloud_filtered);
			ne.setKSearch(50);//50
			//ne.setRadiusSearch(0.01); //New
			ne.compute(*cloud_normals);
			std::cout << " done." << std::endl;
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

			size_t n_planes(0); int n_pts(cloud_filtered->points.size());

			time.tic();
			switch (prim) {
			case michelangelo::PRIMITIVE_CUBE:
				// Create the segmentation object for primitive segmentation and set all the parameters
				seg.setOptimizeCoefficients(true);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setModelType(pcl::SACMODEL_PLANE); // SACMODEL_PARALLEL_PLANE , SACMODEL_NORMAL_PLANE , SACMODEL_NORMAL_PARALLEL_PLANE
				seg.setNormalDistanceWeight(0.1);
				seg.setMaxIterations(RANSAC_MAX_IT);
				seg.setDistanceThreshold(0.005);
				//seg.setRadiusLimits(0.005, 0.050);
				//seg.setInputCloud(cloud_filtered);
				//seg.setInputNormals(cloud_normals);
				//seg.setAxis();
				//seg.setEpsAngle();				
				
				while (cloud_filtered->points.size() > 0.01 * n_pts && n_planes < 3) {

					pcl::PointCloud<PointT>::Ptr cloud_cube_face(new pcl::PointCloud<PointT>());
					pcl::PointIndices::Ptr inliers_cube_face(new pcl::PointIndices);
					pcl::ModelCoefficients::Ptr coefficients_cube_face(new pcl::ModelCoefficients);
					
					// Segment the largest planar component from the remaining cloud
					seg.setInputCloud(cloud_filtered);
					seg.setInputNormals(cloud_normals);
					seg.segment(*inliers_cube_face, *coefficients_cube_face);
					if (inliers_cube_face->indices.size() == 0) {
						std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
						break;
					}

					std::cout << "Cube plane #" << n_planes + 1 << ": " << inliers_cube_face->indices.size() << "/" << cloud_filtered->points.size() << " points." << std::endl;
					
					// Extract the planar inliers from the input cloud
					pcl::ExtractIndices<pcl::PointXYZ> extract;
					extract.setInputCloud(cloud_filtered);
					extract.setIndices(inliers_cube_face);
					extract.setNegative(false);
					extract.filter(*cloud_cube_face);

					// Remove the planar inliers, extract the rest
					extract.setNegative(true);
					extract.filter(*cloud_filtered);

					std::cout << "Remaining filtered PointCloud: " << cloud_filtered->points.size() << " data points." << std::endl;

					// Remove the planar inliers from normals
					pcl::ExtractIndices<pcl::Normal> extract_normals;
					extract_normals.setInputCloud(cloud_normals);
					extract_normals.setIndices(inliers_cube_face);
					extract_normals.setNegative(true);
					extract_normals.filter(*cloud_normals);

					arr_cloud_plane[n_planes] = cloud_cube_face;
					arr_inls_plane[n_planes] = inliers_cube_face;
					arr_coeffs_plane[n_planes] = coefficients_cube_face;

					*cloud_primitive += *cloud_cube_face;

					++n_planes;
				}

				if (n_planes != 0) {
					//pcl::PointCloud<PointT>::Ptr cloud_eigen(new pcl::PointCloud<PointT>());
					pcl::PCA<pcl::PointXYZ> pca;
					pca.setInputCloud(arr_cloud_plane[0]);
					Eigen::Matrix3f eigenvecs(pca.getEigenVectors());
					Eigen::Vector3f eigenvals(pca.getEigenValues());
					Eigen::MatrixXf coeffs(pca.getCoefficients());
					Eigen::Vector4f mean(pca.getMean());					

					//Plane coefficients [normal_x normal_y normal_z d]
					pcl::PointXYZ face_center(mean.x(), mean.y(), mean.z());
					pcl::PointXYZ cube_center(face_center.x + arr_coeffs_plane[0]->values[0] * 0.025, face_center.y + arr_coeffs_plane[0]->values[1] * 0.025, face_center.z + arr_coeffs_plane[0]->values[2] * 0.025);
					Eigen::Quaternionf quat;
					Eigen::Vector3f v1(0.0, 0.0, 1.0);
					Eigen::Vector3f v2(arr_coeffs_plane[0]->values[0], arr_coeffs_plane[0]->values[1], arr_coeffs_plane[0]->values[2]);
					quat.setFromTwoVectors(v1, v2);

					//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
					coefficients_primitive->values.push_back(cube_center.x); //Tx
					coefficients_primitive->values.push_back(cube_center.y); //Ty
					coefficients_primitive->values.push_back(cube_center.z); //Tz
					coefficients_primitive->values.push_back(quat.x()); //Qx
					coefficients_primitive->values.push_back(quat.y()); //Qy
					coefficients_primitive->values.push_back(quat.z()); //Qz
					coefficients_primitive->values.push_back(quat.w()); //Qw
					coefficients_primitive->values.push_back(0.050); //width
					coefficients_primitive->values.push_back(0.050); //height
					coefficients_primitive->values.push_back(0.050); //depth


					/*switch (n_planes) {
					case 1:
						//Plane coefficients [normal_x normal_y normal_z d]
						pca.setInputCloud(arr_cloud_plane[0]);
						eigenvecs.data( = pca.getEigenVectors();
						coeffs = pca.getCoefficients();
						mean = pca.getMean();
						//eigenvals = pca.getEigenValues();

						// Plot cubeface normal
						pcl::PointXYZ face_center(mean.x(), mean.y(), mean.z());
						pcl::PointXYZ face_normal(face_center.x + arr_coeffs_plane[0]->values[0] * 0.025, face_center.y + arr_coeffs_plane[0]->values[1] * 0.025, face_center.z + arr_coeffs_plane[0]->values[2] * 0.025);
						//viewer.addLine(face_center, face_normal, "normal_cubeface_" + std::to_string(1));
						
						//pca.project(arr_cloud_plane[0], *cloud_eigen);				

						//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
						coefficients_primitive->values.push_back(face_normal.x); //Tx
						coefficients_primitive->values.push_back(face_normal.y); //Ty
						coefficients_primitive->values.push_back(face_normal.z); //Tz
						coefficients_primitive->values.push_back(0.0); //Qx
						coefficients_primitive->values.push_back(0.0); //Qy
						coefficients_primitive->values.push_back(0.0); //Qz
						coefficients_primitive->values.push_back(1.0); //Qw
						coefficients_primitive->values.push_back(0.050); //width
						coefficients_primitive->values.push_back(0.050); //height
						coefficients_primitive->values.push_back(0.050); //depth
						break;

					case 2:

						//cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
						//coefficients_primitive->values.push_back();
						break;

					case 3:

						//cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
						//coefficients_primitive->values.push_back();
						break;
					}*/
				}else {
					continue;
				}


				break;

			case michelangelo::PRIMITIVE_SPHERE:
				// Create the segmentation object for primitive segmentation and set all the parameters
				seg.setOptimizeCoefficients(true);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setModelType(pcl::SACMODEL_SPHERE);
				seg.setNormalDistanceWeight(0.1);
				seg.setMaxIterations(RANSAC_MAX_IT);
				seg.setDistanceThreshold(0.05);
				seg.setRadiusLimits(0.005, 0.050);
				seg.setInputCloud(cloud_filtered);
				seg.setInputNormals(cloud_normals);

				// Obtain the primitive inliers and coefficients
				seg.segment(*inliers_primitive, *coefficients_primitive);
				//std::cerr << "primitive inliers: " << *inliers_primitive << std::endl;
				//std::cerr << "primitive coefficients: " << *coefficients_primitive << std::endl;

				// Save the primitive inliers
				extract.setInputCloud(cloud_filtered);
				extract.setIndices(inliers_primitive);
				extract.setNegative(false);
				//pcl::PointCloud<PointT>::Ptr cloud_primitive(new pcl::PointCloud<PointT>());
				extract.filter(*cloud_primitive);
				break;
			case michelangelo::PRIMITIVE_CYLINDER:
				// Create the segmentation object for primitive segmentation and set all the parameters
				seg.setOptimizeCoefficients(true);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setModelType(pcl::SACMODEL_CYLINDER);
				seg.setNormalDistanceWeight(0.1);
				seg.setMaxIterations(RANSAC_MAX_IT);
				seg.setDistanceThreshold(0.05);
				seg.setRadiusLimits(0.005, 0.050);
				seg.setInputCloud(cloud_filtered);
				seg.setInputNormals(cloud_normals);

				// Obtain the primitive inliers and coefficients
				seg.segment(*inliers_primitive, *coefficients_primitive);
				//std::cerr << "primitive inliers: " << *inliers_primitive << std::endl;
				//std::cerr << "primitive coefficients: " << *coefficients_primitive << std::endl;

				// Save the primitive inliers
				extract.setInputCloud(cloud_filtered);
				extract.setIndices(inliers_primitive);
				extract.setNegative(false);
				//pcl::PointCloud<PointT>::Ptr cloud_primitive(new pcl::PointCloud<PointT>());
				extract.filter(*cloud_primitive);
				break;
			}

			if (cloud_primitive->points.empty()) {
				std::cerr << "\tCan't find the cylindrical component." << std::endl;
				std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
				continue;
			} else {
				std::cerr << "PointCloud PRIMITIVE: " << cloud_primitive->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;
			}

			// Obtain the primitive cloud boundaries
			std::array<float, 6> bounds_primitive(getPointCloudBoundaries(*cloud_primitive));
			/*std::cerr << "\nCylinder boundaries: "
				<< "\n\tx: " << "[" << bounds_cylinder[0] << "," << bounds_cylinder[1] << "]"
				<< "\n\ty: " << "[" << bounds_cylinder[2] << "," << bounds_cylinder[3] << "]"
				<< "\n\tz: " << "[" << bounds_cylinder[4] << "," << bounds_cylinder[5] << "]"
				<< std::endl;*/

			if (RENDER) {
				// ICP aligned point cloud is red
				cloud_primitive_color_h.setInputCloud(cloud_primitive);
				viewer.addPointCloud(cloud_primitive, cloud_primitive_color_h, "cloud_primitive", vp);
			}
			/*********************************/

			// Plot primitive shape
			switch (prim) {
			case michelangelo::PRIMITIVE_CUBE:							
				if (RENDER) {
					for (size_t i(0); i < n_planes; ++i) {
						// Plot cubeface cloud
						arr_cloud_cube_color_h[i].setInputCloud(arr_cloud_plane[i]);
						viewer.addPointCloud(arr_cloud_plane[i], arr_cloud_cube_color_h[i], "cloud_cubeface_" + std::to_string(i), vp);

						// Plot cubeface normal
						pcl::PCA<pcl::PointXYZ> pca;
						pca.setInputCloud(arr_cloud_plane[i]);
						Eigen::Vector4f mean(pca.getMean());
						pcl::PointXYZ face_center(mean.x() * 0.01, mean.y() * 0.01, mean.z() * 0.01);
						pcl::PointXYZ face_normal(face_center.x + arr_coeffs_plane[i]->values[0], face_center.y + arr_coeffs_plane[i]->values[1], face_center.z + arr_coeffs_plane[i]->values[2]);
						viewer.addLine(face_center, face_normal, "normal_cubeface_" + std::to_string(i));
					}
					viewer.addCube(*coefficients_primitive, "cube");
				}
				break;
			case michelangelo::PRIMITIVE_SPHERE:
				if (RENDER) {
					viewer.addSphere(*coefficients_primitive, "sphere");
				}
				break;
			case michelangelo::PRIMITIVE_CYLINDER:
				pcl::ModelCoefficients::Ptr corrected_coefs_primitive(new pcl::ModelCoefficients);
				correctCylShape(*corrected_coefs_primitive, *coefficients_primitive, *cloud_primitive);

				// Plot cylinder longitudinal axis //PointT
				pcl::PointXYZ point_on_axis((*coefficients_primitive).values[0], (*coefficients_primitive).values[1], (*coefficients_primitive).values[2]);
				pcl::PointXYZ axis_direction(point_on_axis.x + (*coefficients_primitive).values[3], point_on_axis.y + (*coefficients_primitive).values[4], point_on_axis.z + (*coefficients_primitive).values[5]);
				pcl::PointXYZ cam_origin(0.0, 0.0, 0.0);
				pcl::PointXYZ axis_projection((*coefficients_primitive).values[3], (*coefficients_primitive).values[4], 0.0);

				float camAngle(90.0 * M_PI / 180);
				michelangelo::correctAngle(axis_projection, camAngle);

				if (RENDER) {
					viewer.addCylinder(*corrected_coefs_primitive, "cylinder");
					viewer.addLine(cam_origin, axis_projection, "line");
				}

				// Calculate the angular difference			
				//float dTheta(M_PI - std::atan2(axis_projection.y, axis_projection.x));
				float dTheta(michelangelo::readAngle(axis_projection));
				std::string action(michelangelo::setAction(dTheta));
				std::cout << "* Current angle: " << dTheta * float(180) / M_PI << "\tAction: " << action << std::endl;

#endif	// PRIMITIVE_MODEL
				break;
			}

		}
#endif //ABCD
		if (RENDER) {
			viewer.spinOnce(1, true);
		}
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

int trimPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_trimmed, const std::array<std::array<float,2>,3> ranges, char a = 'o', float width = 0.010)
{
	for (auto i : ranges) {
		if (i[0] > i[1]) {
			return 1;
		}
	}
	if (a == 'o') {
		for (auto p : cloud.points) {
			if (ranges[0][0] <= p.x && p.x <= ranges[0][1] &&
				ranges[1][0] <= p.y && p.y <= ranges[1][1] &&
				ranges[2][0] <= p.z && p.z <= ranges[2][1]) {
				cloud_trimmed.push_back(pcl::PointXYZ(p.x, p.y, p.z));
			}
		}
	} else if (a == '+') {
		for (auto p : cloud.points) {
			if (//horizontal strip
				(ranges[0][0] <= p.x && p.x <= ranges[0][1] &&
				-width/2 <= p.y && p.y <= width/2 &&
				ranges[2][0] <= p.z && p.z <= ranges[2][1]) ||
				//vertical strip
				(-width/2 <= p.x && p.x <= width/2 &&
				ranges[1][0] <= p.y && p.y <= ranges[1][1] &&
				ranges[2][0] <= p.z && p.z <= ranges[2][1])) {
				cloud_trimmed.push_back(pcl::PointXYZ(p.x, p.y, p.z));
			}
		}
	} else {
		return 1;
	}

	return 0;
}

void downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_ds, int factor)
{		
	int i(0);
	for (auto p : cloud.points) {
		if (i == 0)
			cloud_ds.push_back(pcl::PointXYZ(p.x, p.y, p.z));
		if (i + 1 == factor) {
			i = 0;
		} else {
			++i;
		}
	}
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

std::vector<int> getCentroidsLCCP(const pcl::PointCloud<pcl::PointXYZL>& cloud, std::vector<uint32_t>& labels, std::vector<std::array<float, 3>>& centroids)
{
	std::vector<int> counts;
	for (auto p : cloud.points) {
		bool is_in(false);
		size_t i(0);
		while (!is_in && i < labels.size()) {
			if (labels[i] == p.label) {
				is_in = true;
			}
			else {
				++i;
			}
		}
		if (!is_in) {
			labels.push_back(p.label);
			counts.push_back(0);
			centroids.push_back({ 0.0, 0.0, 0.0 });
		}
		counts[i] += 1;
		centroids[i][0] += p.x;
		centroids[i][1] += p.y;
		centroids[i][2] += p.z;
	}
	for (size_t i(0); i < labels.size(); ++i) {
		centroids[i][0] /= counts[i];
		centroids[i][1] /= counts[i];
		centroids[i][2] /= counts[i];
	}
	return counts;
}

uint32_t selCentroidLCCP(const std::vector<uint32_t>& labels, const std::vector<std::array<float, 3>>& centroids)
{
	float d(10.0);
	uint32_t lbl(std::pow(2, 32) - 1);
	for (size_t i(0); i < centroids.size(); ++i) {
		float d_temp = std::sqrt(std::pow(centroids[i][0], 2) + std::pow(centroids[i][1], 2));
		if (0.0001 < d_temp && d_temp < d) {
			lbl = labels[i];
			d = d_temp;
		}
	}
	return lbl;
}

void getLabeledCloudLCCP(const pcl::PointCloud<pcl::PointXYZL>& cloud_lccp, pcl::PointCloud<pcl::PointXYZ>& cloud_seg, uint32_t label)
{
	for (auto p : cloud_lccp.points) {
		if (p.label == label)
			cloud_seg.points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
	}
}

float moving_average(float val, std::list<float>& vec, int n_samples, MovAvg mode=DEFAULT)
{		
	float avg(val);

	switch(mode){
	case DEFAULT:
		// Simple Moving Average		
		for (auto p : vec) {
			avg += p;
		}
		avg /= (vec.size() + 1);

	case EXPONENTIAL:
		// Exponentially Weighted Moving Average
		if(vec.size() > 0) {			
			avg = (1 - 1 / n_samples) * vec.back() + (1 / n_samples) * avg;
		}
	}

	if (vec.size() < n_samples) {
		vec.push_back(avg);
	}
	else {
		vec.pop_front();
		vec.push_back(avg);
	}

	return avg;
}

void correctLineShape(pcl::ModelCoefficients& line, const pcl::PointCloud<PointT>& cloud)
{
	pcl::PointXYZ point_on_axis(line.values[0], line.values[1], line.values[2]);
	pcl::PointXYZ axis_direction(line.values[3], line.values[4], line.values[5]);
	std::array<float, 2> arr(getPointCloudExtremes(cloud, point_on_axis, axis_direction));

	pcl::PointXYZ point_bottom;
	point_bottom.x = point_on_axis.x + arr[0] * axis_direction.x / normPointT(axis_direction);
	point_bottom.y = point_on_axis.y + arr[0] * axis_direction.y / normPointT(axis_direction);
	point_bottom.z = point_on_axis.z + arr[0] * axis_direction.z / normPointT(axis_direction);
	pcl::PointXYZ bottom_top_direction;
	bottom_top_direction.x = (-arr[0] + arr[1]) * axis_direction.x / normPointT(axis_direction);
	bottom_top_direction.y = (-arr[0] + arr[1]) * axis_direction.y / normPointT(axis_direction);
	bottom_top_direction.z = (-arr[0] + arr[1]) * axis_direction.z / normPointT(axis_direction);

	line.values[0] = point_bottom.x;
	line.values[1] = point_bottom.y;
	line.values[2] = point_bottom.z;
	line.values[3] = bottom_top_direction.x;
	line.values[4] = bottom_top_direction.y;
	line.values[5] = bottom_top_direction.z;
}

void correctCircleShape(pcl::ModelCoefficients& circle)
{
	std::array<float, 3> circle_normal = { circle.values[4],circle.values[5],circle.values[6] };
	circle.values[6] = circle.values[3];
	if (std::abs(circle_normal[0]) > 0.5) {
		circle.values[3] = 0.001;
	} else {
		circle.values[3] = circle_normal[0];
	}
	if (std::abs(circle_normal[1]) > 0.5) {
		circle.values[4] = 0.001;
	} else {
		circle.values[4] = circle_normal[1];
	}
	circle.values[5] = circle_normal[2];
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