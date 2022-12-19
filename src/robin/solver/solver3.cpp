#include "solver3.h"

namespace robin
{
	Solver3::Solver3()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		cloud_ = cloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw_clr(new pcl::PointCloud<pcl::PointXYZRGB>());
		cloud_raw_clr_ = cloud_raw_clr;
	}

	Solver3::~Solver3() {}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::getPointCloud() const {
		return cloud_;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Solver3::getRawColored() const {
		return cloud_raw_clr_;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::getPreprocessed() const {
		return cloud_preproc_;
	}

	void Solver3::addSensor(robin::Sensor3* sensor)
	{
		sensors_.push_back(sensor);
		return;
	}

	void Solver3::addSensor(robin::Webcam* sensor)
	{
		sensors2_.push_back(sensor);
		return;
	}

	void Solver3::setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
		seg_obj_ptr_ = seg_obj;

		// Initialization of the SACSegmentation objects for each Primitive3
		// (in case the user calls for multiple simultaneous fitting).
		seg_obj_sph_ = new pcl::SACSegmentation<pcl::PointXYZ>(*seg_obj_ptr_);
		seg_obj_cub_ = new pcl::SACSegmentation<pcl::PointXYZ>(*seg_obj_ptr_);
		seg_obj_cyl_ = new pcl::SACSegmentation<pcl::PointXYZ>(*seg_obj_ptr_);
	}

	void Solver3::setUseNormals(bool seg_normals)
	{
		seg_normals_ = seg_normals;
		return;
	}

	void Solver3::setPlaneRemoval(bool seg_plane_removal)
	{
		seg_plane_removal_ = seg_plane_removal;
		return;
	}

	std::vector<robin::Sensor3*> Solver3::getSensors() const
	{
		return sensors_;
	}


	void Solver3::setCrop(float x0 = 999.9, float x1 = 999.9, float y0 = 999.9, float y1 = 999.9, float z0 = 999.9, float z1 = 999.9)
	{
		filterOnOff_ = true;
		limits_ = { x0, x1, y0, y1, z0, z1 };
		return;
	}

	void Solver3::setDownsample(float voxel_size = 0.005)
	{
		downsampleOnOff_ = true;
		voxel_size_ = voxel_size;
		return;
	}

	void Solver3::setResample(size_t order, float radius)
	{
		resampleOnOff_ = true;
		resamp_order_ = order;
		resamp_radius_ = radius;
		return;
	}

	void Solver3::setDenoise(size_t k, float threshold)
	{
		denoiseOnOff_ = true;
		denoise_k_ = k;
		denoise_threshold_ = threshold;
		return;
	}

	void Solver3::setPCA(bool b = false) {

	}


	void Solver3::setFairSelection(bool fairness)
	{
		fairselectionOnOff_ = fairness;
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::trimPointCloud()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trimmed(new pcl::PointCloud<pcl::PointXYZ>());
		for (auto p : cloud_->points) {
			if (limits_[0] <= p.x && p.x <= limits_[1] &&
				limits_[2] <= p.y && p.y <= limits_[3] &&
				limits_[4] <= p.z && p.z <= limits_[5]) {
				cloud_trimmed->push_back(pcl::PointXYZ(p.x, p.y, p.z));
			}
		}
		return cloud_trimmed;
	}

	void Solver3::crop()
	{
		if (filterOnOff_ && cloud_->points.size() > 0) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Build a passthrough filter to remove unwated points
			pcl::PassThrough<pcl::PointXYZ> pass(true);
			if (false) {
				pass.setInputCloud(cloud_);
				pass.setFilterFieldName("x");
				pass.setFilterLimits(limits_[0], limits_[1]);
				pass.filter(*cloud_);

				pass.setInputCloud(cloud_);
				pass.setFilterFieldName("y");
				pass.setFilterLimits(limits_[2], limits_[3]);
				pass.filter(*cloud_);

				pass.setInputCloud(cloud_);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(limits_[4], limits_[5]);
				pass.filter(*cloud_);
			}
			else {
				cloud_ = trimPointCloud();
			}

			tf = std::time(0);
			std::cout << "PointCloud after filtering: " << cloud_->points.size() << " data points (in " << std::difftime(t0, tf) << " ms).\n";
		}
		return;
	}

	void Solver3::downsample()
	{
		if (downsampleOnOff_ && cloud_->points.size() > 0) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Downsampling the point cloud
			if (true) {
				pcl::ApproximateVoxelGrid<pcl::PointXYZ> dsfilt;
				dsfilt.setInputCloud(cloud_);
				dsfilt.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
				dsfilt.filter(*cloud_);
			}
			else {
				pcl::RandomSample<pcl::PointXYZ> dsfilt;
				dsfilt.setInputCloud(cloud_);
				dsfilt.setSample(2000);
				dsfilt.filter(*cloud_);
			}

			tf = std::time(0);
			std::cerr << "PointCloud after downsampling: " << cloud_->width * cloud_->height << "=" << cloud_->points.size()
				<< " data points (in " << std::difftime(t0, tf) << " ms).\n";
		}
		return;
	}

	void Solver3::resample()
	{
		if (resampleOnOff_ && cloud_->points.size() > 0) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Smoothing a raw point cloud by resampling it

			// Create a kD-Tree for point search
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

			// Output has the PointNormal type in order to store the normals calculated by MLS
			pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

			// Init object (second point type is for the normals, even if unused)
			pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

			mls.setComputeNormals(true);

			// Set MovingLeastSquares parameters
			mls.setInputCloud(cloud_);
			mls.setPolynomialOrder(resamp_order_);
			mls.setSearchMethod(tree);
			mls.setSearchRadius(resamp_radius_);

			// Reconstruct
			mls.process(*mls_points);

			// Copying the PointCloud
			cloud_->clear();
			cloud_->resize(mls_points->size());
			auto ptr = mls_points->points.begin();
			for (auto& p : cloud_->points) {
				p.x = ptr->x;
				p.y = ptr->y;
				p.z = ptr->z;
				ptr++;
			}

			tf = std::time(0);
			std::cerr << "PointCloud after resampling: " << cloud_->width * cloud_->height << "=" << cloud_->points.size()
				<< " data points (in " << std::difftime(t0, tf) << " ms).\n";
		}
		return;
	}

	void Solver3::denoise()
	{
		if (denoiseOnOff_ && cloud_->points.size() > 0) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Denoising the point cloud
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud(cloud_);
			sor.setMeanK(denoise_k_);
			sor.setStddevMulThresh(denoise_threshold_);
			sor.filter(*cloud_);

			tf = std::time(0);
			std::cerr << "PointCloud after denoising: " << cloud_->width * cloud_->height << "=" << cloud_->points.size()
				<< " data points (in " << std::difftime(t0, tf) << " ms).\n";
		}
		return;
	}

	void Solver3::principal_components(robin::Primitive3d3*& prim)
	{
		primitive_ = prim;

		// Reset the solver's (temp and coloured) PointClouds
		cloud_->clear();
		cloud_raw_clr_->clear();

		for (Sensor3* s : sensors_) {
			*cloud_ += *s->getPointCloud();
			*cloud_raw_clr_ += *s->getRawColored();
		}

		this->crop();
		this->downsample();
		this->denoise();

		// Store the pre-processed cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_preproc(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
		cloud_preproc_ = cloud_preproc;

		// Check whether the PointCloud has enough points to proceed
		if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
			primitive_->reset();
			std::cerr << "* Not enough points to compute the Principal Components.\n";
			return;
		}

		primitive_->pca(*cloud_);

		// Copy the address in case a Primitive3d3 has been provided and mutated to a derived primitive
		//prim = primitive_;
		return;
	}

	void Solver3::solve(robin::Primitive3d3*& prim)
	{		
		primitive_ = prim;
		
		// Reset the solver's (temp and coloured) PointClouds
		cloud_->clear();
		cloud_raw_clr_->clear();

		if (!sensors_.empty()) {
			if (typeid(*sensors_[0]) == typeid(robin::Sensor3)) {
				for (Sensor3* s : sensors_) {
					*cloud_ += *s->getPointCloud();
					*cloud_raw_clr_ += *s->getRawColored();
				}
			}
		}
		else if (!sensors2_.empty()) {
			if (typeid(*sensors2_[0]) == typeid(robin::Webcam)) {

				im_src_ = (sensors2_[0]->getImage()); // CV_8UC3
				int H(im_src_->size().height), W(im_src_->size().width);
				// Cropping the 'depthmap' image
				std::shared_ptr<cv::Mat> im_depth(new cv::Mat((*im_src_)(cv::Range(H / 4, 3 * H / 4), cv::Range(0, W / 2))));
				im_depth_ = im_depth;
				std::printf("im_depth (%d x %d)\n", im_depth_->rows, im_depth_->cols);
				// Cropping the 'mask' image
				std::shared_ptr<cv::Mat> im_mask(new cv::Mat((*im_src_)(cv::Range(H / 4, 3 * H / 4), cv::Range(W / 2, W))));
				im_mask_ = im_mask;

				// RS Camera Intrinsics
				float rx(424), ry(240); // Resolution
				float cx(211.937), cy(122.685); // Center dist
				float f(211.357); // Focal lenght
				float s(1000); // Scaling factor

				// --------------------------------------------------------------------------

				// Retrieve scalar from the JET Colormap	
				// Equals the GNU Octave colormap "jet".
				// https://github.com/opencv/opencv/blob/ebb6915e588fcee1e6664cce670f0253bac0e67b/modules/imgproc/src/colormap.cpp
				// https://stackoverflow.com/questions/51824718/opencv-jetmap-or-colormap-to-grayscale-reverse-applycolormap
				const float r[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.00588235294117645f,0.02156862745098032f,0.03725490196078418f,0.05294117647058827f,0.06862745098039214f,0.084313725490196f,0.1000000000000001f,0.115686274509804f,0.1313725490196078f,0.1470588235294117f,0.1627450980392156f,0.1784313725490196f,0.1941176470588235f,0.2098039215686274f,0.2254901960784315f,0.2411764705882353f,0.2568627450980392f,0.2725490196078431f,0.2882352941176469f,0.303921568627451f,0.3196078431372549f,0.3352941176470587f,0.3509803921568628f,0.3666666666666667f,0.3823529411764706f,0.3980392156862744f,0.4137254901960783f,0.4294117647058824f,0.4450980392156862f,0.4607843137254901f,0.4764705882352942f,0.4921568627450981f,0.5078431372549019f,0.5235294117647058f,0.5392156862745097f,0.5549019607843135f,0.5705882352941174f,0.5862745098039217f,0.6019607843137256f,0.6176470588235294f,0.6333333333333333f,0.6490196078431372f,0.664705882352941f,0.6803921568627449f,0.6960784313725492f,0.7117647058823531f,0.7274509803921569f,0.7431372549019608f,0.7588235294117647f,0.7745098039215685f,0.7901960784313724f,0.8058823529411763f,0.8215686274509801f,0.8372549019607844f,0.8529411764705883f,0.8686274509803922f,0.884313725490196f,0.8999999999999999f,0.9156862745098038f,0.9313725490196076f,0.947058823529412f,0.9627450980392158f,0.9784313725490197f,0.9941176470588236f,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.9862745098039216f,0.9705882352941178f,0.9549019607843139f,0.93921568627451f,0.9235294117647062f,0.9078431372549018f,0.892156862745098f,0.8764705882352941f,0.8607843137254902f,0.8450980392156864f,0.8294117647058825f,0.8137254901960786f,0.7980392156862743f,0.7823529411764705f,0.7666666666666666f,0.7509803921568627f,0.7352941176470589f,0.719607843137255f,0.7039215686274511f,0.6882352941176473f,0.6725490196078434f,0.6568627450980391f,0.6411764705882352f,0.6254901960784314f,0.6098039215686275f,0.5941176470588236f,0.5784313725490198f,0.5627450980392159f,0.5470588235294116f,0.5313725490196077f,0.5156862745098039f,0.5f };
				const float g[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.001960784313725483f,0.01764705882352935f,0.03333333333333333f,0.0490196078431373f,0.06470588235294117f,0.08039215686274503f,0.09607843137254901f,0.111764705882353f,0.1274509803921569f,0.1431372549019607f,0.1588235294117647f,0.1745098039215687f,0.1901960784313725f,0.2058823529411764f,0.2215686274509804f,0.2372549019607844f,0.2529411764705882f,0.2686274509803921f,0.2843137254901961f,0.3f,0.3156862745098039f,0.3313725490196078f,0.3470588235294118f,0.3627450980392157f,0.3784313725490196f,0.3941176470588235f,0.4098039215686274f,0.4254901960784314f,0.4411764705882353f,0.4568627450980391f,0.4725490196078431f,0.4882352941176471f,0.503921568627451f,0.5196078431372548f,0.5352941176470587f,0.5509803921568628f,0.5666666666666667f,0.5823529411764705f,0.5980392156862746f,0.6137254901960785f,0.6294117647058823f,0.6450980392156862f,0.6607843137254901f,0.6764705882352942f,0.692156862745098f,0.7078431372549019f,0.723529411764706f,0.7392156862745098f,0.7549019607843137f,0.7705882352941176f,0.7862745098039214f,0.8019607843137255f,0.8176470588235294f,0.8333333333333333f,0.8490196078431373f,0.8647058823529412f,0.8803921568627451f,0.8960784313725489f,0.9117647058823528f,0.9274509803921569f,0.9431372549019608f,0.9588235294117646f,0.9745098039215687f,0.9901960784313726f,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.9901960784313726f,0.9745098039215687f,0.9588235294117649f,0.943137254901961f,0.9274509803921571f,0.9117647058823528f,0.8960784313725489f,0.8803921568627451f,0.8647058823529412f,0.8490196078431373f,0.8333333333333335f,0.8176470588235296f,0.8019607843137253f,0.7862745098039214f,0.7705882352941176f,0.7549019607843137f,0.7392156862745098f,0.723529411764706f,0.7078431372549021f,0.6921568627450982f,0.6764705882352944f,0.6607843137254901f,0.6450980392156862f,0.6294117647058823f,0.6137254901960785f,0.5980392156862746f,0.5823529411764707f,0.5666666666666669f,0.5509803921568626f,0.5352941176470587f,0.5196078431372548f,0.503921568627451f,0.4882352941176471f,0.4725490196078432f,0.4568627450980394f,0.4411764705882355f,0.4254901960784316f,0.4098039215686273f,0.3941176470588235f,0.3784313725490196f,0.3627450980392157f,0.3470588235294119f,0.331372549019608f,0.3156862745098041f,0.2999999999999998f,0.284313725490196f,0.2686274509803921f,0.2529411764705882f,0.2372549019607844f,0.2215686274509805f,0.2058823529411766f,0.1901960784313728f,0.1745098039215689f,0.1588235294117646f,0.1431372549019607f,0.1274509803921569f,0.111764705882353f,0.09607843137254912f,0.08039215686274526f,0.06470588235294139f,0.04901960784313708f,0.03333333333333321f,0.01764705882352935f,0.001960784313725483f,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
				const float b[] = { 0.5f,0.5156862745098039f,0.5313725490196078f,0.5470588235294118f,0.5627450980392157f,0.5784313725490196f,0.5941176470588235f,0.6098039215686275f,0.6254901960784314f,0.6411764705882352f,0.6568627450980392f,0.6725490196078432f,0.6882352941176471f,0.7039215686274509f,0.7196078431372549f,0.7352941176470589f,0.7509803921568627f,0.7666666666666666f,0.7823529411764706f,0.7980392156862746f,0.8137254901960784f,0.8294117647058823f,0.8450980392156863f,0.8607843137254902f,0.8764705882352941f,0.892156862745098f,0.907843137254902f,0.9235294117647059f,0.9392156862745098f,0.9549019607843137f,0.9705882352941176f,0.9862745098039216f,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.9941176470588236f,0.9784313725490197f,0.9627450980392158f,0.9470588235294117f,0.9313725490196079f,0.915686274509804f,0.8999999999999999f,0.884313725490196f,0.8686274509803922f,0.8529411764705883f,0.8372549019607844f,0.8215686274509804f,0.8058823529411765f,0.7901960784313726f,0.7745098039215685f,0.7588235294117647f,0.7431372549019608f,0.7274509803921569f,0.7117647058823531f,0.696078431372549f,0.6803921568627451f,0.6647058823529413f,0.6490196078431372f,0.6333333333333333f,0.6176470588235294f,0.6019607843137256f,0.5862745098039217f,0.5705882352941176f,0.5549019607843138f,0.5392156862745099f,0.5235294117647058f,0.5078431372549019f,0.4921568627450981f,0.4764705882352942f,0.4607843137254903f,0.4450980392156865f,0.4294117647058826f,0.4137254901960783f,0.3980392156862744f,0.3823529411764706f,0.3666666666666667f,0.3509803921568628f,0.335294117647059f,0.3196078431372551f,0.3039215686274508f,0.2882352941176469f,0.2725490196078431f,0.2568627450980392f,0.2411764705882353f,0.2254901960784315f,0.2098039215686276f,0.1941176470588237f,0.1784313725490199f,0.1627450980392156f,0.1470588235294117f,0.1313725490196078f,0.115686274509804f,0.1000000000000001f,0.08431372549019622f,0.06862745098039236f,0.05294117647058805f,0.03725490196078418f,0.02156862745098032f,0.00588235294117645f,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
				int N(sizeof(r) / sizeof(float));

				int N_DEPTH = 500;
				cv::Mat lut_ = linear_colormap(linspace(0, 1, N),
					cv::Mat(256, 1, CV_32FC1, (void*)r).clone(), // red
					cv::Mat(256, 1, CV_32FC1, (void*)g).clone(), // green
					cv::Mat(256, 1, CV_32FC1, (void*)b).clone(), // blue
					linspace(0, 1, N_DEPTH));
				std::printf("lut_ (%d x %d) 0->(%d, %d, %d)\t 499->(%d, %d, %d)\n", lut_.rows, lut_.cols, lut_.at<cv::Vec3b>(0, 0)[0], lut_.at<cv::Vec3b>(0, 0)[1], lut_.at<cv::Vec3b>(0, 0)[2], lut_.at<cv::Vec3b>(N_DEPTH - 1, 0)[0], lut_.at<cv::Vec3b>(N_DEPTH - 1, 0)[1], lut_.at<cv::Vec3b>(N_DEPTH - 1, 0)[2]);
				cv::Mat lut(cv::Mat(cv::Size(3, N_DEPTH), CV_32F, cv::Scalar(0.0)));
				for (int i(0); i < N_DEPTH; ++i) {
					lut.at<float>(i, 0) = float(lut_.at<cv::Vec3b>(i, 0)[0]);
					lut.at<float>(i, 1) = float(lut_.at<cv::Vec3b>(i, 0)[1]);
					lut.at<float>(i, 2) = float(lut_.at<cv::Vec3b>(i, 0)[2]);
				}
				std::printf("lut (%d x %d) 0->(%.3f, %.3f, %.3f)\t 499->(%.3f, %.3f, %.3f)\n", lut.rows, lut.cols, lut.at<float>(0, 0), lut.at<float>(0, 1), lut.at<float>(0, 2), lut.at<float>(N_DEPTH - 1, 0), lut.at<float>(N_DEPTH - 1, 1), lut.at<float>(N_DEPTH - 1, 2));

				auto index_params = new cv::flann::KDTreeIndexParams(5);
				auto search_params = new cv::flann::SearchParams(50);
				cv::FlannBasedMatcher matcher(index_params, search_params);
				matcher.add(lut);
				matcher.train();

				int querySize(im_depth_->rows * im_depth_->cols);
				cv::Mat query(cv::Mat(cv::Size(3, querySize), CV_32F, cv::Scalar(0.0)));
				int i(0);
				for (int y(0); y < im_depth_->rows; ++y) {
					for (int x(0); x < im_depth_->cols; ++x) {
						query.at<float>(i, 0) = float(im_depth_->at<cv::Vec3b>(y, x)[0]);
						query.at<float>(i, 1) = float(im_depth_->at<cv::Vec3b>(y, x)[1]);
						query.at<float>(i, 2) = float(im_depth_->at<cv::Vec3b>(y, x)[2]);
						++i;
					}
				}
				std::printf("query (%d x %d) 0->(%.3f, %.3f, %.3f)\t 255->(%.3f, %.3f, %.3f)\n", query.rows, query.cols, query.at<float>(0, 0), query.at<float>(0, 1), query.at<float>(0, 2), query.at<float>(255, 0), query.at<float>(255, 1), query.at<float>(255, 2));

				std::vector<cv::DMatch> matches;
				matcher.match(query, matches);
				std::cout << "Number of matches is " << matches.size() << " trainIdx: " << matches[0].trainIdx << " distance: " << matches[0].distance << "\n";

				cv::Mat depthmap(cv::Mat(cv::Size(1, querySize), CV_32F, cv::Scalar(0.0)));
				for (int i(0); i < querySize; ++i) {
					depthmap.at<float>(i, 0) = float(matches[i].trainIdx);
				}
				depthmap = depthmap.reshape(0, { im_depth_->rows, im_depth_->cols });

				// --------------------------------------------------------------------------

				for (size_t y(0); y < im_depth_->size().height; ++y) {
					for (size_t x(0); x < im_depth_->size().width; ++x) {
						// Recall BGR ordering
						float pZ = depthmap.at<float>(y, x) / s;
						float pX = (float(x) - cx) * pZ / f;
						float pY = (float(y) - cy) * pZ / f;
						cloud_->push_back(pcl::PointXYZ(pX, pY, pZ));
					}
				}
			}
		}
		else {
			throw std::invalid_argument("No sensor or invalid SensorX was provided.");
		}

		/*try{}
		catch (const std::exception& e) {
			std::cerr << "No sensor or invalid SensorX was provided.\n";
		}*/
		

		this->crop();		
		this->downsample();
		this->denoise();
		
		//this->resample();
		/*pcl::RandomSample<pcl::PointXYZ> dsfilt;
		dsfilt.setInputCloud(cloud_);
		dsfilt.setSample(cloud_->points.size());
		dsfilt.filter(*cloud_);*/

		// Store the pre-processed cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_preproc(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
		cloud_preproc_ = cloud_preproc;

		this->segment();

		// Copy the address in case a Primitive3d3 has been provided and mutated to a derived primitive
		prim = primitive_;
		return;
	}

	void Solver3::segment()
	{
		// Check whether the PointCloud has enough points to proceed
		if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
			primitive_->reset();
			std::cerr << "* Not enough points to perform SAC segmentation.\n";
			return;
		}
		
		// Check whether any Primitive3 has been provided
		if (primitive_ != nullptr) {
			// Perform initial plane removal
			if (seg_plane_removal_) {
				robin::Primitive3Plane plane;
				if (seg_obj_ptr_ != nullptr) {					
					plane.fit(cloud_, seg_obj_ptr_);
				}
				else{
					plane.fit(cloud_, seg_normals_);
				}
			}

			// Check whether a specific/non-generice Primitive3 has been provided
			if (typeid(*primitive_) != typeid(robin::Primitive3d3)) {
				this->fitPrimitive(primitive_, cloud_, seg_obj_ptr_);
			}
			else {
				// Multiple simultaneous fitting of pre-defined Primtive3's 

				// Set the coefficients for each Primitive3 (i.e. initial guess)
				p_sph_->setCoefficients({ 0.000, 0.000, 0.150, 0.050 });
				//p_cub_->setCoefficients({ 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000 }); // N/A
				p_cyl_->setCoefficients({ 0.000, 0.000, 0.150, 0.000, 0.000, 0.100, 0.050 });

				// Define a copy of the PointCloud for each Primitive3				
				pcl::PointCloud<pcl::PointXYZ>::Ptr c_sph(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
				pcl::PointCloud<pcl::PointXYZ>::Ptr c_cub(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
				pcl::PointCloud<pcl::PointXYZ>::Ptr c_cyl(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));

				// Fit each primitive to its respective PointCloud copy
#ifdef MULTITHREADING
				std::thread t_sph = std::thread(&Solver3::fitPrimitive, this, std::ref(p_sph_), std::ref(c_sph), std::ref(seg_obj_sph_));
				std::thread t_cub = std::thread(&Solver3::fitPrimitive, this, std::ref(p_cub_), std::ref(c_cub), std::ref(seg_obj_cub_));
				std::thread t_cyl = std::thread(&Solver3::fitPrimitive, this, std::ref(p_cyl_), std::ref(c_cyl), std::ref(seg_obj_cyl_));
				t_sph.join();
				t_cub.join();
				t_cyl.join();
#else
				p_sph_->fit(c_sph, seg_obj_sph_);
				p_cub_->fit(c_cub, seg_obj_cub_);
				p_cyl_->fit(c_cyl, seg_obj_cyl_);
#endif

				// Selection of the best fitting Primitive3d3
				float cloud_size(cloud_->points.size());
				float fit_percent(0.0);

				float sph_size(p_sph_->getPointCloud()->points.size());
				std::cout << "Sphere FitPercent: " << sph_size / cloud_size << '\n';
				float cub_size(p_cub_->getPointCloud()->points.size());
				std::cout << "Cuboid FitPercent: " << cub_size / cloud_size << '\n';
				float cyl_size(p_cyl_->getPointCloud()->points.size());
				std::cout << "Cylinder FitPercent: " << cyl_size / cloud_size << '\n';
				
				if (fairselectionOnOff_) {
					if (sph_size == 0 || cub_size == 0 || cyl_size == 0) {
						// Fair selection: if at least one of the Primitive3d3 has not been fitted, then no primitive should be selected.
						primitive_ = primitive_;
						pcl::PointCloud<pcl::PointXYZ>::Ptr c_empty(new pcl::PointCloud<pcl::PointXYZ>());
						cloud_ = c_empty;
						return;
					}
				}
				// Pick the biggest fit cloud that corresponds to the correct primitive fitting
				if (fit_percent < sph_size / cloud_size) {
					fit_percent = sph_size / cloud_size;
					primitive_ = p_sph_;
					cloud_ = c_sph;
				}
				if (fit_percent < cub_size / cloud_size) {
					fit_percent = cub_size / cloud_size;
					primitive_ = p_cub_;
					cloud_ = c_cub;
				}
				if (fit_percent < cyl_size / cloud_size) {
					fit_percent = cyl_size / cloud_size;
					primitive_ = p_cyl_;
					cloud_ = c_cyl;
				}
			}
		}
		return;
	}

	void Solver3::fitPrimitive(robin::Primitive3d3*& prim, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::SACSegmentation<pcl::PointXYZ>*& seg_obj)
	{
		// Check whether an instance of Segmentation has been provided.
		if (seg_obj_ptr_ != nullptr) {
			std::cout << "Segmentation object has been provided!\n";
			prim->fit(cloud, seg_obj);
		}
		else {
			std::cout << "NO segmentation object has been provided.\n";
			prim->fit(cloud, seg_normals_);
		}
		return;
	}




	cv::Mat Solver3::linspace(float x0, float x1, int n)
	{
		cv::Mat pts(n, 1, CV_32FC1);
		float step = (x1 - x0) / (n - 1);
		for (int i = 0; i < n; i++)
			pts.at<float>(i, 0) = x0 + i * step;
		return pts;
	}

	void Solver3::sortMatrixRowsByIndices(cv::InputArray _src, cv::InputArray _indices, cv::OutputArray _dst)
	{
		if (_indices.getMat().type() != CV_32SC1)
			CV_Error(cv::Error::StsUnsupportedFormat, "cv::sortRowsByIndices only works on integer indices!");
		cv::Mat src = _src.getMat();
		std::vector<int> indices = _indices.getMat();
		_dst.create(src.rows, src.cols, src.type());
		cv::Mat dst = _dst.getMat();
		for (size_t idx = 0; idx < indices.size(); idx++) {
			cv::Mat originalRow = src.row(indices[idx]);
			cv::Mat sortedRow = dst.row((int)idx);
			originalRow.copyTo(sortedRow);
		}
		return;
	}

	cv::Mat Solver3::sortMatrixRowsByIndices(cv::InputArray src, cv::InputArray indices)
	{
		cv::Mat dst;
		sortMatrixRowsByIndices(src, indices, dst);
		return dst;
	}


	cv::Mat Solver3::argsort(cv::InputArray _src, bool ascending)
	{
		cv::Mat src = _src.getMat();
		if (src.rows != 1 && src.cols != 1)
			CV_Error(cv::Error::StsBadArg, "cv::argsort only sorts 1D matrices.");
		int flags = cv::SORT_EVERY_ROW | (ascending ? cv::SORT_ASCENDING : cv::SORT_DESCENDING);
		cv::Mat sorted_indices;
		cv::sortIdx(src.reshape(1, 1), sorted_indices, flags);
		return sorted_indices;
	}

	cv::Mat Solver3::interp1_(const cv::Mat& X_, const cv::Mat& Y_, const cv::Mat& XI)
	{
		int n = XI.rows;
		// sort input table
		std::vector<int> sort_indices = argsort(X_);
		cv::Mat X = sortMatrixRowsByIndices(X_, sort_indices);
		cv::Mat Y = sortMatrixRowsByIndices(Y_, sort_indices);
		// interpolated values
		cv::Mat yi = cv::Mat::zeros(XI.size(), XI.type());
		for (int i = 0; i < n; i++) {
			int low = 0;
			int high = X.rows - 1;
			// set bounds
			if (XI.at<float>(i, 0) < X.at<float>(low, 0))
				high = 1;
			if (XI.at<float>(i, 0) > X.at<float>(high, 0))
				low = high - 1;
			// binary search
			while ((high - low) > 1) {
				const int c = low + ((high - low) >> 1);
				if (XI.at<float>(i, 0) > X.at<float>(c, 0)) {
					low = c;
				}
				else {
					high = c;
				}
			}
			// linear interpolation
			yi.at<float>(i, 0) += Y.at<float>(low, 0)
				+ (XI.at<float>(i, 0) - X.at<float>(low, 0))
				* (Y.at<float>(high, 0) - Y.at<float>(low, 0))
				/ (X.at<float>(high, 0) - X.at<float>(low, 0));
		}
		return yi;
	}

	cv::Mat Solver3::interp1(cv::InputArray _x, cv::InputArray _Y, cv::InputArray _xi)
	{
		// get matrices
		cv::Mat x = _x.getMat();
		cv::Mat Y = _Y.getMat();
		cv::Mat xi = _xi.getMat();
		// check types & alignment
		CV_Assert((x.type() == Y.type()) && (Y.type() == xi.type()));
		CV_Assert((x.cols == 1) && (x.rows == Y.rows) && (x.cols == Y.cols));
		// call interp1
		return interp1_(x, Y, xi);
	}

	cv::Mat Solver3::linear_colormap(cv::InputArray X, cv::InputArray r, cv::InputArray g, cv::InputArray b, cv::InputArray xi) {
		cv::Mat lut, lut8;
		cv::Mat planes[] = {
				interp1(X, b, xi),
				interp1(X, g, xi),
				interp1(X, r, xi) };
		cv::merge(planes, 3, lut);
		lut.convertTo(lut8, CV_8U, 255.);
		return lut8;
	}
}