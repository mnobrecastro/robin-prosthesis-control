#pragma once
#include "camera_depth.h"

#include <iostream>
#include <ctime>
#include <memory>
#include <thread>
#include <chrono>

#include <royale.hpp>
#include <sample_utils/PlatformResources.hpp>

namespace robin
{
	class RoyalePicoflexx :
		public CameraDepth
	{
	public:
		RoyalePicoflexx();
		~RoyalePicoflexx();

		void printInfo();

		void setDisparity(bool disparity = false);

		void start(bool);// override;

	protected:
		void points_to_pcl(const royale::DepthData* data, uint8_t depthConfidence);
        void exposure();
		void captureFrame();

	private:

        std::unique_ptr<royale::ICameraDevice> dev_;
		bool DISPARITY_ = false;

		// Royale pipeline objects and pointcloud, points.
        royale::DepthData* data_;
        std::mutex mu_data_;
        royale::Vector<royale::StreamId> streamIds_;
        std::thread thread_exposure_;


        /* A listener which receives a callback to onNewData when each depth frame is captured by the royale camera. */
        class Listener : public royale::IDepthDataListener
        {            
            /**
             * The output is scaled to fit in a terminal of this size. Although this is bigger than the
             * normal default size of 24 lines, a user with a smaller terminal is likely to scroll upwards
             * and understand the data that they're seeing. Limiting it to 24 lines makes the image much
             * harder to understand.
             */
            static const size_t MAX_HEIGHT = 40;
            /**
             * The output is scaled to fit in a terminal of this size. Here a terminal size of at least 80
             * columns is assumed, and a user with a narrower terminal window will get a very confusing
             * picture; but with all the wrapped-round lines being the same length the user will hopefully
             * understand that a larger window is required.
             */
            static const size_t MAX_WIDTH = 76;

            /* Data that has been received in onNewData, and will be printed in the paint() method. */
            struct FrameData
            {
                std::vector<uint32_t> exposureTimes;
                std::vector<std::string> asciiFrame;
                pcl::PointCloud<pcl::PointXYZ> cloudFrame;
            };

        public:
            
            /* This callback is called for each depth frame that is captured.  In a mixed-mode use case
             * (a use case with multiple streams), each callback refers to data from a single stream.
             */
            void onNewData(const royale::DepthData* data) override
            {
                // The royale::DepthData* data pointer will become invalid after onNewData returns.
                // When processing the data, it's necessary to either:
                // 1. Do all the processing before this method returns, or
                // 2. Copy the data (not just the pointer) for later processing, or
                // 3. Do processing that needs the full data here, and generate or copy only the data
                //    required for later processing
                //
                // The Royale library's depth-processing thread may block while waiting for this function to
                // return; if this function is slow then there may be some lag between capture and onNewData
                // for the next frame.  If it's very slow then Royale may drop frames to catch up.
                //
                // NOTE: to reduce the depth data by using it as an array of (data->width * data->height)
                // z coordinates would lose the accuracy.  The 3D depth points are not arranged in a
                // rectilinear projection, and the discarded x and y coordinates from the depth points
                // account for the optical projection (or optical distortion) of the camera.


                // Demonstration of how to retrieve the exposure times for the current stream. This is a
                // vector which can contain several numbers, because the depth frame is calculated from
                // several individual raw frames.
                auto exposureTimes = data->exposureTimes;

                // Scope for a lock while updating the internal model
                {
                    std::unique_lock<std::mutex> lock(m_lockForReceivedData);
                    auto& receivedData = m_receivedData[data->streamId];
                    receivedData.exposureTimes = exposureTimes.toStdVector();
                    receivedData.cloudFrame = *points_to_pcl(data, 128);
                }

                // passData() should be done in a separate thread, to avoid blocking onNewData().
                this->passData();
            }

            /**
             * In a full application, this could be the graphical widget's method for updating the screen,
             * called from the UI toolkit.
             */
            void passData()
            {                
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

                // Scope for the thread-safety lock
                {
                    // While locked, copy the data to local structures
                    std::unique_lock<std::mutex> lock(m_lockForReceivedData);
                    if (m_receivedData.empty()) {
                        return;
                    }
                    
                    // For each stream, append its data
                    for (auto streamId : m_streamIds)
                    {
                        const auto received = m_receivedData.find(streamId);
                        if (received != m_receivedData.end()) {
                            //allExposureTimes[streamId] = received->second.exposureTimes;
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cpy(new pcl::PointCloud<pcl::PointXYZ>(received->second.cloudFrame));
                            cloud = cloud_cpy;
                        }
                    }
                }

                cam_->mu_data_.lock();
                cam_->cloud_ = cloud;
                cam_->mu_data_.unlock();
            }


            pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const royale::DepthData* data, uint8_t depthConfidence)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                cloud->width = data->width;
                cloud->height = data->height;
                cloud->is_dense = false;
                cloud->points.resize(data->points.size());
                auto ptr = data->points.begin();
                for (auto& p : cloud->points) {
                	p.x = ptr->x;
                	p.y = ptr->y;
                	p.z = ptr->z;
                	ptr++;
                }      
                return cloud;
            }

             /**
              * Creates a listener which will have callbacks from two sources - the Royale framework, when a
              * new frame is received, and the UI toolkit, when the graphics are ready to repaint.
              */
            explicit Listener(const royale::Vector<royale::StreamId>& streamIds, RoyalePicoflexx& cam) :
                cam_(&cam),
                m_streamIds(streamIds)
            {}

        private:

            /* Pointer to the "parent" camera class object. */
            RoyalePicoflexx* cam_;

            /**
             * The StreamIds for all streams that are expected to be received.  For this example, it's a
             * constant set, so doesn't need protecting with a mutex.
             */
            const royale::Vector<royale::StreamId> m_streamIds;

            /**
             * Updated in each call to onNewData, for each stream it contains the most recently received
             * frame. Should only be accessed with m_lockForReceivedData held.
             */
            std::map<royale::StreamId, FrameData> m_receivedData;
            std::mutex m_lockForReceivedData;
        };
	};
}