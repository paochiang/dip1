#pragma once
//#include "header.h"
#include <condition_variable>
#include <mutex>
#include <thread>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>

namespace libfreenect2opencv {

	class  Libfreenect2OpenCV {

		libfreenect2::PacketPipeline *m_pipeline;
		libfreenect2::Freenect2Device *m_dev;
		libfreenect2::Registration *m_registration;
		libfreenect2::SyncMultiFrameListener *m_listener;
		libfreenect2::Freenect2 m_freenect2;

		cv::Mat m_rgbMat;
		cv::Mat m_depthMat;
		cv::Mat m_depthMatUndistorted;
		cv::Mat m_IRMat;
		cv::Mat m_rgb2depthMat;
		cv::Mat m_depth2rgbMat;
	public:
		Libfreenect2OpenCV();
		void updateMat();
		virtual ~Libfreenect2OpenCV();

		const libfreenect2::Freenect2Device::IrCameraParams getIRCameraParams() { return m_dev->getIrCameraParams(); }
		const libfreenect2::Freenect2Device::ColorCameraParams getColorCameraParams() { return m_dev->getColorCameraParams(); }
		const cv::Mat & getDepthMat()
		{
			return m_depthMat;
		}

		const cv::Mat & getDepthMatUndistorted()
		{
			return m_depthMatUndistorted;
		}

		const cv::Mat & getIRMat()
		{
			return m_IRMat;
		}

		const cv::Mat & getRGB2Depth()
		{
			return m_rgb2depthMat;
		}

		const cv::Mat & getDepth2RGB()
		{
			return m_depth2rgbMat;
		}

		const cv::Mat & getRGBMat()
		{
			return m_rgbMat;
		}
	};
}

