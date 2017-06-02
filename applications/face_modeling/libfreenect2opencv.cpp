#include <iostream>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include "libfreenect2opencv.h"
#define DEPTHTORGB
using namespace std;

namespace libfreenect2opencv {
	
	Libfreenect2OpenCV::Libfreenect2OpenCV() :
		m_pipeline(nullptr),
		m_dev(nullptr),
		m_registration(nullptr),
		m_listener(nullptr)
	{
		if (m_freenect2.enumerateDevices() == 0) {
			throw runtime_error("no device connected");
		}

		string serial = m_freenect2.getDefaultDeviceSerialNumber();

		std::cout << "SERIAL: " << serial << std::endl;

		if (m_pipeline) {
			m_dev = m_freenect2.openDevice(serial, m_pipeline);
		}
		else {
			m_dev = m_freenect2.openDevice(serial);
		}

		if (m_dev == 0) {
			throw runtime_error("failure opening device");
		}

		m_listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color |
			libfreenect2::Frame::Depth |
			libfreenect2::Frame::Ir);

		m_dev->setColorFrameListener(m_listener);
		m_dev->setIrAndDepthFrameListener(m_listener);
		//libfreenect2::Freenect2Device::IrCameraParams irin = m_dev->getIrCameraParams();
		m_dev->start();

		std::cout << "device serial: " << m_dev->getSerialNumber() << std::endl;
		std::cout << "device firmware: " << m_dev->getFirmwareVersion() << std::endl;

		m_registration = new libfreenect2::Registration(m_dev->getIrCameraParams(),
			m_dev->getColorCameraParams());
	}
	void Libfreenect2OpenCV::updateMat()
	{
		libfreenect2::FrameMap frames;

		libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
		if(true)
		{
			m_listener->waitForNewFrame(frames);
			libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
			libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
#ifdef IR
			libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
#endif
			cv::Mat tmpRGB(rgb->height, rgb->width, CV_8UC4, rgb->data);
			cv::Mat tmpDepth(depth->height, depth->width, CV_32FC1, depth->data);
#ifdef IR
			cv::Mat tmpIR(ir->height, ir->width, CV_32FC1, ir->data);
#endif	
			cv::flip(tmpRGB, m_rgbMat, 1);			
			cv::flip(tmpDepth, m_depthMat, 1);
#ifdef IR
			cv::flip(tmpIR, m_IRMat, 1);
			m_IRMat /= 65535.0f;			
#endif
#ifdef DEPTHTORGB
			m_registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
#else
			m_registration->apply(rgb, depth, &undistorted, &registered);
#endif
			cv::Mat tmpUndisDepth(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
			cv::flip(tmpUndisDepth, m_depthMatUndistorted, 1);
			cv::Mat tmpRGB2Depth(registered.height, registered.width, CV_8UC4, registered.data);
			cv::flip(tmpRGB2Depth, m_rgb2depthMat, 1);
#ifdef DEPTHTORGB
			cv::Mat tmpDepth2RGB;
			cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).rowRange(cv::Range(1, 1081)).copyTo(tmpDepth2RGB);
			cv::flip(tmpDepth2RGB, m_depth2rgbMat, 1);
#endif
			m_listener->release(frames);
		}
	}
	Libfreenect2OpenCV::~Libfreenect2OpenCV()
	{
		m_dev->stop();
		delete m_registration;
	}

}