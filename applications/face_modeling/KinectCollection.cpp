#include <thread>
#include <opencv2/opencv.hpp>
#include "KinectCollection.h"
#include <time.h>
#define	SafeRelease(ptr)		do{ if ( ptr != NULL ){ ptr->Release(); ptr = NULL; } }while(0)
#define	SafeDelete(ptr)		do{ if ( ptr != NULL ){ delete ptr; ptr = NULL; } }while(0)

template<class Interface>
inline void SafeReleaseObj(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}


KinectCollection::KinectCollection()
{

}


KinectCollection::~KinectCollection()
{
}

bool KinectCollection::init(int idx)
{
	kinect_sensor_ = NULL;
	coordinate_mapper_ = NULL;
	color_frame_reader_ = NULL;
	depth_frame_reader_ = NULL;
	m_coordinateMappingChangedEvent = NULL;
	m_pMultiFrameReader = NULL;
	is_collction_ = false;
    //color_buffer_.resize(color_width_*color_height_* sizeof(RGBQUAD));	
	return true;
}
bool KinectCollection::checkCoordinateChange() {
	if (nullptr == kinect_sensor_)
	{
		std::cout << "checkCoordinateChange::cannot get kinect sensor!" << std::endl;
		exit(0);
	}
	//检查相机参数变化
	if (m_coordinateMappingChangedEvent != NULL && WAIT_OBJECT_0 == WaitForSingleObject((HANDLE)m_coordinateMappingChangedEvent, 0))
	{
		std::cout << "checkCoordinateChange::camere corrdinate map chainge!" << std::endl;
		ResetEvent((HANDLE)m_coordinateMappingChangedEvent);
		return true;
	}
	return false;
}
bool KinectCollection::start()
{
	if (!initSensor())
		return false;

	if (!is_collction_)
	{
		is_collction_ = true;
	}
	while (!checkCoordinateChange());
	CameraIntrinsics intrinsics = {};
	coordinate_mapper_->GetDepthCameraIntrinsics(&intrinsics);
	fx_ = intrinsics.FocalLengthX;
	fy_ = intrinsics.FocalLengthY;
	cx_ = intrinsics.PrincipalPointX;
	cy_ = intrinsics.PrincipalPointY;
	disFourth_ = intrinsics.RadialDistortionFourthOrder;
	disSecond_ = intrinsics.RadialDistortionSecondOrder;
	disSixth_ = intrinsics.RadialDistortionSixthOrder;
	return true;
}

void KinectCollection::stop()
{
	is_collction_ = false;
	uninitSensor();
}

void KinectCollection::update()
{
	//if (is_collction_) {
	//	updateColor();
	//	updateDepth();
	//	updateMap();
	//	mapColorToDepth();
	//}

	//clock_t s1, s2;
	if (is_collction_ )
	{
		IDepthFrameReference* m_pDepthFrameReference = NULL;
		IColorFrameReference* m_pColorFrameReference = NULL;
		IDepthFrame* pDepthFrame = NULL;
		IColorFrame* pColorFrame = NULL;
		IMultiSourceFrame* pMultiFrame = nullptr;
		HRESULT hr = m_pMultiFrameReader->AcquireLatestFrame(&pMultiFrame);
		if (SUCCEEDED(hr)) {
			if (SUCCEEDED(hr))
				hr = pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
			if (SUCCEEDED(hr))
				hr = m_pColorFrameReference->AcquireFrame(&pColorFrame);
			if (SUCCEEDED(hr))
				hr = pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
			if (SUCCEEDED(hr))
				hr = m_pDepthFrameReference->AcquireFrame(&pDepthFrame);
			if (SUCCEEDED(hr)) {
				//acquire color image
				color_mat_ = cv::Mat::zeros(color_height_, color_width_, CV_8UC4);
				int nBufferSize = color_width_ * color_height_ * 4;
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(color_mat_.data), ColorImageFormat_Bgra);
			}	
			//update depth image
			if (SUCCEEDED(hr))
			{
				//hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pBuffer); 
				depth_mat_ = cv::Mat::zeros(depth_height_, depth_width_, CV_16UC1);
				hr = pDepthFrame->CopyFrameDataToArray(depth_height_ * depth_width_, reinterpret_cast<UINT16*>(depth_mat_.data));
			}
			if (SUCCEEDED(hr)) {
				updateMap();
				mapColorToDepth();
			}
		}
		SafeReleaseObj(pDepthFrame);
		SafeReleaseObj(pColorFrame);
		SafeReleaseObj(m_pColorFrameReference);
		SafeReleaseObj(m_pDepthFrameReference);
		SafeReleaseObj(pMultiFrame);
	}

}

bool KinectCollection::initSensor()
{
	//uninitSensor();
	//HRESULT hr;
	//hr = GetDefaultKinectSensor(&kinect_sensor_);
	//if (FAILED(hr))
	//{
	//	return false;
	//}
	//if (kinect_sensor_)
	//{
	//	// Initialize the Kinect and get coordinate mapper and the body reader
	//	IColorFrameSource* pColorFrameSource = NULL;
	//	IDepthFrameSource* pDepthFrameSource = NULL;
	//	hr = kinect_sensor_->Open();
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = kinect_sensor_->get_ColorFrameSource(&pColorFrameSource);
	//	}
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = pColorFrameSource->OpenReader(&color_frame_reader_);
	//	}
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = kinect_sensor_->get_DepthFrameSource(&pDepthFrameSource);
	//	}
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = pDepthFrameSource->OpenReader(&depth_frame_reader_);
	//	}
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = kinect_sensor_->get_CoordinateMapper(&coordinate_mapper_);
	//	}
	//	if (!SUCCEEDED(hr)) {
	//		return false;
	//	}
	//	SafeReleaseObj(pColorFrameSource);
	//	SafeReleaseObj(pDepthFrameSource);
	//}
	//if (!kinect_sensor_ || FAILED(hr))
	//{
	//	return false;
	//}

	uninitSensor();
    HRESULT hr;
    hr = GetDefaultKinectSensor(&kinect_sensor_);
    if (FAILED(hr))
    {
		return false;
   }
	if (kinect_sensor_ != NULL) {
		hr = kinect_sensor_->Open();
		if (SUCCEEDED(hr)) {
			hr = kinect_sensor_->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Depth, &m_pMultiFrameReader);
		}
		if (SUCCEEDED(hr))
			hr = kinect_sensor_->get_CoordinateMapper(&coordinate_mapper_);
		if (SUCCEEDED(hr))
			hr = coordinate_mapper_->SubscribeCoordinateMappingChanged(&m_coordinateMappingChangedEvent);
	}
    if (!kinect_sensor_ || FAILED(hr))
    {
       return false;
    }
	return true;
}

bool KinectCollection::uninitSensor()
{
    SafeReleaseObj(color_frame_reader_);
    SafeReleaseObj(depth_frame_reader_);
    SafeReleaseObj(coordinate_mapper_);
	SafeReleaseObj(m_pMultiFrameReader);
	if (nullptr != coordinate_mapper_)
		coordinate_mapper_->UnsubscribeCoordinateMappingChanged(m_coordinateMappingChangedEvent);
   // close the Kinect Sensor
    if (kinect_sensor_)
    {
        kinect_sensor_->Close();
    }
    SafeReleaseObj(kinect_sensor_);
	return true;
}

void KinectCollection::updateDepth()
{
    IDepthFrame* pDepthFrame = NULL;

    HRESULT hr = depth_frame_reader_->AcquireLatestFrame(&pDepthFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        UINT16 *pBuffer = NULL;

        hr = pDepthFrame->get_RelativeTime(&nTime);

		UINT nDepthBufferSize = 0;		//缓存深度图像大小
        if (SUCCEEDED(hr))
        {
            //hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pBuffer); 
			depth_mat_ = cv::Mat::zeros(depth_height_, depth_width_, CV_16UC1);
			hr = pDepthFrame->CopyFrameDataToArray(depth_height_ * depth_width_, reinterpret_cast<UINT16*>(depth_mat_.data));
        }
    //    if (SUCCEEDED(hr))
    //    {
    //        if (depth_mat_.rows != depth_height_ || depth_mat_.cols != depth_width_)
    //        {
				//depth_mat_ = cv::Mat::zeros(depth_height_,depth_width_,CV_16UC1);
    //        }
    //        memcpy(depth_mat_.data,pBuffer,nDepthBufferSize*sizeof(UINT16));
    //    }
        SafeRelease(pFrameDescription);
    }

    SafeRelease(pDepthFrame);
}

void KinectCollection::updateColor()
{
   // RGBQUAD *pBuffer = NULL;
    if (!color_frame_reader_)
		return;   
    IColorFrame* pColorFrame = NULL;
    HRESULT hr = color_frame_reader_->AcquireLatestFrame(&pColorFrame);

    if (SUCCEEDED(hr))
    {
        int nWidth = 0;
        int nHeight = 0;
        UINT nBufferSize = 0;  
		//INT64 nTime = 0;
		//ColorImageFormat imageFormat = ColorImageFormat_None;
		//hr = pColorFrame->get_RelativeTime(&nTime);
		//IFrameDescription* pFrameDescription = NULL;
		//hr = getFrameSize(pColorFrame, &pFrameDescription, nWidth, nHeight);
		//if (SUCCEEDED(hr))
		//{
		//	hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		//}

        if (SUCCEEDED(hr))
        {
			//auto pDstBuffer = &color_buffer_[0];
			//pBuffer = (RGBQUAD*)pDstBuffer;
            nBufferSize = color_width_ * color_height_ * sizeof(RGBQUAD);
			color_mat_ = cv::Mat::zeros(color_height_, color_width_, CV_8UC4);
            hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize
				, reinterpret_cast<BYTE*>(color_mat_.data), ColorImageFormat_Bgra);
			// , reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);     
			//int pixsize = sizeof(RGBQUAD);
			//for (int row = 0;row < color_mat_.rows;row++)
			//{
			//	for (int col = 0;col < color_mat_.cols;col++)
			//	{
			//		auto& dst = color_mat_.at<cv::Vec3b>(row,col);
			//		auto& src = pBuffer[row*color_width_ + col];
			//		dst[0] = src.rgbBlue;
			//		dst[1] = src.rgbGreen;
			//		dst[2] = src.rgbRed;
			//	}
			//}
       }
      //  SafeReleaseObj(pFrameDescription);
    }
    SafeReleaseObj(pColorFrame);
}
void KinectCollection::mapColorToDepth()
{
	if (depth_mat_.data == NULL)
		return;
	if (coordinate_mapper_ != NULL)
	{
		DepthSpacePoint* dsp = new DepthSpacePoint[color_height_ * color_width_];
		HRESULT hr = coordinate_mapper_->MapColorFrameToDepthSpace(depth_height_ * depth_width_, reinterpret_cast<UINT16*>(depth_mat_.data), color_height_ * color_width_, dsp);
		if (SUCCEEDED(hr)) {
			colorToDepthMap_mat_ = cv::Mat(color_height_, color_width_, CV_16SC2, cv::Scalar::all(-1));
			for (int y = 0; y < color_height_; y++)
				for (int x = 0; x < color_width_; x++) {
					int index = y * color_width_ + x;
					DepthSpacePoint p = dsp[index];
					// Values that are negative infinity means it is an invalid color to depth mapping so we
					// skip processing for this pixel
					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
					{
						int depthX = static_cast<int>(p.X + 0.5f);
						int depthY = static_cast<int>(p.Y + 0.5f);
						if ((depthX >= 0 && depthX < depth_width_) && (depthY >= 0 && depthY < depth_height_))
						{
							colorToDepthMap_mat_.at<cv::Vec2s>(y, x) = cv::Vec2s(depthY, depthX);
						}
					}
					else {
						colorToDepthMap_mat_.at<cv::Vec2s>(y, x) = cv::Vec2s(-1, -1);
					}
				}			
		}
		delete[] dsp;
	}

}
void KinectCollection::updateMap()
{
	if (depth_mat_.data == NULL)
		return;
	if (color_mat_.data == NULL)
		return;
    if (coordinate_mapper_ != NULL)
    {
		colorSpace = new ColorSpacePoint[depth_width_ * depth_height_];
	    HRESULT hr = coordinate_mapper_->MapDepthFrameToColorSpace(depth_width_ * depth_height_	, reinterpret_cast<UINT16*>(depth_mat_.data), depth_width_ * depth_height_, colorSpace);
		if (SUCCEEDED(hr)) {
			map_mat_ = cv::Mat::zeros(depth_height_, depth_width_, CV_8UC3);
			for (int y = 0; y < depth_height_; y++)
				for (int x = 0; x < depth_width_; x++) {
					int index = y * depth_width_ + x;
					ColorSpacePoint p = colorSpace[index];
					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity()) {
						int colorX = static_cast<int>(p.X + 0.5f);
						int colorY = static_cast<int>(p.Y + 0.5f);
						if ((colorX >= 0 && colorX < color_width_) && (colorY >= 0 && colorY < color_height_)) {
							map_mat_.at<cv::Vec3b>(y, x) =  cv::Vec3b(color_mat_.at<cv::Vec4b>(colorY, colorX)[0], color_mat_.at<cv::Vec4b>(colorY, colorX)[1], color_mat_.at<cv::Vec4b>(colorY, colorX)[2]);
						}							
					}
				}
		}
		delete[] colorSpace;
    }
}
