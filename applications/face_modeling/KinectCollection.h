/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 13:16
Description	: kinect���ݲɼ�
**************************************************************************/
#ifndef KINECTCOLLECTION_H_
#define KINECTCOLLECTION_H_
#include <Kinect.h>
class KinectCollection
{
public:
	KinectCollection();
	~KinectCollection();
	bool init(int idx);
	bool start();
	void stop();
	void update();
	cv::Mat getColorMat() { return color_mat_; };
	cv::Mat getDepthMat() { return depth_mat_; };
	cv::Mat getDepthWithColorMat() { return map_mat_; };
	cv::Mat getColorToDepthMapMat() { return colorToDepthMap_mat_; };
private:
	bool initSensor();
	bool uninitSensor();
	void updateDepth();
	void updateColor();
	void updateMap();
	void mapColorToDepth();
	bool checkCoordinateChange();

private:
    IKinectSensor*                kinect_sensor_;
	IMultiSourceFrameReader*	  m_pMultiFrameReader;
    ICoordinateMapper*            coordinate_mapper_;
    IColorFrameReader*            color_frame_reader_;
    IDepthFrameReader*            depth_frame_reader_;
	WAITABLE_HANDLE               m_coordinateMappingChangedEvent;//����ϵӳ��任��־

    static const  int                           depth_width_ = 512;			//���ͼ���
    static const int                           depth_height_ = 424;		//���ͼ���

	static const int              color_width_  = 1920;
    static const int              color_height_ = 1080;
    // sensor�򿪱�־λ
    bool                          is_collction_;

    std::vector<unsigned char>    color_buffer_; //�����ɫͼ��Ļ���
	cv::Mat                       color_mat_;
	cv::Mat                       depth_mat_;
	cv::Mat                       map_mat_;
	cv::Mat						  colorToDepthMap_mat_;
	ColorSpacePoint*              colorSpace;
};


#endif //KINECTCOLLECTION_H_
