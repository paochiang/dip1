/*
Copyright (c) 2013-2015, Gregory P. Meyer
                         University of Illinois Board of Trustees
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Standard Libraries
#include <stdio.h>
#include <stdlib.h>

// OpenGL
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// DIP
#include <dip/cameras/dumpfile.h>
#include <dip/cameras/primesense.h>
#include <dip/cameras/softkinetic.h>

#include <dip/common/types.h>
#include <dip/io/objfile.h>
#include <dip/projects/facemodeling.h>
#include <dip/surface/mesh.h>
#include <dip/visualization/colorize.h>


#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include "rs.hpp"
#include "libfreenect2opencv.h"


//#define KINECT
#define KINECT2
//#define REALSENSE

using namespace dip;
#ifndef KINECT2
int depth_width = 640;
int depth_height = 480;
#else
int depth_width = 960;// 512;
int depth_height = 540;// 424;
#endif
int color_width = 1920;
int color_height = 1080;

/*GLFW*/
//static void key_callback(GLFWwindow* window, int key, int scancode,
//                         int action, int mods) {
//  if ((key == GLFW_KEY_ESCAPE) && (action == GLFW_PRESS))
//    glfwSetWindowShouldClose(window, GL_TRUE);
//}

using namespace std;
using namespace cv;

std::vector<cv::Point3f> ImageToSpace(cv::Mat depth, float fx, float fy, float px, float py) {
	std::vector<cv::Point3f> out;
	for (int r = 0; r < depth.rows; r++) {
		for (int c = 0; c < depth.cols; c++) {
			unsigned short Depth = depth.at<unsigned short>(r, c);
			Point3f pt;
			pt.x = (c - px) / fx * Depth;
			pt.y = (r - py) / fy * Depth;
			pt.z = Depth;
			out.push_back(pt);
		}
	}
	return out;
}

void SaveCloud(std::vector<Point3f> pts, string fileName) {
	int count = 0;
	for (int i = 0; i < pts.size(); i++)
	{
		cv::Point3f p = pts[i];
		if (p.x != -std::numeric_limits<float>::infinity() && p.y != -std::numeric_limits<float>::infinity() && p.z != -std::numeric_limits<float>::infinity() && abs(p.z) > 1e-6)
		{
			count++;
		}
	}

	ofstream ofs;
	ofs.open(fileName);
	if (ofs.is_open()) {
		ofs << "ply\n";
		ofs << "format ascii 1.0\n";
		ofs << "element vertex " << count << "\n";
		ofs << "property float x\n";
		ofs << "property float y\n";
		ofs << "property float z\n";
		ofs << "property float nx\n";
		ofs << "property float ny\n";
		ofs << "property float nz\n";
		ofs << "property uchar diffuse_red\n";
		ofs << "property uchar diffuse_green\n";
		ofs << "property uchar diffuse_blue\n";
		ofs << "property uchar alpha\n";
		ofs << "end_header\n";
		for (int i = 0; i < pts.size(); i++)
		{
			cv::Point3f p = pts[i];
			if (p.x != -std::numeric_limits<float>::infinity() && p.y != -std::numeric_limits<float>::infinity() && p.z != -std::numeric_limits<float>::infinity() && abs(p.z) > 1e-6)
			{
				float cameraX = static_cast<float>(p.x);
				float cameraY = static_cast<float>(p.y);
				float cameraZ = static_cast<float>(p.z);
				ofs << cameraX << " " << cameraY << " " << cameraZ << " ";
				ofs << "0 0 0 ";
				ofs << "255 255 255 ";
				ofs << "255" << endl;
			}
		}
		ofs.close();
		cout << "depthtocloud ok!" << endl;

	}

}

int main(int argc, char **argv) {
  if ((argc < 2) || (argc > 3)) {
    printf("Usage: %s <Mesh File> [Dump File]\n", argv[0]);
    return -1;
  }

#ifdef KINECT
  /*kinect*/
  // Initialize Camera
  Camera *camera = NULL;
  if (argc < 3) {
#ifndef SOFTKINETIC
    camera = new PrimeSense();
#else
    camera = new SoftKinetic();
#endif
  } else {
    camera = new DumpFile(argv[2]);
  }

  if (!camera->enabled()) {
    printf("Unable to Open Camera\n");
    return -1;
  }
#endif
#ifdef REALSENSE
  ///*realsense*/
  rs::device *dev;
  rs::context ctx;
  int device_idx_ = 0;
  if (ctx.get_device_count() == 0)
  {
	  std::cout << "No device detected. Is it plugged in?" << std::endl;
	  return -1;
  }
  // Enumerate all devices
  std::vector<rs::device *> devices;
  for (int i = 0; i<ctx.get_device_count(); ++i)
  {
	  devices.push_back(ctx.get_device(i));
  }
  sort(devices.begin(), devices.end(), [](rs::device * l, rs::device *r) {return strcmp(l->get_serial(), r->get_serial()) < 0; });
  dev = nullptr;
  if (device_idx_ >= 0 && device_idx_ < devices.size())
  {
	  dev = devices[device_idx_];
	  std::cout << "Starting " << dev->get_serial() << "... ";
	  dev->enable_stream(rs::stream::depth, depth_width, depth_height, rs::format::z16, 30);
	  dev->enable_stream(rs::stream::color, color_width, color_height, rs::format::bgr8, 30);
	 // dev->enable_stream(rs::stream::infrared, depth_width, depth_height, rs::format::y8, 30);
	  dev->start();
  }
  else {
	  std::cout << "please check device!" << std::endl;
  }
  rs::intrinsics depth_intri = dev->get_stream_intrinsics(rs::stream::depth);
  cout << "============depth==============" << endl;
  cout << "fx:" << depth_intri.fx << ",fy: " << depth_intri.fy << "px:" << depth_intri.ppx << ", py:" << depth_intri.ppy << endl;
  cout << "distortion:" << depth_intri.coeffs[0] << "," << depth_intri.coeffs[1] << "," << depth_intri.coeffs[2] << "," << depth_intri.coeffs[3] << "," << depth_intri.coeffs[4] << endl;
  cout << "============color==============" << endl;
  rs::intrinsics color_intri = dev->get_stream_intrinsics(rs::stream::color);
  cout << "fx:" << color_intri.fx << ", fy:" << color_intri.fy << ",ppx:" << color_intri.ppx << ",ppy:" << color_intri.ppy << endl;
  cout << "distortion:" << color_intri.coeffs[0] << "," << color_intri.coeffs[1] << "," << color_intri.coeffs[2] << "," << color_intri.coeffs[3] << "," << color_intri.coeffs[4] << endl;
  float depth_scale = dev->get_depth_scale();
  cout << "depth scale:" << dev->get_depth_scale() << endl;
  string parameterFileName = "./parameter.txt";
  ofstream ofs;
  ofs.open(parameterFileName);
  if (ofs.is_open()) {
	  ofs << "depth camera parameter:" << endl;
	  ofs << "fx: " << depth_intri.fx << endl;
	  ofs << "fy: " << depth_intri.fy << endl;
	  ofs << "px: " << depth_intri.ppx << endl;
	  ofs << "py: " << depth_intri.ppy << endl;
	  ofs << "distorttion: " << depth_intri.coeffs[0] << " " << depth_intri.coeffs[1] << " " << depth_intri.coeffs[2] << " " << depth_intri.coeffs[3] << " " << depth_intri.coeffs[4] << endl<<endl;
	  ofs << "color camera parameter:" << endl;
	  ofs << "fx: " << color_intri.fx << endl;
	  ofs << "fy: " << color_intri.fy << endl;
	  ofs << "px: " << color_intri.ppx << endl;
	  ofs << "py: " << color_intri.ppy << endl;
	  ofs << "distorttion: " << color_intri.coeffs[0] << " " << color_intri.coeffs[1] << " " << color_intri.coeffs[2] << " " << color_intri.coeffs[3] << " " << color_intri.coeffs[4] << endl<<endl;
	  ofs.close();
  }
#endif
#ifdef KINECT2
  libfreenect2opencv::Libfreenect2OpenCV libfree;
#endif
  int mes_count = 0;
loop:
#ifdef KINECT
  /*kinect*/
  FaceModeling *modeling = new FaceModeling(
      camera->width(DEPTH_SENSOR), camera->height(DEPTH_SENSOR),
      camera->fx(DEPTH_SENSOR), camera->fy(DEPTH_SENSOR),
	 // 254.4063f, 210.3299f);
      camera->width(DEPTH_SENSOR) / 2.0f, camera->height(DEPTH_SENSOR) / 2.0f);
  // Initialize Buffers
  Depth *depth = new Depth[camera->width(DEPTH_SENSOR) *
                           camera->height(DEPTH_SENSOR)];
  Color *colorized_depth = new Color[camera->width(DEPTH_SENSOR) *
                                     camera->height(DEPTH_SENSOR)];
  Color *color = new Color[camera->width(COLOR_SENSOR) *
                           camera->height(COLOR_SENSOR)];
  Color *normals = new Color[camera->width(DEPTH_SENSOR) *
                             camera->height(DEPTH_SENSOR)];
  depth_width = camera->width(DEPTH_SENSOR);
  depth_height = camera->height(DEPTH_SENSOR);
  color_width = camera->width(COLOR_SENSOR);
  color_height = camera->height(COLOR_SENSOR);
#endif
#ifdef REALSENSE
  /*realsense*/
  //// Initialize 3D face modeling.
  FaceModeling *modeling = new FaceModeling(
	  depth_width, depth_height,
	  depth_intri.fx, depth_intri.fy,
	  depth_intri.ppx, depth_intri.ppy);
  // Initialize Buffers
  Depth *depth = new Depth[depth_width * depth_height];
  Color *color = new Color[color_width * color_height];
  Color *colorized_depth = new Color[depth_width * depth_height];
  Color *normals = new Color[depth_width * depth_height];
#endif
#ifdef KINECT2
  libfreenect2::Freenect2Device::ColorCameraParams color_intri = libfree.getColorCameraParams();
  FaceModeling *modeling = new FaceModeling(
	  depth_width, depth_height,
	  //368.114, 368.114,
	  //258.342, 203.319);
	  depth_width/(color_width / color_intri.fx), depth_height/(color_height / color_intri.fy),
	  depth_width/2, depth_height/2);
  // Initialize Buffers
  Depth *depth = new Depth[depth_width * depth_height];
  Color *color = new Color[color_width * color_height];
  Color *colorized_depth = new Color[depth_width * depth_height];
  Color *normals = new Color[depth_width * depth_height];
  string parameterFileName = "./parameter.txt";
  ofstream ofs;
  ofs.open(parameterFileName);
  if (ofs.is_open()) {
	  libfreenect2::Freenect2Device::IrCameraParams depth_intri = libfree.getIRCameraParams();
	  ofs << "depth camera parameter:" << endl;
	  ofs << "fx: " << depth_intri.fx << endl;
	  ofs << "fy: " << depth_intri.fy << endl;
	  ofs << "px: " << depth_intri.cx << endl;
	  ofs << "py: " << depth_intri.cy << endl;
	  ofs << "distorttion: " << depth_intri.k1 << " " << depth_intri.k2 << " " << depth_intri.k3 << " " << depth_intri.p1 << " " << depth_intri.p2 << endl << endl;
	  libfreenect2::Freenect2Device::ColorCameraParams color_intri = libfree.getColorCameraParams();
	  ofs << "color camera parameter:" << endl;
	  ofs << "fx: " << color_intri.fx << endl;
	  ofs << "fy: " << color_intri.fy << endl;
	  ofs << "px: " << color_intri.cx << endl;
	  ofs << "py: " << color_intri.cy << endl;
	  ofs << "fx2:" << depth_width / (color_width / color_intri.fx) << endl;
	  ofs << "fy2:" << depth_height / (color_height / color_intri.fy) << endl;
	  ofs << "px2: " << depth_width/2 << endl;
	  ofs << "py2: " << depth_height/2 << endl;
	  //cout << "width/fx: " << color_width / color_intri.fx << ", height/fy:" << color_height / color_intri.fy << endl;
	  ofs.close();
  }
#endif
  /*GLFW part*/
  //// Initialize GLFW
  //if (!glfwInit()) {
  //  printf("Unable to Initialize GLFW.\n");
  //  return -1;
  //}
  //GLFWwindow *window = glfwCreateWindow(kWindowWidth * 3, kWindowHeight,
  //                                      "3D Face Modeling", NULL, NULL);
  //if (!window) {
  //  printf("Unable to create window.\n");
  //  glfwTerminate();
  //  return -1;
  //}
  //glfwMakeContextCurrent(window);
  //glfwSetKeyCallback(window, key_callback);
  //// Initialize Texture
  //GLuint textures[3];
  //glEnable(GL_TEXTURE_2D);
  //glGenTextures(3, textures);
  //for (int i = 0; i < 3; i++) {
  //  glBindTexture(GL_TEXTURE_2D, textures[i]);
  //  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  //  if (i == 0) {
  //    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
  //    //             camera->width(COLOR_SENSOR), camera->height(COLOR_SENSOR),
  //    //             0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
  //  } else {
  //    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
  //                 camera->width(DEPTH_SENSOR), camera->height(DEPTH_SENSOR),
  //                 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
  //  }
  //}
 
 unsigned int count = 0;
 int count_ = 1;        //number the saved color and depth images
 int c_w = color_width; //camera->width(COLOR_SENSOR);
 int c_h = color_height;// camera->height(COLOR_SENSOR);
 Colorize colorize;
 bool flag = false;
 cv::Mat normal_out = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);
 clock_t start, end;
 start = clock();
  while (1) {							//while (!glfwWindowShouldClose(window)) {	
	  //end = clock();
	  //double dur = (double)(end - start);
	  //if (dur >= 3000) {
		 // cout << "seconds:" << static_cast<int>(dur / CLOCKS_PER_SEC) << endl;
		 // cout << "count:" << count << endl;
		 // break;
	  //}
#ifdef KINECT
	  /*kinect*/
	 // Update depth image.
	  if (camera->Update(depth)) {
		  printf("Unable to update depth image.\n");
		  break;
	  }
	  //cv::Mat depth_show(depth_height, depth_width, CV_8UC1);
	  //for (int r = 0; r < depth_height; r++) {
		 // for (int c = 0; c < depth_width; c++) {
			//  int index = r * depth_width + c;
			//  depth_show.at<uchar>(r, c) = depth[index] * 255/ 4096;
		 // }
	  //}
	  //cv::imshow("depth", depth_show);

	  // Update color image.
	  if (camera->Update(color)) {
		  printf("Unable to update color image.\n");
		  break;
	  }
	  cv::Mat color_img = cv::Mat(color_height, color_width, CV_8UC3);
	  for (int r = 0; r < c_h; r++) {
		  for (int c = 0; c < c_w; c++) {
			  int index = r * c_w + c;
			  color_img.at<cv::Vec3b>(r, c)[0] = color[index].b;
			  color_img.at<cv::Vec3b>(r, c)[1] = color[index].g;
			  color_img.at<cv::Vec3b>(r, c)[2] = color[index].r;
		  }
	  }
	  //cv::Mat color_show;
	  //cv::resize(c_out, color_show, cv::Size(480, 320));
	  //cv::imshow("RGB", color_show);
#endif
#ifdef REALSENSE
	  /*realsense*/
	  dev->poll_for_frames();
	  const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev->get_frame_data(rs::stream::depth));
	  const cv::Vec3b * color_frame = reinterpret_cast<const cv::Vec3b *>(dev->get_frame_data(rs::stream::color));
	  const cv::Vec3b * color_align_depth_frame = reinterpret_cast<const cv::Vec3b *>(dev->get_frame_data(rs::stream::color_aligned_to_depth));
	  const uint16_t * depth_align_color_frame = reinterpret_cast<const uint16_t *>(dev->get_frame_data(rs::stream::depth_aligned_to_color));
	  //const uchar* ir_frame = reinterpret_cast<const uchar *>(dev->get_frame_data(rs::stream::infrared));
	  cv::Mat color_img = cv::Mat(color_height, color_width, CV_8UC3, (void*)color_frame).clone();
	  cv::Mat color_align_depth_img = cv::Mat(depth_height, depth_width, CV_8UC3, (void*)color_align_depth_frame).clone();
	  cv::Mat depth_img = cv::Mat(depth_height, depth_width, CV_16UC1, (void*)depth_frame).clone();
	  cv::Mat depth_align_color_img = cv::Mat(color_height, color_width, CV_16UC1, (void*)depth_align_color_frame).clone();
	  cv::Mat depth_undistortion_img;
	  cv::Mat color_align_depth_undistortion_img;
	  float in[9] = { depth_intri.fx, 0, depth_intri.ppx, 0, depth_intri.fy, depth_intri.ppy, 0 , 0, 1 };
	  cv::Mat intr(3, 3, CV_32FC1, in);
	  cv::Mat coe(1, 5, CV_32FC1, depth_intri.coeffs);
	  cv::undistort(depth_img, depth_undistortion_img, intr, coe);
	  cv::undistort(color_align_depth_img, color_align_depth_undistortion_img, intr, coe);
	  for (int y = 0; y < depth_height; y++)
		  for (int x = 0; x < depth_width; x++)
		  {
			  int index = y * depth_width + x;
			  depth_img.at<unsigned short>(y, x) = depth_img.at<unsigned short>(y, x) * depth_scale * 1000;
			  depth_undistortion_img.at<unsigned short>(y, x) = depth_undistortion_img.at<unsigned short>(y, x) * depth_scale * 1000;
			  depth[index] = depth_undistortion_img.at<unsigned short>(y, x);
		  }
	  //imshow("depth", depth_img);
	  //imshow("undistortion_depth", depth_undistortion_img);
	  //imshow("align_undistortion_color", color_align_depth_undistortion_img);
	  for (int y = 0; y < color_height; y++)
		  for (int x = 0; x < color_width; x++)
		  {
			  int index = y * color_width + x;
			  depth_align_color_img.at<unsigned short>(y, x) = depth_align_color_img.at<unsigned short>(y, x) * depth_scale * 1000;
			  color[index].r = color_img.at<cv::Vec3b>(y, x)[0];
			  color[index].g = color_img.at<cv::Vec3b>(y, x)[1];
			  color[index].b = color_img.at<cv::Vec3b>(y, x)[2];
		  }
#endif
#ifdef KINECT2
	  libfree.updateMat();
	  cv::Mat color_img;
	  cv::Mat depth_img;
	  cv::Mat depth_img_16U = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);
	  cv::Mat color_align_depth_img;
	  libfree.getRGBMat().copyTo(color_img);
	  //libfree.getDepthMat().copyTo(depth_img);
	  cv::resize(libfree.getDepth2RGB(), depth_img, cv::Size(depth_width, depth_height));
	  libfree.getRGB2Depth().copyTo(color_align_depth_img);
	  for (int y = 0; y < depth_height; y++)
		  for (int x = 0; x < depth_width; x++)
		  {
			  int index = y * depth_width + x;
			  if (isnan(depth_img.at<float>(y, x))
				  || depth_img.at<float>(y, x) < 0
				  || depth_img.at<float>(y, x) == std::numeric_limits<float>::infinity()
				  || depth_img.at<float>(y, x) == -std::numeric_limits<float>::infinity())
				  depth_img.at<float>(y, x) = 0;
			  depth[index] = static_cast<unsigned short>(depth_img.at<float>(y, x));
			  depth_img_16U.at<unsigned short>(y, x) = static_cast<unsigned short>(depth_img.at<float>(y, x));
		  }
	  for (int y = 0; y < color_height; y++)
		  for (int x = 0; x < color_width; x++)
		  {
			  int index = y * color_width + x;
			  color[index].b = color_img.at<cv::Vec3b>(y, x)[0];
			  color[index].g = color_img.at<cv::Vec3b>(y, x)[1];
			  color[index].r = color_img.at<cv::Vec3b>(y, x)[2];
		  }
#endif
	  //// Update Model
	  Eigen::Matrix4f transform, global_to_camera;
	  int rrr = modeling->Run(depth, normals, &transform);
	  global_to_camera = transform.inverse();
#ifdef RUNMODEL
	  if (!flag && (rrr == 0)) {
		  cout << "mark first color image" << endl;
		  //color_img.copyTo(firtFrameColor);
		  std::vector<int> compression_params;
		  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		  compression_params.push_back(0);    // ÎÞÑ¹Ëõpng.
		  cv::imwrite("./color0.png", color_img, compression_params);
		  //cv::imwrite("./depth_raw0.png", depth_img_16U, compression_params);
#ifdef REALSENSE
		  cv::imwrite("./color_align_depth0.png", color_align_depth_img, compression_params);
		  cv::imwrite("./color_align_depth_undistortion0.png", color_align_depth_undistortion_img, compression_params);
		 
		  cv::imwrite("./depth_undistortion0.png", depth_undistortion_img,compression_params);
		  cv::imwrite("./depth_align_color0.png", depth_align_color_img, compression_params);
		  ofstream ofs;
		  ofs.open(parameterFileName, ios::app);
		  if (ofs.is_open()) {
			  ofs << "mesh to depth0 transform:" << endl;
			  ofs << global_to_camera(0, 0) << " " << global_to_camera(0, 1) << " " << global_to_camera(0, 2) << " " << global_to_camera(0, 3) << endl;
			  ofs << global_to_camera(1, 0) << " " << global_to_camera(1, 1) << " " << global_to_camera(1, 2) << " " << global_to_camera(1, 3) << endl;
			  ofs << global_to_camera(2, 0) << " " << global_to_camera(2, 1) << " " << global_to_camera(2, 2) << " " << global_to_camera(2, 3) << endl;
			  ofs << global_to_camera(3, 0) << " " << global_to_camera(3, 1) << " " << global_to_camera(3, 2) << " " << global_to_camera(3, 3) << endl<<endl;
			  ofs.close();
		  }
#endif
#ifdef KINECT2
		  //cv::imwrite("./color_align_depth0.png", color_align_depth_img, compression_params);
		  ofstream ofs;
		  ofs.open(parameterFileName, ios::app);
		  if (ofs.is_open()) {
			  ofs << "mesh to depth0 transform:" << endl;
			  ofs << global_to_camera(0, 0) << " " << global_to_camera(0, 1) << " " << global_to_camera(0, 2) << " " << global_to_camera(0, 3) << endl;
			  ofs << global_to_camera(1, 0) << " " << global_to_camera(1, 1) << " " << global_to_camera(1, 2) << " " << global_to_camera(1, 3) << endl;
			  ofs << global_to_camera(2, 0) << " " << global_to_camera(2, 1) << " " << global_to_camera(2, 2) << " " << global_to_camera(2, 3) << endl;
			  ofs << global_to_camera(3, 0) << " " << global_to_camera(3, 1) << " " << global_to_camera(3, 2) << " " << global_to_camera(3, 3) << endl << endl;
			  ofs.close();
		  }
#endif
		  flag = true;
	  }
	  if (rrr == 1) {
		  stringstream ss;
		  ss << mes_count << endl;
		  string out;
		  ss >> out;
		  string outfileName = "./mesh" + out + ".obj";
		  OBJFile obj_file(outfileName.c_str(), CREATE_OBJ);
		  if (obj_file.enabled()) {
			  Mesh mesh;
			  modeling->Model(&mesh);
			  obj_file.Write(&mesh);
		  }
		  mes_count++;
		  normal_out = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);
		  cv::destroyWindow("model");
		  // delete camera;
		  delete modeling;
		  delete[] depth;
		  delete[] colorized_depth;
		  delete[] color;
		  delete[] normals;		  
		  goto loop;
	  } 
	  //// Colorize depth image.
	  //colorize.Run(depth_width, depth_height, depth, colorized_depth);
	  ////colored depth
	  //cv::Mat cd_out(depth_height, depth_width, CV_8UC3);
	  //for (int r = 0; r < depth_height; r++) {
		 // for (int c = 0; c < depth_width; c++) {
			//  int index = r * depth_width + c;
			//  cd_out.at<cv::Vec3b>(r, c)[0] = colorized_depth[index].b;
			//  cd_out.at<cv::Vec3b>(r, c)[1] = colorized_depth[index].g;
			//  cd_out.at<cv::Vec3b>(r, c)[2] = colorized_depth[index].r;
		 // }
	  //}
	  //cv::imshow("color_depth", cd_out);
	  for (int r = 0; r < depth_height; r++) {
		  for (int c = 0; c < depth_width; c++) {
			  int index = r * depth_width + c;
			  normal_out.at<cv::Vec3b>(r, c)[0] = normals[index].b;
			  normal_out.at<cv::Vec3b>(r, c)[1] = normals[index].g;
			  normal_out.at<cv::Vec3b>(r, c)[2] = normals[index].r;
		  }
	  }
	  cv::imshow("model", normal_out);
	  cv::moveWindow("model", 600 + depth_width + 10, 60);
	  int res = cv::waitKey(1);
	if (res == 32) {
		//OBJFile obj_file(argv[1], CREATE_OBJ);
		//if (obj_file.enabled()) {
		//	Mesh mesh;
		//	modeling->Model(&mesh);
		//	obj_file.Write(&mesh);
		//}
		stringstream ss;
		ss << count_ << endl;
		string out;
		ss >> out;
		std::vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(0);    // ÎÞÑ¹Ëõpng.
		cv::imwrite("./color" + out + ".png", color_img, compression_params);
		//cv::imwrite("./depth_raw" + out + ".png", depth_img_16U, compression_params);
#ifdef REALSENSE		
		cv::imwrite("./color_align_depth_undistortion" + out + ".png", color_align_depth_undistortion_img, compression_params);		
		cv::imwrite("./depth_undistortion" + out + ".png", depth_undistortion_img, compression_params);
		cv::imwrite("./depth_align_color" + out + ".png", depth_align_color_img, compression_params);
		ofstream ofs;
		ofs.open(parameterFileName, ios::app);
		if (ofs.is_open()) {
			ofs << "mesh to depth"<<out<< " transform:" << endl;
			ofs << global_to_camera(0, 0) << " " << global_to_camera(0, 1) << " " << global_to_camera(0, 2) << " " << global_to_camera(0, 3) << endl;
			ofs << global_to_camera(1, 0) << " " << global_to_camera(1, 1) << " " << global_to_camera(1, 2) << " " << global_to_camera(1, 3) << endl;
			ofs << global_to_camera(2, 0) << " " << global_to_camera(2, 1) << " " << global_to_camera(2, 2) << " " << global_to_camera(2, 3) << endl;
			ofs << global_to_camera(3, 0) << " " << global_to_camera(3, 1) << " " << global_to_camera(3, 2) << " " << global_to_camera(3, 3) << endl<<endl;
			ofs.close();
		}
#endif
#ifdef KINECT2
		//cv::imwrite("./color_align_depth" + out + ".png", color_align_depth_img, compression_params);
		ofstream ofs;
		ofs.open(parameterFileName, ios::app);
		if (ofs.is_open()) {
			ofs << "mesh to depth" << out << " transform:" << endl;
			ofs << global_to_camera(0, 0) << " " << global_to_camera(0, 1) << " " << global_to_camera(0, 2) << " " << global_to_camera(0, 3) << endl;
			ofs << global_to_camera(1, 0) << " " << global_to_camera(1, 1) << " " << global_to_camera(1, 2) << " " << global_to_camera(1, 3) << endl;
			ofs << global_to_camera(2, 0) << " " << global_to_camera(2, 1) << " " << global_to_camera(2, 2) << " " << global_to_camera(2, 3) << endl;
			ofs << global_to_camera(3, 0) << " " << global_to_camera(3, 1) << " " << global_to_camera(3, 2) << " " << global_to_camera(3, 3) << endl << endl;
			ofs.close();
		}
#endif
		/*out put data*/
		//cv::imwrite("./depth_" + out + ".png", depth_img, compression_params);
		//std::vector<cv::Point3f> points3d = ImageToSpace(depth_img, focal.x, focal.y, pxy.x, pxy.y);
		//SaveCloud(points3d, "./cloud_depth" + out + ".ply");
		count_++;
		printf("saved ok!============================================================================\n");
	}
	else if(res == VK_ESCAPE){
		break;
	}
	count++;
#endif	 

	/*GLFW*/
    //glfwMakeContextCurrent(window);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glMatrixMode(GL_PROJECTION);
    //glLoadIdentity();
    //glOrtho(0.0f, 1.0f, 0.0f, 1.0f, -10.0f, 10.0f);
    //glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();
    //glViewport(0, 0, kWindowWidth, kWindowHeight);
    //glBindTexture(GL_TEXTURE_2D, textures[0]);
    //glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
    //                camera->width(COLOR_SENSOR), camera->height(COLOR_SENSOR),
    //                GL_RGB, GL_UNSIGNED_BYTE, color);
    //glBegin(GL_QUADS);
    //  glTexCoord2f(0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f);
    //  glTexCoord2f(0.0f, 0.0f); glVertex3f(0.0f, 1.0f, 0.0f);
    //  glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, 1.0f, 0.0f);
    //  glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, 0.0f, 0.0f);
    //glEnd();
    //glViewport(kWindowWidth, 0, kWindowWidth, kWindowHeight);
    //glBindTexture(GL_TEXTURE_2D, textures[1]);
    //glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
    //                camera->width(DEPTH_SENSOR), camera->height(DEPTH_SENSOR),
    //                GL_RGB, GL_UNSIGNED_BYTE, colorized_depth);
    //glBegin(GL_QUADS);
    //  glTexCoord2f(0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f);
    //  glTexCoord2f(0.0f, 0.0f); glVertex3f(0.0f, 1.0f, 0.0f);
    //  glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, 1.0f, 0.0f);
    //  glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, 0.0f, 0.0f);
    //glEnd();
    //glViewport(2 * kWindowWidth, 0, kWindowWidth, kWindowHeight);
    //glBindTexture(GL_TEXTURE_2D, textures[2]);
    //glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
    //                camera->width(DEPTH_SENSOR), camera->height(DEPTH_SENSOR),
    //                GL_RGB, GL_UNSIGNED_BYTE, normals);
    //glBegin(GL_QUADS);
    //  glTexCoord2f(0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f);
    //  glTexCoord2f(0.0f, 0.0f); glVertex3f(0.0f, 1.0f, 0.0f);
    //  glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, 1.0f, 0.0f);
    //  glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, 0.0f, 0.0f);
    //glEnd();
    //glfwSwapBuffers(window);
    //glfwPollEvents();
  }

  /*GLFW*/
  //glfwDestroyWindow(window);
  //glfwTerminate();

  OBJFile obj_file(argv[1], CREATE_OBJ);
  if (obj_file.enabled()) {
    Mesh mesh;
    modeling->Model(&mesh);

    obj_file.Write(&mesh);
  }
 // delete camera;
  delete modeling;
  delete [] depth;
  delete [] colorized_depth;
  delete [] color;
  delete [] normals;

  return 0;
}
