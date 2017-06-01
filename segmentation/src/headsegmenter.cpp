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

#include <dip/segmentation/headsegmenter.h>

#include <string.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include<string>

//#define SHOWCOUT

//const float foreground_size_percent = 0.2 * 0.2;
//const float head_center_left_percent = 0.3;
//const float head_center_right_percent = 0.7;
//const float head_top_up_percent = 0.2;
//const float head_top_down_percent = 0.7;

const float foreground_size_percent = 0.1 * 0.1;
const float head_center_left_percent = 0.2;
const float head_center_right_percent = 0.8;
const float head_top_up_percent = 0.05;
const float head_top_down_percent = 0.95;
int count_ = 0;

namespace dip {

HeadSegmenter::~HeadSegmenter() {
  if (labels_ != NULL)
    delete [] labels_;
  if (column_histogram_ != NULL)
    delete [] column_histogram_;
  if (row_histogram_ != NULL)
    delete [] row_histogram_;
}

int HeadSegmenter::Run(int min_depth, int max_depth, int max_difference,
	int min_width, int min_height,
	int max_width, int max_height,
	float fx, float fy, int width, int height,
	const Depth *depth, Depth *segmented_depth, int head_pose) {
	if (head_pose == -1) {////右转90°
		Depth* turnRightDepth = new Depth[width * height];
		int index = 0;
		for (int x = 0; x < width; x++)
		{
			for (int y = height - 1; y >= 0; y--, index++)
			{
				int src = y * width + x;
				turnRightDepth[index] = depth[src];
			}
		}
		Depth* turnRightDepth_seg_ = new Depth[width * height];
		// Segment the user's head from the depth image.
		int res = Run(min_depth, max_depth, max_difference,
			min_width, min_height,
			max_width, max_height,
			fy, fx, height, width, turnRightDepth, turnRightDepth_seg_);
		if (res == 0) {
			index = 0;
			for (int x = height - 1; x >= 0; x--)
				for (int y = 0; y < width; y++, index++) {
					int src = y * height + x;
					segmented_depth[index] = turnRightDepth_seg_[src];
				}
		}
		delete[] turnRightDepth;
		delete[] turnRightDepth_seg_;
		return res;
	}
	if (head_pose == 1) {////左转90°
		Depth* turnLeftDepth = new Depth[width * height];
		int index = 0;
		for (int x = width - 1; x >= 0; x--)
		{
			for (int y = 0; y < height; y++, index++)
			{
				int src = y * width + x;
				turnLeftDepth[index] = depth[src];
			}
		}
		Depth* turnLeftDepth_seg_ = new Depth[width * height];
		// Segment the user's head from the depth image.
		int res = Run(min_depth, max_depth, max_difference,
			min_width, min_height,
			max_width, max_height,
			fy, fx, height, width, turnLeftDepth, turnLeftDepth_seg_);
		
		if (res == 0) {
			index = 0;
			for (int x = 0; x < height; x++)
			{
				for (int y = width - 1; y >= 0; y--, index++)
				{
					int src = y * height + x;
					segmented_depth[index] = turnLeftDepth_seg_[src];
				}
			}
		}
		delete[] turnLeftDepth;
		delete[] turnLeftDepth_seg_;
		return res;
	}
	else if (head_pose == 0) {
		int res = Run(min_depth, max_depth, max_difference,
			min_width, min_height,
			max_width, max_height,
			fx, fy, width, height, depth, segmented_depth);
		return res;
	}
	else {
		return -1;
	}
}

int HeadSegmenter::Run(int min_depth, int max_depth, int max_difference,
                       int min_width, int min_height,
                       int max_width, int max_height,
                       float fx, float fy, int width, int height,
                       const Depth *depth, Depth *segmented_depth) {
  if (size_ < (width * height)) {
    size_ = width * height;

	if (labels_ != NULL)
	{
		//delete[] labels_;
		memset(labels_, 0, sizeof(int) * size_);
	}
     
	else 
		labels_ = new int[size_];
  }

  if (width_ < width) {
    width_ = width;

	if (column_histogram_ != NULL) {
		//delete[] column_histogram_;
		memset(column_histogram_, 0, sizeof(int) * width_);
	}      
	else 
		column_histogram_ = new int[width_];
  }

  if (height_ < height) {
    height_ = height;

	if (row_histogram_ != NULL) {
		//delete[] row_histogram_;
		memset(row_histogram_, 0, sizeof(int) * height_);
	}    
	else
		row_histogram_ = new int[height_];
  }

  memset(segmented_depth, 0, sizeof(Depth) * width * height);
  
  Depth *depth_ = new Depth[width * height];
  for (int r = 0; r < height; r++) {
	  for (int c = 0; c < width; c++) {
		  unsigned index = r * width + c;
		  if (depth[index] < min_depth || depth[index] > max_depth)
			  depth_[index] = 0;
		  else
			  depth_[index] = depth[index];
	  }
  }
  //// Determine foreground region
  connected_components_.Run(max_difference, width, height, depth_, components_,
                            labels_);
  delete[] depth_;

  int foreground_id = 0;
  int foreground_size = 0;
  int foreground_depth = 0;
 
  for (unsigned int n = 0; n < components_.size(); n++) {
	  if (components_.at(n).root) {
		  if (components_.at(n).size > foreground_size && components_.at(n).mean > min_depth) {
			  foreground_id = components_.at(n).parent;
			  foreground_size = components_.at(n).size;
			  foreground_depth = components_.at(n).mean;
		  }
	  }
  }
  int head_center = 0;
  int head_bottom = 0;
  int head_left = 0;
  int head_right = 0;
  int head_top = 0;
  cv::Mat depth_foreground = cv::Mat::zeros(height_, width_, CV_8UC1);
  for (int r = 0; r < height_; r++) {
	  for (int c = 0; c < width_; c++) {
		  int index = r * width_ + c;
		  if (labels_[index] == foreground_id) {
			  depth_foreground.at<uchar>(r, c) = 255;
		  }
	  }
  }
  cv::imshow("depth_foreground", depth_foreground);
  cv::moveWindow("depth_foreground", 600, 40);
  // Determine head region
  memset(column_histogram_, 0, sizeof(int) * width);
  memset(row_histogram_, 0, sizeof(int) * height);
  std::vector<float> row_histogram_value(height, 0);

  // Generate Histograms
  int count = 0;  //头部+躯干区域占据的像素总数
  int i = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++, i++) {
      if (labels_[i] == foreground_id) {
        column_histogram_[x]++;
        row_histogram_[y]++;
		row_histogram_value[y] += depth[y * width + x];
        count++;
      }
    }
  }
  if (foreground_depth > max_depth || foreground_depth < min_depth) {
#ifdef SHOWCOUT
	  std::cout << "foreground depth value is not between " << min_depth << "---" << max_depth << std::endl;
#endif
	  return -1;
  }
  if (foreground_size < width_ * height_ * foreground_size_percent) {
#ifdef SHOWCOUT
	  std::cout << "foreground size:"<<foreground_size<<" is too small!" << std::endl;
#endif
	  return -1;
  }
  // Locate Top of Head
  int max_value = 0;
  for (int x = 0; x < width; x++) {
    if (column_histogram_[x] > max_value) {
      max_value = column_histogram_[x];
      head_center = x;
    }
  }
  for (int y = 0; y < height; y++) {
    int i = head_center + y * width;

    if (labels_[i] == foreground_id) {
      head_top = y;
      break;
    }
  }

  if ((head_center > width_ * head_center_right_percent) || (head_center < width_ * head_center_left_percent)) {
#ifdef SHOWCOUT
	  std::cout << "too left or right!" << std::endl;
#endif
	  return -1;
  }	 
  if ((head_top < height_ * head_top_up_percent) || (head_top > height_ * head_top_down_percent)) {
#ifdef SHOWCOUT
	  std::cout << "to high or low!" << std::endl;
	  std::cout << "standard up:" << height_ * head_top_up_percent << ",  standard down:" << height_ * head_top_down_percent << ", value:" << head_top << std::endl;
#endif
	  return -1;
  }
	 

  //int col_max_count = 0;
  //for (int c = 0; c < width_; c++) {
	 // if (column_histogram_[c] > col_max_count)
		//  col_max_count = column_histogram_[c];
  //}
  //cv::Mat col_ave = cv::Mat::zeros(height_, width_, CV_8UC1);
  //for (int c = 0; c < width_; c++) {
	 // col_ave.at<uchar>(static_cast<int>(column_histogram_[c] * (height_ / 2) / col_max_count), c) = 255;
  //}
  //cv::imshow("col_ave", col_ave);
  //cv::moveWindow("col_ave", 600+width_, 60+height_);
  //int max_count = 0, mark_r = 0;
  //for (int r = head_top; r < row_histogram_value.size(); r++) {
	 // //row_histogram_value[r] = row_histogram_value[r] / row_histogram_[r];
	 // if (isnan(row_histogram_value[r])) {
		//  row_histogram_value[r] = 0;
	 // }
	 // if (row_histogram_value[r] > max_count) {
		//  max_count = row_histogram_value[r];
		//  mark_r = r;
	 // }		 
  //}
  //cv::Mat ave = cv::Mat::zeros(height_, width_, CV_8UC1);
  //for (int r = head_top; r < height_; r++)
	 // ave.at<uchar>(r, static_cast<int>(row_histogram_value[r] * (width_/2) / max_count)) = 255;
  //cv::imshow("row_ave_depth", ave);
  //cv::moveWindow("row_ave_depth", 600+width_, 60);

  // Locate Bottom of Head
  float max_variance = 0.0f;
  float weight_torso = 0.0f, weight_head = 0.0f; //头和躯干在横跨了多少row
  int count_head = 0;
  for (int y = head_top; y < height; y++) {
    // Compute Weights
    weight_head++;
    weight_torso = (float)((height - 1) - head_top) - weight_head;
    if ((weight_head > 0) && (weight_torso > 0)) {
      count_head += row_histogram_[y];
      // Compute Means
      float mean_torso, mean_head;
      mean_head = count_head / weight_head;   //横跨的row中，平均每row拥有的像素个数
      mean_torso = (count - count_head) / weight_torso;
      // Compute Between Class Variance
      float between_variance;
      between_variance = (weight_head / ((height - 1) - head_top)) *
                         (weight_torso / ((height - 1) - head_top)) *
                         (mean_head - mean_torso) * (mean_head - mean_torso);
      if (between_variance > max_variance) {
        max_variance = between_variance;
        head_bottom = y;
      }
    }
  }
  head_bottom -= (int)((head_bottom - head_top) * 0.10f);

  //locate left and right of head
  memset(column_histogram_, 0, sizeof(int) * width);
  memset(row_histogram_, 0, sizeof(int) * height);
  std::vector<int> left_width;
  // Generate Histograms
  int count_left_right = 0;  //头部+躯干区域占据的像素总数
  for (int y = head_top; y < head_bottom; y++) {
	  bool flag = false;
	  for (int x = 0; x < width; x++) {
		  int index = y * width + x;
		  if (labels_[index] == foreground_id) {
			  column_histogram_[x]++;
			  row_histogram_[y]++;
			  count_left_right++;
			  if (!flag) {
				  left_width.push_back(x);
				  flag = true;
			  }
		  }
		  
	  }
  }
  //locate left of head
  float max_variance_left = 0.0f;
  float weight_torso_left = 0.0f, weight_head_left = 0.0f; //头和躯干在横跨了多少row
  int count_head_left = 0;
  int left_start = 0;
  int count_left = 0;
  bool flag = false;
  for (int x = 0; x <= head_center; x++) {
	  count_left += column_histogram_[x];
	  if (!flag && column_histogram_[x] > 0) {
		  left_start = x;
		  flag = true;
	  }
  }
  for (int x = left_start; x <= head_center; x++) {
	  // Compute Weights
	  weight_head_left++;
	  weight_torso_left = (float)(head_center - left_start) - weight_head_left;
	  if ((weight_head_left > 0) && (weight_torso_left > 0)) {
		  count_head_left += column_histogram_[x];
		  // Compute Means
		  float mean_torso, mean_head;
		  mean_head = count_head_left / weight_head_left;   //横跨的row中，平均每row拥有的像素个数
		  mean_torso = (count_left - count_head_left) / weight_torso_left;
		  // Compute Between Class Variance
		  float between_variance;
		  between_variance = (weight_head_left / (head_center - left_start)) *
			  (weight_torso_left / (head_center - left_start)) *
			  (mean_head - mean_torso) * (mean_head - mean_torso);
		  if (between_variance > max_variance_left) {
			  max_variance_left = between_variance;
			  head_left = x;
		  }
	  }
  }
  //locate right of head
  float max_variance_right = 0.0f;
  float weight_torso_right = 0.0f, weight_head_right = 0.0f; //头和躯干在横跨了多少row
  int count_head_right = 0;
  int count_right = 0;
  int right_start = width_ - 1;
  bool flag2 = false;
  for (int x = width_ - 1; x > head_center; x--) {
	  count_right += column_histogram_[x];
	  if (!flag && column_histogram_[x] > 0) {
		  right_start = x;
		  flag2 = true;
	  }
  }
  for (int x = right_start; x >= head_center; x--) {
	  // Compute Weights
	  weight_head_right++;
	  weight_torso_right = (float)(right_start - head_center) - weight_head_right;
	  if ((weight_head_right > 0) && (weight_torso_right > 0)) {
		  count_head_right += column_histogram_[x];
		  // Compute Means
		  float mean_torso, mean_head;
		  mean_head = count_head_right / weight_head_right;   //横跨的row中，平均每row拥有的像素个数
		  mean_torso = (count_right - count_head_right) / weight_torso_right;
		  // Compute Between Class Variance
		  float between_variance;
		  between_variance = (weight_head_right / (right_start - head_center)) *
			  (weight_torso_right / (right_start - head_center)) *
			  (mean_head - mean_torso) * (mean_head - mean_torso);
		  if (between_variance > max_variance_right) {
			  max_variance_right = between_variance;
			  head_right = x;
		  }
	  }
  }
  int tempw = head_right - head_left;
  head_left -= tempw * 0.2f;
  head_right += tempw * 0.2f;
  if (head_left < 0)
	  head_left = 0;
  if (head_right > width_ - 1)
	  head_right = width_ - 1;
  // Locate Left and Right side of Head
  int head_left2 = 0;
  int head_right2 = 0;
  for (int y = head_top; y < head_bottom; y++) {
    for (int x = 0; x < width; x++) {
      int i = x + y * width;
      if (labels_[i] == foreground_id) {
        if (x < head_left2)
          head_left2 = x;
        if (x > head_right2)
          head_right2 = x;
      }
    }
  }
  
  head_right = head_right < head_right2 ? head_right : head_right2;

  std::sort(left_width.begin(), left_width.end());
  int head_left3 = left_width[left_width.size() / 2];
  head_left3 -= ((head_right - head_left3) * 0.1f);
  if (head_left3 < 0)
	  head_left3 = 0;
  int mid = head_left;
  if ((head_left2 - head_left) * (head_left2 - head_left3) < 0)
	  mid = head_left2;
  if ((head_left3 - head_left) * (head_left3 - head_left2) < 0)
	  mid = head_left3;
  head_left =  mid;

  // Check head dimensions.
  float head_width = ((head_right - head_left) * foreground_depth) / fx;
  float head_height = ((head_bottom - head_top) * foreground_depth) / fy;

  bool segement_mark = true;
  for (int y = head_top; y < head_bottom; y++) {
	  for (int x = head_left; x < head_right; x++) {
		  int i = x + y * width;

		  if (labels_[i] == foreground_id)
			  segmented_depth[i] = depth[i];
	  }
  }
  if ((head_width > min_width) && (head_width < max_width)) {
    if ((head_height > min_height) && (head_height < max_height)) {
      // Segment User's Head
      for (int y = head_top; y < head_bottom; y++) {
        for (int x = head_left; x < head_right; x++) {
          int i = x + y * width;

          if (labels_[i] == foreground_id)
            segmented_depth[i] = depth[i];
        }
      }	  
	 // return 0;
	}
	else {
		segement_mark = false;
#ifdef SHOWCOUT
		std::cout  << "error head height:" << head_height << std::endl;
#endif
	}
  }
  else {
	  segement_mark = false;
#ifdef SHOWCOUT
	  std::cout << "error head Width:" << head_width <<std::endl;
#endif
  }  
  cv::Mat head_show = cv::Mat::zeros(height_, width_, CV_8UC3);
  for (int r = 0; r < height_; r++) {
	  for (int c = 0; c < width_; c++) {
		  int index = r * width_ + c;
		  if (segement_mark) {
			  head_show.at<cv::Vec3b>(r, c)[0] = (segmented_depth[index] == 0 ? 0 : 255);
			  head_show.at<cv::Vec3b>(r, c)[1] = (segmented_depth[index] == 0 ? 0 : 255);
			  head_show.at<cv::Vec3b>(r, c)[2] = (segmented_depth[index] == 0 ? 0 : 255);
		  }
		  else {
			  head_show.at<cv::Vec3b>(r, c)[2] = (segmented_depth[index] == 0 ? 0 : 255);
		  }

	  }
  }
  //left, right, center lines
  for (int r = 0; r < height_; r++) {
	  head_show.at<cv::Vec3b>(r, head_left) = cv::Vec3b(255, 255, 255);
	  head_show.at<cv::Vec3b>(r, head_right) = cv::Vec3b(255, 255, 255);
	  head_show.at<cv::Vec3b>(r, head_center) = cv::Vec3b(255, 255, 255);
  }
  //top, botton lines
  for (int c = 0; c < width_; c++) {
	  head_show.at<cv::Vec3b>(head_top, c) = cv::Vec3b(255, 255, 255);
	 // head_show.at<cv::Vec3b>(height_ * head_top_down_percent, c)  = cv::Vec3b(0, 0, 255);	  
	  head_show.at<cv::Vec3b>(head_bottom, c) = cv::Vec3b(255, 255, 255);
  }
  cv::imshow("head", head_show);
  cv::moveWindow("head", 600, 40 + height_);
  cv::waitKey(1);
  //int res = cv::waitKey(1);
  //if (res == 115) {
	 // std::stringstream ss;
	 // ss << count_ << std::endl;
	 // std::string out;
	 // ss >> out;
	 // cv::Mat rawDepth = cv::Mat::zeros(height_, width_, CV_8UC1);
	 // for (int y = 0; y < height_; y++)
	 // {
		//  for (int x = 0; x < width_; x++) {
		//	  int index = y * width_ + x;
		//	  rawDepth.at<uchar>(y, x) = depth[index] * 255 / 4096;
		//  }
	 // }
	 // std::vector<int> compression_params;
	 // compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	 // compression_params.push_back(1);    // 无压缩png.
	 // cv::imwrite("./depth" + out +".png", rawDepth, compression_params);
	 // cv::Mat head = cv::Mat::zeros(height_, width_, CV_8UC1);
	 // for (int y = 0; y < height_; y++)
	 // {
		//  for (int x = 0; x < width_; x++) {
		//	  int index = y * width_ + x;
		//	  head.at<uchar>(y, x) = segmented_depth[index] * 255 / 4096;
		//  }
	 // }
	 // cv::imwrite("./head"+ out + ".png", head, compression_params);
	 // count_++;
	 // std::cout << "---------------------------------------------------" << std::endl;
  //}
  //else if (res == 27)
	 // exit(0);

  if (segement_mark)
	  return 0;
  return -1;
}

} // namespace dip
