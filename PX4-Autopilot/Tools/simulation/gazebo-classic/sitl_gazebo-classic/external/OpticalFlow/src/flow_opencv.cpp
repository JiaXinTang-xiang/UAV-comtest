/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
*  flow_opencv.cpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#include "flow_opencv.hpp"

/****************************************************************************
 * OpenCV optical flow calculation
 ****************************************************************************/

OpticalFlowOpenCV::OpticalFlowOpenCV(float f_length_x, float f_length_y, int ouput_rate, int img_width, int img_height,
				     int num_feat,
				     float conf_multi) :
	num_features(num_feat),
	confidence_multiplier(conf_multi)
{
	setImageWidth(img_width);
	setImageHeight(img_height);
	setFocalLengthX(f_length_x);
	setFocalLengthY(f_length_y);
	setOutputRate(ouput_rate);

	set_camera_matrix = false;
	set_camera_distortion = false;
	camera_matrix.create(3, 3);
	camera_distortion.create(1, 5);

	initLimitRate();
}

OpticalFlowOpenCV::~OpticalFlowOpenCV(void)
{

}

void OpticalFlowOpenCV::setCameraMatrix(float focal_len_x, float focal_len_y,
					float principal_point_x, float principal_point_y)
{
	camera_matrix <<   focal_len_x, 0.0f, principal_point_x,
		      0.0f, focal_len_y, principal_point_y,
		      0.0f, 0.0f, 1.0f;

	set_camera_matrix = true;
}

void OpticalFlowOpenCV::setCameraDistortion(float k1, float k2, float k3, float p1, float p2)
{
	camera_distortion <<   k1, k2, p1, p2, k3;

	set_camera_distortion = true;
}

// Track the same features between consecutive rendered frames. Redetect only
// after displacement is measured, so newly assigned feature indices cannot
// be compared against unrelated points from the previous frame.
int OpticalFlowOpenCV::calcFlow(uint8_t *img_current, const uint32_t &img_time_us,
                               int &dt_us, float &flow_x, float &flow_y)
{
    cv::Mat frame(image_height, image_width, CV_8UC1, img_current);
    float dx = 0.0f, dy = 0.0f;
    int quality = 0;
    if (!previous_frame.empty() && !tracked_points.empty()) {
        std::vector<cv::Point2f> current, back;
        std::vector<uchar> status, back_status;
        std::vector<float> error, back_error;
        cv::calcOpticalFlowPyrLK(previous_frame, frame, tracked_points, current,
                                status, error, cv::Size(15, 15), 2);
        cv::calcOpticalFlowPyrLK(frame, previous_frame, current, back,
                                back_status, back_error, cv::Size(15, 15), 2);
        std::vector<float> xs, ys;
        for (size_t i = 0; i < tracked_points.size(); ++i) {
            if (status[i] && back_status[i] && error[i] < 30.0f &&
                cv::norm(back[i] - tracked_points[i]) < 1.0f &&
                current[i].x >= 0 && current[i].x < image_width &&
                current[i].y >= 0 && current[i].y < image_height) {
                xs.push_back(current[i].x - tracked_points[i].x);
                ys.push_back(current[i].y - tracked_points[i].y);
            }
        }
        if (xs.size() >= 4) {
            std::sort(xs.begin(), xs.end());
            std::sort(ys.begin(), ys.end());
            dx = xs[xs.size() / 2];
            dy = ys[ys.size() / 2];
            quality = static_cast<int>(255 * xs.size() / tracked_points.size());
        }
    }
    cv::goodFeaturesToTrack(frame, tracked_points, 40, 0.01, 4.0);
    frame.copyTo(previous_frame);
    quality = limitRate(quality, img_time_us, &dt_us, &dx, &dy);
    flow_x = atan2(dx, focal_length_x);
    flow_y = atan2(dy, focal_length_y);
    return quality;
}
