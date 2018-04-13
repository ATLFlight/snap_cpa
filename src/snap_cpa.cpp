/****************************************************************************
 *   Copyright (c) 2018 Michael Shomin. All rights reserved.
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
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
 ****************************************************************************/
#include "snap_cpa/snap_cpa.hpp"

SnapCpa::SnapCpa(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh),
    pnh_(pnh),
    im_trans_(nh_) {
  running_ = false;
  cpa_ptr_ = nullptr;
  previous_cpa_timestamp_ = ros::Time(0);
  cpa_config_.cpaType = MVCPA_MODE::MVCPA_MODE_COST;

  pnh_.param("camera_driver_name", camera_driver_name_, std::string("snap_cam_nodelet"));

  double gain_cost, exp_cost;
  int filter_size;
  pnh_.param("gain_cost", gain_cost, 0.25);
  pnh_.param("exposure_cost", exp_cost, 0.75);
  pnh_.param("filter_size", filter_size, 2);

  cpa_config_.legacyCost.gainCost = gain_cost;
  cpa_config_.legacyCost.exposureCost = exp_cost;
  cpa_config_.legacyCost.filterSize = filter_size;

  pnh_.param("min_period", min_period_, 0.3);

  int width, height;
  ros::param::get(camera_driver_name_+"/width", width);
  ros::param::get(camera_driver_name_+"/height", height);
  cpa_config_.width = width;
  cpa_config_.height = height;

  ROS_INFO_STREAM("[CPA]: using camera params width:" << width << "height: " << height);
  image_subscriber_ = im_trans_.subscribeCamera("image_raw", 10, &SnapCpa::ImageCallback, this);
}


SnapCpa::~SnapCpa() {
  if (cpa_ptr_ != nullptr) {
    mvCPA_Deinitialize( cpa_ptr_ );
    cpa_ptr_ = nullptr;
  }
}


bool SnapCpa::Start() {
  std::cout << "Start" << std::endl;

  // Initialize mvCPA
  cpa_ptr_ = mvCPA_Initialize(&cpa_config_);
  if (cpa_ptr_ == nullptr) {
    ROS_ERROR_STREAM("Could not initialize mvCPA object.");
    return false;
  }

  running_=true;
  return true;
}


void SnapCpa::Stop() {
  running_=false;
}

void SnapCpa::SetCameraParameters(float gain_f, float exp_f) {

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::Config conf;
    dynamic_reconfigure::DoubleParameter gain, exp;

    gain.name = "gain";
    exp.name = "exposure";
    gain.value = gain_f;
    exp.value = exp_f;

    conf.doubles.push_back(gain);
    conf.doubles.push_back(exp);
    srv_req.config = conf;

    ros::service::call(camera_driver_name_+"/set_parameters", srv_req, srv_resp);
}


void SnapCpa::ImageCallback(const sensor_msgs::ImageConstPtr& msg,
                            const sensor_msgs::CameraInfoConstPtr& cinfo) {
  if (msg->header.stamp - previous_cpa_timestamp_ > ros::Duration(min_period_)) {
    mvCPA_AddFrame(cpa_ptr_, &msg->data[0], msg->step);

    float gain_f, exp_f;
    mvCPA_GetValues(cpa_ptr_, &exp_f, &gain_f);
    SetCameraParameters(gain_f, exp_f);
    previous_cpa_timestamp_ = msg->header.stamp;
  }
}
