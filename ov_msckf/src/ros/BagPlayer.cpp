/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "BagPlayer.h"

#include <algorithm>
#include <cmath>
#include <functional>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "core/VioManager.h"
#include "ros/ROS2Visualizer.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

BagPlayer::BagPlayer(const std::string &bag_path, const VioManagerOptions &params, ov_core::YamlParser &parser, rclcpp::Node &node,
                     double bag_start, double bag_duration, bool truth_initialized)
    : use_mask(params.use_mask), masks(params.masks), track_frequency(params.track_frequency) {
  if (!std::isfinite(track_frequency) || track_frequency <= 0.0)
    throw std::runtime_error("track_frequency must be finite and positive");
  if (!std::isfinite(bag_start) || bag_start < 0.0 || !std::isfinite(bag_duration) || (bag_duration != -1.0 && bag_duration <= 0.0))
    throw std::runtime_error("bag_start must be finite and nonnegative; bag_duration must be positive or -1 for the full bag");
  if (!std::isfinite(params.calib_camimu_dt))
    throw std::runtime_error("Camera-to-IMU offset must be finite");
  std::string imu_topic = "/imu0";
  parser.parse_external("relative_config_imu", "imu0", "rostopic", imu_topic);
  node.get_parameter("topic_imu", imu_topic);

  std::vector<std::string> camera_topics;
  for (int i = 0; i < params.state_options.num_cameras; ++i) {
    std::string topic = "/cam" + std::to_string(i) + "/image_raw";
    parser.parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", topic);
    node.get_parameter("topic_camera" + std::to_string(i), topic);
    camera_topics.push_back(topic);
  }
  std::vector<std::string> camera_topics_override;
  if (node.get_parameter("camera_topics", camera_topics_override) && !camera_topics_override.empty()) {
    if (camera_topics_override.size() != camera_topics.size())
      throw std::runtime_error("camera_topics must contain one topic per configured camera");
    camera_topics = camera_topics_override;
  }
  if (std::find(camera_topics.begin(), camera_topics.end(), imu_topic) != camera_topics.end())
    throw std::runtime_error("IMU and camera topics must be different");
  for (size_t i = 0; i < camera_topics.size(); ++i) {
    if (std::find(camera_topics.begin(), camera_topics.begin() + i, camera_topics[i]) != camera_topics.begin() + i)
      throw std::runtime_error("Camera topics must be unique");
  }
  if (!parser.successful())
    throw std::runtime_error("Unable to parse all estimator parameters");

  imu.open(bag_path, imu_topic, "sensor_msgs/msg/Imu");
  if (!imu.has_next)
    throw std::runtime_error("No IMU messages on " + imu_topic);
  replay_start = rclcpp::Time(imu.next.header.stamp).seconds() + bag_start;
  const double end_time = bag_duration < 0.0 ? std::numeric_limits<double>::infinity() : replay_start + bag_duration;
  // Preserve leading camera messages when replay starts at the beginning of the bag.
  const double minimum_time = bag_start > 0.0 ? replay_start : -std::numeric_limits<double>::infinity();
  if (truth_initialized) {
    // Retain the last sample before startup so propagation can interpolate at startup.
    while (imu.has_next && rclcpp::Time(imu.next.header.stamp).seconds() < replay_start) {
      leading_imu = std::move(imu.next);
      have_leading_imu = true;
      imu.advance();
    }
  }
  imu.restrict_interval(minimum_time, end_time);

  for (const auto &topic : camera_topics) {
    auto camera = std::make_unique<BagStream<sensor_msgs::msg::Image>>();
    camera->open(bag_path, topic, "sensor_msgs/msg/Image");
    const double camera_start = truth_initialized ? replay_start - params.calib_camimu_dt : minimum_time;
    camera->restrict_interval(camera_start, end_time);
    cameras.push_back(std::move(camera));
  }
  if (!imu.has_next)
    throw std::runtime_error("No IMU messages on " + imu_topic);
  for (size_t i = 0; i < cameras.size(); ++i) {
    if (!cameras[i]->has_next)
      throw std::runtime_error("No camera messages on " + camera_topics[i]);
  }

  first_imu_time = truth_initialized ? replay_start : rclcpp::Time(imu.next.header.stamp).seconds();
  last_camera_times.assign(cameras.size(), -std::numeric_limits<double>::infinity());
  // The subscription runner synchronizes two cameras even when tracking them independently.
  if (cameras.size() == 2) {
    stereo_sync = std::make_unique<message_filters::Synchronizer<StereoPolicy>>(StereoPolicy(10));
    auto callback = [this](Image::ConstSharedPtr left, Image::ConstSharedPtr right) {
      stereo_pairs.emplace_back(left, right);
      stereo_images_matched += 2;
    };
    stereo_sync->registerCallback(std::bind(callback, std::placeholders::_1, std::placeholders::_2));
  }
}

void BagPlayer::feed_imu_message(const sensor_msgs::msg::Imu &message, VioManager &sys, ROS2Visualizer *viz) {
  ov_core::ImuData data;
  data.timestamp = rclcpp::Time(message.header.stamp).seconds();
  data.wm << message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z;
  data.am << message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z;
  sys.feed_measurement_imu(data);
  last_imu_time = data.timestamp;
  ++imu_count;
  if (viz)
    viz->visualize_odometry(data.timestamp);
}

void BagPlayer::feed_imu(VioManager &sys, ROS2Visualizer *viz) {
  feed_imu_message(imu.next, sys, viz);
  imu.advance();
}

bool BagPlayer::process_next_camera(VioManager &sys, ROS2Visualizer *viz) {
  if (camera_finished || finished)
    return false;
  if (processing_start == std::chrono::steady_clock::time_point()) {
    if (have_leading_imu)
      feed_imu_message(leading_imu, sys, viz);
    processing_start = std::chrono::steady_clock::now();
  }
  while (rclcpp::ok()) {
    std::vector<std::pair<int, Image::ConstSharedPtr>> frames;
    do {
      int first = -1;
      for (size_t i = 0; i < cameras.size(); ++i) {
        if (cameras[i]->has_next &&
            (first < 0 || rclcpp::Time(cameras[i]->next.header.stamp) < rclcpp::Time(cameras[first]->next.header.stamp)))
          first = static_cast<int>(i);
      }
      if (first < 0)
        break;
      auto image = std::make_shared<Image>(std::move(cameras[first]->next));
      cameras[first]->advance();
      ++images_read;
      if (stereo_sync) {
        if (first == 0)
          stereo_sync->add<0>(image);
        else
          stereo_sync->add<1>(image);
      } else {
        frames.emplace_back(first, image);
      }
    } while (stereo_sync && stereo_pairs.empty());
    if (stereo_sync && !stereo_pairs.empty()) {
      frames.emplace_back(0, stereo_pairs.front().first);
      frames.emplace_back(1, stereo_pairs.front().second);
      stereo_pairs.pop_front();
    }
    if (frames.empty()) {
      camera_finished = true;
      return false;
    }

    const double camera_time = rclcpp::Time(frames.front().second->header.stamp).seconds();
    const double required_imu_time = camera_time + sys.get_state()->_calib_dt_CAMtoIMU->value()(0);
    while (imu.has_next && last_imu_time <= required_imu_time)
      feed_imu(sys, viz);
    if (last_imu_time <= required_imu_time) {
      PRINT_WARNING("[BAG]: No IMU lookahead for camera time %.9f; trailing images cannot be updated\n", camera_time);
      camera_finished = true;
      return false;
    }

    // Match the ROS callbacks: throttle each monocular stream, or the synchronized pair using camera 0's timestamp.
    // Continue dispatching IMU above even when this image is dropped.
    double &last_camera_time = last_camera_times.at(frames.front().first);
    if (camera_time < last_camera_time + 1.0 / track_frequency) {
      images_throttled += frames.size();
      continue;
    }
    last_camera_time = camera_time;

    ov_core::CameraData data;
    data.timestamp = camera_time;
    for (const auto &frame : frames) {
      auto image = cv_bridge::toCvCopy(frame.second, sensor_msgs::image_encodings::MONO8);
      data.sensor_ids.push_back(frame.first);
      data.images.push_back(image->image);
      if (use_mask)
        data.masks.push_back(masks.at(frame.first));
      else
        data.masks.push_back(cv::Mat::zeros(image->image.rows, image->image.cols, CV_8UC1));
      ++image_count;
    }
    sys.feed_measurement_camera(data);
    ++update_count;
    last_processed_camera = camera_time;
    return true;
  }
  return false;
}

void BagPlayer::finish(VioManager &sys, ROS2Visualizer *viz) {
  if (finished)
    return;
  finished = true;
  while (rclcpp::ok() && imu.has_next)
    feed_imu(sys, viz);
  const double processing_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - processing_start).count();
  PRINT_INFO("[BAG]: Fed %zu IMU samples and %zu images in %zu camera updates\n", imu_count, image_count, update_count);
  PRINT_INFO("[BAG]: Dropped %zu images to enforce track_frequency %.3f Hz\n", images_throttled, track_frequency);
  PRINT_INFO("[BAG]: Simulated time %.3f seconds | processing time %.3f seconds\n", last_imu_time - first_imu_time, processing_time);
  if (stereo_sync && images_read > stereo_images_matched)
    PRINT_WARNING("[BAG]: %zu stereo images were not matched by OpenVINS synchronization\n", images_read - stereo_images_matched);
}
