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

#ifndef OV_MSCKF_BAG_PLAYER_H
#define OV_MSCKF_BAG_PLAYER_H

#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace ov_core {
class YamlParser;
}

namespace ov_msckf {

class VioManager;
struct VioManagerOptions;
class ROS2Visualizer;

template <typename Message> struct BagStream {
  rosbag2_cpp::Reader reader;
  Message next;
  bool has_next = false;
  int64_t last_timestamp = std::numeric_limits<int64_t>::min();
  double end_time = std::numeric_limits<double>::infinity();
  std::string topic;

  void open(const std::string &bag_path, const std::string &topic_name, const std::string &expected_type) {
    topic = topic_name;
    reader.open(bag_path);
    bool found = false;
    for (const auto &metadata : reader.get_all_topics_and_types()) {
      if (metadata.name != topic)
        continue;
      found = true;
      if (metadata.type != expected_type)
        throw std::runtime_error("Expected " + expected_type + " on " + topic + ", found " + metadata.type);
    }
    if (!found)
      throw std::runtime_error("Topic is absent from bag: " + topic);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {topic};
    reader.set_filter(filter);
    advance();
  }

  void advance() {
    has_next = reader.has_next();
    if (!has_next)
      return;
    next = reader.read_next<Message>();
    const int64_t timestamp = rclcpp::Time(next.header.stamp).nanoseconds();
    if (timestamp < last_timestamp)
      throw std::runtime_error("Header timestamps go backwards on " + topic);
    last_timestamp = timestamp;
    if (rclcpp::Time(next.header.stamp).seconds() >= end_time)
      has_next = false;
  }

  void restrict_interval(double start, double end) {
    end_time = end;
    while (has_next && rclcpp::Time(next.header.stamp).seconds() < start)
      advance();
    if (has_next && rclcpp::Time(next.header.stamp).seconds() >= end_time)
      has_next = false;
  }
};

/// Offline playback shared by the single-agent and multi-agent ROS 2 runners.
class BagPlayer {
public:
  BagPlayer(const std::string &bag_path, const VioManagerOptions &params, ov_core::YamlParser &parser, rclcpp::Node &node, double bag_start,
            double bag_duration, bool truth_initialized);

  double start_time() const { return replay_start; }
  double camera_time() const { return last_processed_camera; }

  /// Feed IMU lookahead and one unthrottled camera update; false ends camera playback.
  bool process_next_camera(VioManager &sys, ROS2Visualizer *viz);

  /// Drain the remaining IMU measurements and print the playback summary.
  void finish(VioManager &sys, ROS2Visualizer *viz);

private:
  void feed_imu_message(const sensor_msgs::msg::Imu &message, VioManager &sys, ROS2Visualizer *viz);
  void feed_imu(VioManager &sys, ROS2Visualizer *viz);

  using Image = sensor_msgs::msg::Image;
  using StereoPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  BagStream<sensor_msgs::msg::Imu> imu;
  std::vector<std::unique_ptr<BagStream<Image>>> cameras;
  std::deque<std::pair<Image::ConstSharedPtr, Image::ConstSharedPtr>> stereo_pairs;
  std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> stereo_sync;
  sensor_msgs::msg::Imu leading_imu;
  bool have_leading_imu = false;
  bool camera_finished = false;
  bool finished = false;
  const bool use_mask;
  const std::map<size_t, cv::Mat> masks;
  const double track_frequency;
  double replay_start;
  double first_imu_time;
  double last_imu_time = -std::numeric_limits<double>::infinity();
  double last_processed_camera = -std::numeric_limits<double>::infinity();
  std::vector<double> last_camera_times;
  size_t imu_count = 0;
  size_t image_count = 0;
  size_t update_count = 0;
  size_t images_read = 0;
  size_t images_throttled = 0;
  size_t stereo_images_matched = 0;
  std::chrono::steady_clock::time_point processing_start;
};

} // namespace ov_msckf

#endif // OV_MSCKF_BAG_PLAYER_H
