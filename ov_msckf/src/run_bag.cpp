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

#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "core/VioManager.h"
#include "ros/ROS2Visualizer.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

namespace {

template <typename Message> struct BagStream {
  rosbag2_cpp::Reader reader;
  Message next;
  bool has_next = false;
  int64_t last_timestamp = std::numeric_limits<int64_t>::min();
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
  }
};

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("run_bag", options);

    std::string config_path;
    std::string bag_path;
    if (!node->get_parameter("config_path", config_path) || !node->get_parameter("bag_path", bag_path) || bag_path.empty())
      throw std::runtime_error("config_path and bag_path are required");

    auto parser = std::make_shared<ov_core::YamlParser>(config_path);
    parser->set_node(node);
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);

    VioManagerOptions params;
    params.print_and_load(parser);
    if (params.filter_type != VioManagerOptions::FilterType::OPENVINS)
      throw std::runtime_error("run_bag supports only filter_type=openvins");
    params.set_results_namespace(node->get_namespace());
    params.num_opencv_threads = 0;
    params.use_multi_threading_pubs = false;
    params.use_multi_threading_subs = false;

    std::string imu_topic = "/imu0";
    parser->parse_external("relative_config_imu", "imu0", "rostopic", imu_topic);
    node->get_parameter("topic_imu", imu_topic);

    std::vector<std::string> camera_topics;
    for (int i = 0; i < params.state_options.num_cameras; ++i) {
      std::string topic = "/cam" + std::to_string(i) + "/image_raw";
      parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", topic);
      camera_topics.push_back(topic);
    }
    std::vector<std::string> camera_topics_override;
    if (node->get_parameter("camera_topics", camera_topics_override) && !camera_topics_override.empty()) {
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
    if (!parser->successful())
      throw std::runtime_error("Unable to parse all estimator parameters");

    BagStream<sensor_msgs::msg::Imu> imu;
    imu.open(bag_path, imu_topic, "sensor_msgs/msg/Imu");
    std::vector<std::unique_ptr<BagStream<sensor_msgs::msg::Image>>> cameras;
    for (const auto &topic : camera_topics) {
      auto camera = std::make_unique<BagStream<sensor_msgs::msg::Image>>();
      camera->open(bag_path, topic, "sensor_msgs/msg/Image");
      cameras.push_back(std::move(camera));
    }
    if (!imu.has_next)
      throw std::runtime_error("No IMU messages on " + imu_topic);
    for (size_t i = 0; i < cameras.size(); ++i) {
      if (!cameras[i]->has_next)
        throw std::runtime_error("No camera messages on " + camera_topics[i]);
    }

    auto sys = std::make_shared<VioManager>(params);
    bool visualize = false;
    node->get_parameter("visualize", visualize);
    std::shared_ptr<ROS2Visualizer> viz;
    if (visualize)
      viz = std::make_shared<ROS2Visualizer>(node, sys);

    double last_imu_time = -std::numeric_limits<double>::infinity();
    size_t imu_count = 0;
    size_t image_count = 0;
    size_t update_count = 0;
    size_t images_read = 0;
    size_t stereo_images_matched = 0;
    using Image = sensor_msgs::msg::Image;
    using StereoPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
    std::deque<std::pair<Image::ConstSharedPtr, Image::ConstSharedPtr>> stereo_pairs;
    std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> stereo_sync;
    if (cameras.size() == 2 && params.use_stereo) {
      stereo_sync = std::make_unique<message_filters::Synchronizer<StereoPolicy>>(StereoPolicy(10));
      auto callback = [&](Image::ConstSharedPtr left, Image::ConstSharedPtr right) {
        stereo_pairs.emplace_back(left, right);
        stereo_images_matched += 2;
      };
      stereo_sync->registerCallback(std::bind(callback, std::placeholders::_1, std::placeholders::_2));
    }
    auto feed_imu = [&] {
      ov_core::ImuData data;
      data.timestamp = rclcpp::Time(imu.next.header.stamp).seconds();
      data.wm << imu.next.angular_velocity.x, imu.next.angular_velocity.y, imu.next.angular_velocity.z;
      data.am << imu.next.linear_acceleration.x, imu.next.linear_acceleration.y, imu.next.linear_acceleration.z;
      sys->feed_measurement_imu(data);
      last_imu_time = data.timestamp;
      ++imu_count;
      if (viz)
        viz->visualize_odometry(data.timestamp);
      imu.advance();
    };
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
      if (frames.empty())
        break;

      const double camera_time = rclcpp::Time(frames.front().second->header.stamp).seconds();
      const double required_imu_time = camera_time + sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
      while (imu.has_next && last_imu_time <= required_imu_time)
        feed_imu();
      if (last_imu_time <= required_imu_time) {
        PRINT_WARNING("[BAG]: No IMU lookahead for camera time %.9f; trailing images cannot be updated\n", camera_time);
        break;
      }

      ov_core::CameraData data;
      data.timestamp = camera_time;
      for (const auto &frame : frames) {
        auto image = cv_bridge::toCvCopy(frame.second, sensor_msgs::image_encodings::MONO8);
        data.sensor_ids.push_back(frame.first);
        data.images.push_back(image->image);
        if (params.use_mask)
          data.masks.push_back(params.masks.at(frame.first));
        else
          data.masks.push_back(cv::Mat::zeros(image->image.rows, image->image.cols, CV_8UC1));
        ++image_count;
      }
      sys->feed_measurement_camera(data);
      ++update_count;
      if (viz)
        viz->visualize();
    }

    while (rclcpp::ok() && imu.has_next)
      feed_imu();
    PRINT_INFO("[BAG]: Fed %zu IMU samples and %zu images in %zu camera updates\n", imu_count, image_count, update_count);
    if (stereo_sync && images_read > stereo_images_matched)
      PRINT_WARNING("[BAG]: %zu stereo images were not matched by OpenVINS synchronization\n", images_read - stereo_images_matched);
    if (viz)
      viz->visualize_final();
    viz.reset();
    sys.reset();
    parser.reset();
    node.reset();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("run_bag"), "%s", error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
}
