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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

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
    params.set_results_namespace(node->get_namespace());
    // Bag measurements are dispatched directly, without subscription workers.
    params.use_multi_threading_subs = false;
    if (!std::isfinite(params.track_frequency) || params.track_frequency <= 0.0)
      throw std::runtime_error("track_frequency must be finite and positive");
    double bag_start = 0.0;
    double bag_duration = -1.0;
    node->get_parameter("bag_start", bag_start);
    node->get_parameter("bag_duration", bag_duration);
    if (!std::isfinite(bag_start) || bag_start < 0.0 || !std::isfinite(bag_duration) || (bag_duration != -1.0 && bag_duration <= 0.0))
      throw std::runtime_error("bag_start must be finite and nonnegative; bag_duration must be positive or -1 for the full bag");

    std::string imu_topic = "/imu0";
    parser->parse_external("relative_config_imu", "imu0", "rostopic", imu_topic);
    node->get_parameter("topic_imu", imu_topic);

    std::vector<std::string> camera_topics;
    for (int i = 0; i < params.state_options.num_cameras; ++i) {
      std::string topic = "/cam" + std::to_string(i) + "/image_raw";
      parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", topic);
      node->get_parameter("topic_camera" + std::to_string(i), topic);
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
    if (!imu.has_next)
      throw std::runtime_error("No IMU messages on " + imu_topic);
    const double start_time = rclcpp::Time(imu.next.header.stamp).seconds() + bag_start;
    const double end_time = bag_duration < 0.0 ? std::numeric_limits<double>::infinity() : start_time + bag_duration;
    // Preserve leading camera/GPS messages when replay starts at the beginning of the bag.
    const double minimum_time = bag_start > 0.0 ? start_time : -std::numeric_limits<double>::infinity();
    imu.restrict_interval(minimum_time, end_time);

    const bool use_gps = params.max_gps_init_time > 0.0;
    std::unique_ptr<BagStream<sensor_msgs::msg::NavSatFix>> gps_fix;
    std::unique_ptr<BagStream<geometry_msgs::msg::TwistWithCovarianceStamped>> gps_velocity;
    if (use_gps) {
      std::string fix_topic = "/gps/fix";
      std::string velocity_topic = "/gps/velocity";
      node->get_parameter("topic_gps_fix", fix_topic);
      node->get_parameter("topic_gps_velocity", velocity_topic);
      gps_fix = std::make_unique<BagStream<sensor_msgs::msg::NavSatFix>>();
      gps_velocity = std::make_unique<BagStream<geometry_msgs::msg::TwistWithCovarianceStamped>>();
      gps_fix->open(bag_path, fix_topic, "sensor_msgs/msg/NavSatFix");
      gps_velocity->open(bag_path, velocity_topic, "geometry_msgs/msg/TwistWithCovarianceStamped");
      gps_fix->restrict_interval(minimum_time, end_time);
      gps_velocity->restrict_interval(minimum_time, end_time);
      if (!gps_fix->has_next || !gps_velocity->has_next)
        throw std::runtime_error("GPS initialization requires nonempty fix and velocity topics");
    }

    std::vector<std::unique_ptr<BagStream<sensor_msgs::msg::Image>>> cameras;
    for (const auto &topic : camera_topics) {
      auto camera = std::make_unique<BagStream<sensor_msgs::msg::Image>>();
      camera->open(bag_path, topic, "sensor_msgs/msg/Image");
      camera->restrict_interval(minimum_time, end_time);
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
    bool save_total_state = false;
    node->get_parameter("save_total_state", save_total_state);
    std::string path_gt;
    node->get_parameter("path_gt", path_gt);
    std::shared_ptr<ROS2Visualizer> viz;
    if (visualize || save_total_state || !path_gt.empty())
      viz = std::make_shared<ROS2Visualizer>(node, sys);

    const double first_imu_time = rclcpp::Time(imu.next.header.stamp).seconds();
    double last_imu_time = -std::numeric_limits<double>::infinity();
    size_t imu_count = 0;
    size_t image_count = 0;
    size_t update_count = 0;
    size_t gps_pairs = 0;
    size_t gps_applied = 0;
    size_t gps_unpaired = 0;
    size_t gps_invalid = 0;
    double first_gps_applied = 0.0;
    double last_gps_applied = 0.0;
    size_t images_read = 0;
    size_t images_throttled = 0;
    size_t stereo_images_matched = 0;
    std::vector<double> last_camera_times(cameras.size(), -std::numeric_limits<double>::infinity());
    const double camera_interval = 1.0 / params.track_frequency;
    using Image = sensor_msgs::msg::Image;
    using StereoPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
    std::deque<std::pair<Image::ConstSharedPtr, Image::ConstSharedPtr>> stereo_pairs;
    std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> stereo_sync;
    // The subscription runner synchronizes two cameras even when tracking them independently.
    if (cameras.size() == 2) {
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
    const auto processing_start = std::chrono::steady_clock::now();
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
      // GPS headers share the IMU clock. Apply matched samples before the next camera update.
      while (gps_fix && gps_fix->has_next && gps_velocity->has_next) {
        const int64_t fix_stamp = rclcpp::Time(gps_fix->next.header.stamp).nanoseconds();
        const int64_t velocity_stamp = rclcpp::Time(gps_velocity->next.header.stamp).nanoseconds();
        if (std::min(fix_stamp, velocity_stamp) * 1e-9 > required_imu_time)
          break;
        if (fix_stamp != velocity_stamp) {
          ++gps_unpaired;
          if (fix_stamp < velocity_stamp)
            gps_fix->advance();
          else
            gps_velocity->advance();
          continue;
        }
        const double gps_time = rclcpp::Time(gps_fix->next.header.stamp).seconds();
        if (sys->initialized_time() >= 0.0 &&
            gps_time - sys->get_state()->_calib_dt_CAMtoIMU->value()(0) >= sys->initialized_time() + params.max_gps_init_time) {
          gps_fix.reset();
          gps_velocity.reset();
          break;
        }
        ++gps_pairs;
        const auto &fix = gps_fix->next;
        const auto &velocity = gps_velocity->next;
        if (velocity.header.frame_id != "enu")
          throw std::runtime_error("GPS velocity must be in the local ENU frame (header.frame_id = enu)");
        if (fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
            fix.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
          ++gps_invalid;
        } else {
          while (imu.has_next && last_imu_time <= gps_time)
            feed_imu();
          if (last_imu_time > gps_time) {
            ov_core::GPSData gps;
            gps.timestamp = gps_time;
            gps.lla << fix.latitude, fix.longitude, fix.altitude;
            gps.velocity << velocity.twist.twist.linear.x, velocity.twist.twist.linear.y, velocity.twist.twist.linear.z;
            gps.cov_position = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(fix.position_covariance.data());
            gps.cov_velocity =
                Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(velocity.twist.covariance.data()).topLeftCorner<3, 3>();
            if (sys->feed_measurement_gps(gps)) {
              if (gps_applied == 0)
                first_gps_applied = gps_time;
              last_gps_applied = gps_time;
              ++gps_applied;
            }
          }
        }
        gps_fix->advance();
        gps_velocity->advance();
      }
      while (imu.has_next && last_imu_time <= required_imu_time)
        feed_imu();
      if (last_imu_time <= required_imu_time) {
        PRINT_WARNING("[BAG]: No IMU lookahead for camera time %.9f; trailing images cannot be updated\n", camera_time);
        break;
      }

      // Match the ROS callbacks: throttle each monocular stream, or the synchronized pair using camera 0's timestamp.
      // Continue dispatching IMU and GPS above even when this image is dropped.
      double &last_camera_time = last_camera_times.at(frames.front().first);
      if (camera_time < last_camera_time + camera_interval) {
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
    const double processing_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - processing_start).count();
    PRINT_INFO("[BAG]: Fed %zu IMU samples and %zu images in %zu camera updates\n", imu_count, image_count, update_count);
    PRINT_INFO("[BAG]: Dropped %zu images to enforce track_frequency %.3f Hz\n", images_throttled, params.track_frequency);
    PRINT_INFO("[BAG]: Simulated time %.3f seconds | processing time %.3f seconds\n", last_imu_time - first_imu_time, processing_time);
    if (use_gps) {
      PRINT_INFO("[GPS]: Applied %zu of %zu paired samples; %zu invalid fixes and %zu unpaired messages; first/last %.9f %.9f\n",
                 gps_applied, gps_pairs, gps_invalid, gps_unpaired, first_gps_applied, last_gps_applied);
      PRINT_INFO("[GPS]: VIO initialization %.9f, cutoff %.9f (camera clock)\n", sys->initialized_time(),
                 sys->initialized_time() + params.max_gps_init_time);
      if (gps_applied == 0)
        PRINT_WARNING("[GPS]: No usable GPS measurements were applied during initialization\n");
    }
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
