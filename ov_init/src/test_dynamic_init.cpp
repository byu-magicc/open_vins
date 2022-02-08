/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
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

#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#if ROS_AVAILABLE == 1
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#endif

#include "dynamic/DynamicInitializer.h"
#include "init/InertialInitializerOptions.h"
#include "sim/Simulator.h"

#include "track/TrackSIM.h"
#include "utils/colors.h"
#include "utils/sensor_data.h"

using namespace ov_init;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) { std::exit(signum); }

#if ROS_AVAILABLE == 1
// taken from ov_eval/src/alignment/AlignUtils.h
static inline double get_best_yaw(const Eigen::Matrix<double, 3, 3> &C) {
  double A = C(0, 1) - C(1, 0);
  double B = C(0, 0) + C(1, 1);
  // return M_PI_2 - atan2(B, A);
  return atan2(A, B);
}
// taken from ov_eval/src/alignment/AlignUtils.h
void align_posyaw_single(const Eigen::Vector4d &q_es_0, const Eigen::Vector3d &p_es_0, const Eigen::Vector4d &q_gt_0,
                         const Eigen::Vector3d &p_gt_0, Eigen::Matrix3d &R, Eigen::Vector3d &t) {
  Eigen::Matrix3d g_rot = ov_core::quat_2_Rot(q_gt_0).transpose();
  Eigen::Matrix3d est_rot = ov_core::quat_2_Rot(q_es_0).transpose();
  Eigen::Matrix3d C_R = est_rot * g_rot.transpose();
  double theta = get_best_yaw(C_R);
  R = ov_core::rot_z(theta);
  t.noalias() = p_gt_0 - R * p_es_0;
}
#endif

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

#if ROS_AVAILABLE == 1
  // Launch our ros node
  ros::init(argc, argv, "test_dynamic_init");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("config_path", config_path, config_path);

  // Topics to publish
  auto pub_pathimu = nh->advertise<nav_msgs::Path>("/ov_msckf/pathimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathimu.getTopic().c_str());
  auto pub_pathgt = nh->advertise<nav_msgs::Path>("/ov_msckf/pathgt", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathgt.getTopic().c_str());
  auto pub_loop_point = nh->advertise<sensor_msgs::PointCloud>("/ov_msckf/loop_feats", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_loop_point.getTopic().c_str());
  auto pub_points_sim = nh->advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_sim", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_sim.getTopic().c_str());
#endif

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#endif

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create the simulator
  InertialInitializerOptions params;
  params.print_and_load(parser);
  params.print_and_load_simulation(parser);
  Simulator sim(params);

  // Our initialization class objects
  auto imu_readings = std::make_shared<std::vector<ov_core::ImuData>>();
  auto tracker = std::make_shared<ov_core::TrackSIM>(params.camera_intrinsics, 0);
  auto initializer = std::make_shared<DynamicInitializer>(params, tracker->get_feature_database(), imu_readings);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Buffer our camera image
  double buffer_timecam = -1;
  std::vector<int> buffer_camids;
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> buffer_feats;

  // Continue to simulate until we have processed all the measurements
  signal(SIGINT, signal_callback_handler);
  while (sim.ok()) {

    // IMU: get the next simulated IMU measurement if we have it
    ov_core::ImuData message_imu;
    bool hasimu = sim.get_next_imu(message_imu.timestamp, message_imu.wm, message_imu.am);
    if (hasimu) {
      imu_readings->push_back(message_imu);
    }

    // CAM: get the next simulated camera uv measurements if we have them
    double time_cam;
    std::vector<int> camids;
    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;
    bool hascam = sim.get_next_cam(time_cam, camids, feats);
    if (hascam) {

      // Pass to our feature database / tracker
      if (buffer_timecam != -1) {

        // Feed it
        tracker->feed_measurement_simulation(buffer_timecam, buffer_camids, buffer_feats);

        // Display the resulting tracks
        // cv::Mat img_history;
        // tracker->display_history(img_history, 255, 255, 0, 255, 255, 255);
        // cv::imshow("Track History", img_history);
        // cv::waitKey(1);
      }
      buffer_timecam = time_cam;
      buffer_camids = camids;
      buffer_feats = feats;

      // Return states of our initializer
      double timestamp = -1;
      Eigen::MatrixXd covariance;
      std::vector<std::shared_ptr<ov_type::Type>> order;
      std::shared_ptr<ov_type::IMU> _imu;
      std::map<double, std::shared_ptr<ov_type::PoseJPL>> _clones_IMU;
      std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> _features_SLAM;
      std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>> _calib_IMUtoCAM;
      std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>> _cam_intrinsics;

      // First we will try to make sure we have all the data required for our initialization
      boost::posix_time::ptime rT1 = boost::posix_time::microsec_clock::local_time();
      bool success =
          initializer->initialize(timestamp, covariance, order, _imu, _clones_IMU, _features_SLAM, _calib_IMUtoCAM, _cam_intrinsics);
      boost::posix_time::ptime rT2 = boost::posix_time::microsec_clock::local_time();
      double time = (rT2 - rT1).total_microseconds() * 1e-6;
      if (success) {

        // debug that we finished!
        // TODO: do some evaulation here?
        PRINT_INFO(GREEN "success! got initialized state information (%.4f seconds)\n" RESET, time);

#if ROS_AVAILABLE == 1
        // Align the groundtruth to the current estimate yaw
        // Only use the first frame so we can see the drift
        double oldestpose_time = -1;
        std::shared_ptr<ov_type::PoseJPL> oldestpose = nullptr;
        for (auto const &_pose : _clones_IMU) {
          if (oldestpose == nullptr || _pose.first < oldestpose_time) {
            oldestpose_time = _pose.first;
            oldestpose = _pose.second;
          }
        }
        assert(oldestpose != nullptr);
        Eigen::Vector4d q_es_0, q_gt_0;
        Eigen::Vector3d p_es_0, p_gt_0;
        q_es_0 = oldestpose->quat();
        p_es_0 = oldestpose->pos();
        Eigen::Matrix<double, 17, 1> gt_imustate_0;
        assert(sim.get_state(oldestpose_time + sim.get_true_parameters().calib_camimu_dt, gt_imustate_0));
        q_gt_0 = gt_imustate_0.block(1, 0, 4, 1);
        p_gt_0 = gt_imustate_0.block(5, 0, 3, 1);
        Eigen::Matrix3d R_ESTtoGT;
        Eigen::Vector3d t_ESTinGT;
        align_posyaw_single(q_es_0, p_es_0, q_gt_0, p_gt_0, R_ESTtoGT, t_ESTinGT);

        // Pose states
        nav_msgs::Path arrEST, arrGT;
        arrEST.header.stamp = ros::Time::now();
        arrEST.header.frame_id = "global";
        arrGT.header.stamp = ros::Time::now();
        arrGT.header.frame_id = "global";
        for (auto const &_pose : _clones_IMU) {
          geometry_msgs::PoseStamped poseEST, poseGT;
          poseEST.header.stamp = ros::Time(_pose.first);
          poseEST.header.frame_id = "global";
          poseEST.pose.orientation.x = _pose.second->quat()(0, 0);
          poseEST.pose.orientation.y = _pose.second->quat()(1, 0);
          poseEST.pose.orientation.z = _pose.second->quat()(2, 0);
          poseEST.pose.orientation.w = _pose.second->quat()(3, 0);
          poseEST.pose.position.x = _pose.second->pos()(0, 0);
          poseEST.pose.position.y = _pose.second->pos()(1, 0);
          poseEST.pose.position.z = _pose.second->pos()(2, 0);
          Eigen::Matrix<double, 17, 1> gt_imustate;
          assert(sim.get_state(_pose.first + sim.get_true_parameters().calib_camimu_dt, gt_imustate));
          gt_imustate.block(1, 0, 4, 1) = ov_core::quat_multiply(ov_core::rot_2_quat(R_ESTtoGT.transpose()), gt_imustate.block(1, 0, 4, 1));
          gt_imustate.block(5, 0, 3, 1) = R_ESTtoGT.transpose() * (gt_imustate.block(5, 0, 3, 1) - t_ESTinGT);
          poseGT.header.stamp = ros::Time(_pose.first);
          poseGT.header.frame_id = "global";
          poseGT.pose.orientation.x = gt_imustate(1);
          poseGT.pose.orientation.y = gt_imustate(2);
          poseGT.pose.orientation.z = gt_imustate(3);
          poseGT.pose.orientation.w = gt_imustate(4);
          poseGT.pose.position.x = gt_imustate(5);
          poseGT.pose.position.y = gt_imustate(6);
          poseGT.pose.position.z = gt_imustate(7);
          arrEST.poses.push_back(poseEST);
          arrGT.poses.push_back(poseGT);
        }
        pub_pathimu.publish(arrEST);
        pub_pathgt.publish(arrGT);

        // Features ESTIMATES pointcloud
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.frame_id = "global";
        point_cloud.header.stamp = ros::Time::now();
        for (auto const &featpair : _features_SLAM) {
          geometry_msgs::Point32 p;
          p.x = (float)featpair.second->get_xyz(false)(0, 0);
          p.y = (float)featpair.second->get_xyz(false)(1, 0);
          p.z = (float)featpair.second->get_xyz(false)(2, 0);
          point_cloud.points.push_back(p);
        }
        pub_loop_point.publish(point_cloud);

        // Features GROUNDTRUTH pointcloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = "global";
        cloud.header.stamp = ros::Time::now();
        cloud.width = 3 * _features_SLAM.size();
        cloud.height = 1;
        cloud.is_bigendian = false;
        cloud.is_dense = false; // there may be invalid points
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(3 * _features_SLAM.size());
        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
        for (auto &featpair : _features_SLAM) {
          // TrackSIM adds 1 to sim id if num_aruco is zero
          Eigen::Vector3d feat = sim.get_map().at(featpair.first - 1);
          feat = R_ESTtoGT.transpose() * (feat - t_ESTinGT);
          *out_x = (float)feat(0);
          ++out_x;
          *out_y = (float)feat(1);
          ++out_y;
          *out_z = (float)feat(2);
          ++out_z;
        }
        pub_points_sim.publish(cloud);

        // wait for user approval
        do {
          cout << '\n' << "Press a key to continue...";
        } while (cin.get() != '\n');
#endif

        // Reset our tracker and simulator so we can try to init again
        if (params.sim_do_perturbation) {
          sim.perturb_parameters(params);
        }
        imu_readings = std::make_shared<std::vector<ov_core::ImuData>>();
        tracker = std::make_shared<ov_core::TrackSIM>(params.camera_intrinsics, 0);
        initializer = std::make_shared<DynamicInitializer>(params, tracker->get_feature_database(), imu_readings);
      } else if (timestamp != -1) {
        PRINT_INFO(RED "failed (%.4f seconds)\n\n" RESET, time);
      }
    }
  }

  // Done!
  return EXIT_SUCCESS;
}