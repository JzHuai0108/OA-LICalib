#pragma once

#include <angles/angles.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <sensor_data/lidar_feature.h>
#include <sensor_msgs/PointCloud2.h>

namespace liso {

class PandarPoints {
 public:
  typedef std::shared_ptr<PandarPoints> Ptr;

  PandarPoints(int ring_no, const LidarDataOptions& options)
      : pandar_ring_No_(ring_no), num_firing_(2000), options_(options) {
      std::string package_name = "oa_licalib";
      std::string PACKAGE_PATH = ros::package::getPath(package_name);
      log_dir_ = PACKAGE_PATH + "/data/bag";
      ROS_INFO_STREAM("Max point distance " << options.max_point_distance_ << ".");
  }

  double RotationTravelledClockwise(double now_angle) const {
      return now_angle;
  }
  /**
   * raw_data or point_raw is unneeded unless lidar intrinsic calibration is enabled.
   */
  void get_organized_and_raw_cloud(
      const sensor_msgs::PointCloud2::ConstPtr &lidarMsg,
      LiDARFeature &output) {
    pcl::PointCloud<hesai_ros::Point> pc_in;
    pcl::fromROSMsg(*lidarMsg, pc_in);
    static int numMsg = 0;
    if (numMsg < 5) {
        pcl::io::savePCDFileASCII(log_dir_ + "/pandar_cloud_in_" + std::to_string(numMsg) + ".pcd", pc_in);
    }

    int ring_number = int(pandar_ring_No_);

    double timebase = lidarMsg->header.stamp.toSec();
    output.timestamp = timebase;

    /// point cloud
    output.full_features->clear();
    output.full_features->height = ring_number;
    output.full_features->width = num_firing_;
    output.full_features->is_dense = false;
    output.full_features->resize(output.full_features->height *
                                 output.full_features->width);

    /// raw_data
    output.raw_data->height = ring_number;
    output.raw_data->width = num_firing_;
    output.raw_data->is_dense = false;
    output.raw_data->resize(output.raw_data->height * output.raw_data->width);

    PosPoint NanPoint;
    NanPoint.x = NAN;
    NanPoint.y = NAN;
    NanPoint.z = NAN;
    NanPoint.timestamp = timebase;

    int num_points = ring_number * num_firing_;
    for (int k = 0; k < num_points; k++) {
      output.full_features->points[k] = NanPoint;
      output.raw_data->points[k] = NanPoint;
    }

    int w = 0;
    int lastPointRing = 0;
    for (int i = 0; i < pc_in.width; ++i) {
        const auto &src = pc_in.at(i);
        int h = src.ring;
        int d = h - lastPointRing;
        if (d < 0) {
            ++w;
        }

        lastPointRing = h;

        double depth = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
        if (depth > options_.max_point_distance_) continue;

        PosPoint dst_point;
        dst_point.x = src.x;
        dst_point.y = src.y;
        dst_point.z = src.z;
        dst_point.timestamp = src.timestamp;

        PosPoint point_raw;
        // t_offset wrt. first point
        point_raw.timestamp = src.timestamp;
        // laser id
        point_raw.x = src.ring;
        // angle rad
        double horizon_angle = atan2(src.y, src.x);
        point_raw.y = RotationTravelledClockwise(horizon_angle);
        // range m
        point_raw.z = depth;

        output.full_features->at(w, h) = dst_point;
        output.raw_data->at(w, h) = point_raw;
    }
    if (numMsg < 5) {
        ROS_INFO_STREAM("Last column index of Pandar scan " << w << " should be " << (num_firing_ - 1));
        pcl::io::savePCDFileASCII(
            log_dir_ + "/pandar_cloud_out_" + std::to_string(numMsg) + ".pcd",
            *output.full_features);
    }
    numMsg++;
  }

 private:
  int pandar_ring_No_;

  int num_firing_;
  std::string log_dir_;
  LidarDataOptions options_;
};
}  // namespace liso
