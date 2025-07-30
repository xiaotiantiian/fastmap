#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <deque>
#include <algorithm>
#include <iostream>
// #include "/home/tian/workspace/fastmap_git/src/fastmap/voxel_plus/src/map_builder/lio_builder.h"
#include "/home/tian/workspace/fastmap_git/src/fastmap/voxel_plus/src/map_builder/ieskf.h"

// 前置声明（替代 #include "lio_builder.h"）
namespace kf {
    struct State;  // 前置声明 kf::State
}

namespace lio {
    struct SyncPackage;  // 前置声明 lio::SyncPackage
}

namespace tian
{
    class Lidar_processing
    {
    public:
        Lidar_processing() = default;
        ~Lidar_processing() = default;

        // int sweep_mode = 0;
        // int conbined_mode = 0;

    private:
        int pop_point = 0;
        int end_point_time = 0;
        int meas_num = 0;
        using PointType = pcl::PointXYZINormal;
        using PointCloudXYZI = pcl::PointCloud<PointType>;

    public:
        void splitByTime(
            const PointCloudXYZI::Ptr &input_cloud,
            double msg_timestamp,
            int num_splits,
            std::deque<PointCloudXYZI::Ptr> &lidar_buffer,
            std::deque<double> &time_buffer);

        void splitByCount(
            const PointCloudXYZI::Ptr &input_cloud,
            double msg_timestamp,
            int nun_splits,
            std::deque<PointCloudXYZI::Ptr> &lidar_buffer,
            std::deque<double> &time_buffer);

        void fuseClouds(
            const PointCloudXYZI::Ptr &feats_undistort,
            const PointCloudXYZI::Ptr &feats_undistort_prev,
            const kf::State &state_point,
            const kf::State &state_prev,
            PointCloudXYZI::Ptr &feats_combined);
    };
} // namespace tian
