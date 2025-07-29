#include "pointcloud_split.h"

namespace tian
{
    void Lidar_processing::splitByTime(
        const PointCloudXYZI::Ptr &input_cloud,
        double msg_timestamp,
        int num_splits,
        std::deque<PointCloudXYZI::Ptr> &lidar_buffer,
        std::deque<double> &time_buffer)
    {
        /*** 按时间分割点云 ***/
        // 1. 按时间戳排序点云
        std::sort(input_cloud->points.begin(), input_cloud->points.end(),
                  [](const PointType &a, const PointType &b)
                  {
                      return a.curvature < b.curvature;
                  });

        // 最晚时刻点的相对时间戳大于0.1s 移除最后一个点
        while ((input_cloud->points.back().curvature / 1000.0 - input_cloud->points.front().curvature / 1000.0 > 0.1))
        {
            input_cloud->points.pop_back();
            pop_point++;
            std::cout << "pop_point_num: " << pop_point << std::endl;
        }

        if (input_cloud->points.empty())
        {
            return;
        }

        // 2. 计算时间范围
        double frame_start_time = 0.00;
        double frame_end_time = input_cloud->points.back().curvature / 1000.0;
        double frame_duration = frame_end_time - frame_start_time; // 原始单帧时间
        double split_duration = frame_duration / num_splits;       // 分割后单帧时间

        // 3. 分割点云
        for (int i = 0; i < num_splits; ++i)
        {
            PointCloudXYZI::Ptr split_cloud(new PointCloudXYZI());

            // 计算分割后每一帧的开始和结束时间（相对时间）
            double split_start = frame_start_time + i * split_duration;
            double split_end = (i == num_splits - 1) ? frame_end_time : frame_start_time + (i + 1) * split_duration;

            // 提取当前时间段的点
            for (const auto &point : input_cloud->points)
            {
                double pt_time = point.curvature / 1000.0;
                if (pt_time > split_start && pt_time <= split_end)
                {
                    PointType new_point = point;
                    if (i > 0)
                    {
                        new_point.curvature = (pt_time - split_start) * 1000.0;
                    }
                    split_cloud->points.push_back(new_point);
                }
            }

            // 设置点云属性并存入buffer
            if (!split_cloud->points.empty())
            {
                split_cloud->width = split_cloud->points.size();
                split_cloud->height = 1;
                split_cloud->is_dense = true;

                double split_timestamp;
                if (i == 0)
                {
                    end_point_time = split_cloud->points.back().curvature / 1000;
                    split_timestamp = msg_timestamp;
                }
                else
                {
                    split_timestamp = msg_timestamp + end_point_time;
                }

                lidar_buffer.push_back(split_cloud);
                time_buffer.push_back(split_timestamp);
            }
        }
    }

    void Lidar_processing::splitByCount(
        const PointCloudXYZI::Ptr &input_cloud,
        double msg_timestamp,
        int num_splits,
        std::deque<PointCloudXYZI::Ptr> &lidar_buffer,
        std::deque<double> &time_buffer)
    {
        /*** 按数量分割点云 ***/
        // 1. 按时间戳排序点云
        std::sort(input_cloud->points.begin(), input_cloud->points.end(),
                  [](const PointType &a, const PointType &b)
                  {
                      return a.curvature < b.curvature;
                  });

        if (input_cloud->points.empty())
        {
            return;
        }

        // 2. 计算每帧应有的点数
        size_t total_points = input_cloud->points.size();
        size_t points_per_split = total_points / num_splits;

        // 3. 分割点云
        for (int i = 0; i < num_splits; ++i)
        {
            PointCloudXYZI::Ptr split_cloud(new PointCloudXYZI());

            // 计算当前分割的起点和终点索引
            size_t start_idx = i * points_per_split;
            size_t end_idx = (i == num_splits - 1) ? total_points : (i + 1) * points_per_split;

            // 获取当前段起始时间
            double segment_start_time = input_cloud->points[start_idx].curvature / 1000.0;

            // 提取当前分段的点
            for (size_t j = start_idx; j < end_idx; ++j)
            {
                PointType new_point = input_cloud->points[j];
                if (i > 0)
                {
                    double pt_time = new_point.curvature / 1000.0;
                    new_point.curvature = (pt_time - segment_start_time) * 1000.0;
                }
                split_cloud->points.push_back(new_point);
            }

            // 设置点云属性并存入buffer
            if (!split_cloud->points.empty())
            {
                split_cloud->width = split_cloud->points.size();
                split_cloud->height = 1;
                split_cloud->is_dense = true;

                double split_timestamp;
                if (i == 0)
                    split_timestamp = msg_timestamp;
                else
                    split_timestamp = msg_timestamp + input_cloud->points[start_idx].curvature / 1000.0;

                lidar_buffer.push_back(split_cloud);
                time_buffer.push_back(split_timestamp);
            }
        }
    }

    void Lidar_processing::fuseClouds(
        const PointCloudXYZI::Ptr &feats_undistort,
        const PointCloudXYZI::Ptr &feats_undistort_prev,
        const kf::State &state_point,
        const kf::State &state_prev,
        PointCloudXYZI::Ptr &feats_combined)
    {
        // 校验输入指针
        if (!feats_undistort || !feats_undistort_prev || !feats_combined)
        {
            ROS_ERROR("Null pointer passed to fuseClouds!");
            return;
        }
        // 清空并预分配内存
        feats_combined->clear();
        feats_combined->reserve(feats_undistort->points.size() + feats_undistort_prev->points.size());

        // 变换上一帧点云到当前帧坐标系
        if (!feats_undistort_prev->empty())
        {
            for (const auto &pt : feats_undistort_prev->points)
            {
                PointType new_pt;
                Eigen::Vector3d p_lidar_prev(pt.x, pt.y, pt.z);

                // 坐标系变换：prev_lidar -> prev_body -> prev_world -> curr_world -> curr_body -> curr_lidar
                Eigen::Vector3d p_lidar_curr = state_point.rot_ext.transpose() * (state_point.rot.transpose() * (state_prev.rot * (state_prev.rot_ext * p_lidar_prev + state_prev.pos_ext) +
                                                                                                          state_prev.pos - state_point.pos) -
                                                                           state_point.pos_ext);

                new_pt.x = p_lidar_curr.x();
                new_pt.y = p_lidar_curr.y();
                new_pt.z = p_lidar_curr.z();
                new_pt.intensity = pt.intensity;
                feats_combined->push_back(new_pt);
            }
        }

        // 合并当前帧点云
        // 合并当前帧点云（需校验非空）
        if (!feats_undistort->empty())
        {
            *feats_combined += *feats_undistort;
        }
    }

} // namespace tian