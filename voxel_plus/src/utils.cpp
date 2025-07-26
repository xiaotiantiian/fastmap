#include "utils.h"
#include <ros/ros.h> // 添加 ROS 日志支持
#include <cmath>     // 用于 std::isnan 和 std::isinf
#include <sensor_msgs/point_cloud2_iterator.h>

void pointcloud2_to_pcl(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZINormal>::Ptr out, int filter_num, double range_min, double range_max)
{
    if (!msg || !out) {
        ROS_ERROR("Null input message or output cloud pointer");
        return;
    }

    // 打印字段信息
    // ROS_INFO("PointCloud2 fields:");
    // for (const auto &field : msg->fields) {
    //     ROS_INFO("  name: %s, offset: %u, datatype: %u, count: %u",
    //              field.name.c_str(), field.offset, field.datatype, field.count);
    // }
    // ROS_INFO("PointCloud2: width=%u, height=%u, point_step=%u, is_dense=%d",
    //          msg->width, msg->height, msg->point_step, msg->is_dense);

    // 验证字段
    bool has_x = false, has_y = false, has_z = false, has_intensity = false;
    bool has_time_interval = false, has_offset_time = false, has_return_value = false, has_ring = false;
    for (const auto &field : msg->fields) {
        if (field.name == "x") has_x = true;
        if (field.name == "y") has_y = true;
        if (field.name == "z") has_z = true;
        if (field.name == "intensity") has_intensity = true;
        if (field.name == "time_interval") has_time_interval = true;
        if (field.name == "offset_time") has_offset_time = true;
        if (field.name == "return_value") has_return_value = true;
        if (field.name == "ring") has_ring = true;
    }
    if (!has_x || !has_y || !has_z || !has_intensity || !has_offset_time) {
        ROS_ERROR("Missing required fields (x, y, z, intensity, offset_time)");
        return;
    }

    out->clear();
    out->reserve(msg->width / filter_num + 1);

    // 初始化迭代器
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_time_interval(*msg, "time_interval");
    sensor_msgs::PointCloud2ConstIterator<float> iter_offset_time(*msg, "offset_time");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_return_value(*msg, "return_value");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_ring(*msg, "ring");

    uint valid_num = 0;
    uint nan_count = 0, range_count = 0, livox_count = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity,
           ++iter_time_interval, ++iter_offset_time, ++iter_return_value, ++iter_ring, ++valid_num)
    {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;
        float intensity = *iter_intensity;
        float time_interval = *iter_time_interval;
        float offset_time = *iter_offset_time;
        uint8_t return_value = *iter_return_value;
        uint8_t ring = *iter_ring;

        // // 调试前 100 个点
        // if (valid_num < 100) {
        //     ROS_INFO("Point %u: x=%f, y=%f, z=%f, intensity=%f, time_interval=%f, offset_time=%f, return_value=%u, ring=%u",
        //              valid_num, x, y, z, intensity, time_interval, offset_time, return_value, ring);
        // }

        // NaN/Inf 过滤
        if (std::isnan(x) || std::isnan(y) || std::isnan(z) ||
            std::isinf(x) || std::isinf(y) || std::isinf(z)) {
            nan_count++;
            continue;
        }

        // 范围过滤
        double sq_range = x * x + y * y + z * z;
        if (sq_range <= (range_min * range_min) || sq_range >= (range_max * range_max)) {
            range_count++;
            continue;
        }

        // Livox 过滤（暂时禁用，待验证）
        // if (ring >= 4 || return_value == 0) {
        //     livox_count++;
        //     continue;
        // }

        if (valid_num % filter_num != 0)
            continue;

        pcl::PointXYZINormal p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = intensity;
        p.curvature = offset_time * 1000.0; // 秒转换为毫秒
        p.normal_x = 0.0;
        p.normal_y = 0.0;
        p.normal_z = 0.0;

        out->push_back(p);
    }

    // ROS_INFO("Total points: %u, NaN/Inf filtered: %u, Range filtered: %u, Livox filtered: %u, Valid points after filtering: %lu",
    //          msg->width, nan_count, range_count, livox_count, out->size());

    if (out->empty()) {
        ROS_WARN("No valid points after filtering. Check range_min (%f), range_max (%f), or point data validity.", range_min, range_max);
    }
}

void livox2pcl(const livox_ros_driver2::CustomMsg::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZINormal>::Ptr out, int filter_num, double range_min, double range_max)
{
    int point_num = msg->point_num;
    out->clear();
    out->reserve(point_num / filter_num + 1);
    uint valid_num = 0;
    for (uint i = 0; i < point_num; i++)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            if ((valid_num++) % filter_num != 0)
                continue;
            pcl::PointXYZINormal p;
            p.x = msg->points[i].x;
            p.y = msg->points[i].y;
            p.z = msg->points[i].z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / float(1000000); // 纳秒->毫秒
            double sq_range = p.x * p.x + p.y * p.y + p.z * p.z;
            if (sq_range > (range_min * range_min) && sq_range < range_max * range_max)
            {
                out->push_back(p);
            }
        }
    }
}

sensor_msgs::PointCloud2 pcl2msg(pcl::PointCloud<pcl::PointXYZINormal>::Ptr inp, const std::string &frame_id, const double &timestamp)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*inp, msg);
    if (timestamp < 0)
        msg.header.stamp = ros::Time().now();
    else
        msg.header.stamp = ros::Time().fromSec(timestamp);
    msg.header.frame_id = frame_id;
    return msg;
}

geometry_msgs::TransformStamped eigen2Transform(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos, const std::string &frame_id, const std::string &child_frame_id, const double &timestamp)
{
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = frame_id;
    transform.header.stamp = ros::Time().fromSec(timestamp);
    transform.child_frame_id = child_frame_id;
    transform.transform.translation.x = pos(0);
    transform.transform.translation.y = pos(1);
    transform.transform.translation.z = pos(2);
    Eigen::Quaterniond q = Eigen::Quaterniond(rot);

    transform.transform.rotation.w = q.w();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    return transform;
}

nav_msgs::Odometry eigen2Odometry(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos, const std::string &frame_id, const std::string &child_frame_id, const double &timestamp)
{
    nav_msgs::Odometry odom;
    odom.header.frame_id = frame_id;
    odom.header.stamp = ros::Time().fromSec(timestamp);
    odom.child_frame_id = child_frame_id;
    Eigen::Quaterniond q = Eigen::Quaterniond(rot);
    odom.pose.pose.position.x = pos(0);
    odom.pose.pose.position.y = pos(1);
    odom.pose.pose.position.z = pos(2);

    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    return odom;
}

void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b)
{
    r = 255;
    g = 255;
    b = 255;

    if (v < vmin)
    {
        v = vmin;
    }

    if (v > vmax)
    {
        v = vmax;
    }

    double dr, dg, db;

    if (v < 0.1242)
    {
        db = 0.504 + ((1. - 0.504) / 0.1242) * v;
        dg = dr = 0.;
    }
    else if (v < 0.3747)
    {
        db = 1.;
        dr = 0.;
        dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
    }
    else if (v < 0.6253)
    {
        db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
        dg = 1.;
        dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
    }
    else if (v < 0.8758)
    {
        db = 0.;
        dr = 1.;
        dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
    }
    else
    {
        db = 0.;
        dg = 0.;
        dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
    }

    r = (uint8_t)(255 * dr);
    g = (uint8_t)(255 * dg);
    b = (uint8_t)(255 * db);
}

void calcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q)
{
    Eigen::Matrix3d rot;
    rot.col(0) = x_vec;
    rot.col(1) = y_vec;
    rot.col(2) = z_vec;
    Eigen::Quaterniond eq(rot);
    eq.normalize();
    q.w = eq.w();
    q.x = eq.x();
    q.y = eq.y();
    q.z = eq.z();
}

void calcVectQuation(const Eigen::Vector3d &norm_vec, geometry_msgs::Quaternion &q)
{
    Eigen::Quaterniond rq = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), norm_vec);
    q.w = rq.w();
    q.x = rq.x();
    q.y = rq.y();
    q.z = rq.z();
}

visualization_msgs::MarkerArray voxel2MarkerArray(std::shared_ptr<lio::VoxelMap> map, const std::string &frame_id, const double &timestamp, int max_capacity, double voxel_size)
{
    visualization_msgs::MarkerArray voxel_plane;
    int size = std::min(static_cast<int>(map->cache.size()), max_capacity);

    voxel_plane.markers.reserve(size);
    int count = 0;
    for (auto &k : map->cache)
    {
        if (count >= size)
            break;
        std::shared_ptr<lio::VoxelGrid> grid = map->featmap[k];
        if (!grid->is_plane || grid->update_enable)
            continue;

        Eigen::Vector3d grid_center = grid->center;

        double trace = grid->plane->cov.block<3, 3>(0, 0).trace();
        if (trace >= 0.25)
            trace = 0.25;
        trace = trace * (1.0 / 0.25);
        trace = pow(trace, 0.2);
        uint8_t r, g, b;
        mapJet(trace, 0, 1, r, g, b);
        Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
        double alpha = 0.8;

        visualization_msgs::Marker plane;
        plane.header.frame_id = frame_id;
        plane.header.stamp = ros::Time().fromSec(timestamp);
        plane.ns = "plane";
        plane.id = count++;
        if (!grid->merged)
            plane.type = visualization_msgs::Marker::CYLINDER;
        else
            plane.type = visualization_msgs::Marker::CUBE;
        plane.action = visualization_msgs::Marker::ADD;
        plane.pose.position.x = grid_center[0];
        plane.pose.position.y = grid_center[1];
        plane.pose.position.z = grid_center[2];
        geometry_msgs::Quaternion q;
        calcVectQuation(grid->plane->norm, q);
        plane.pose.orientation = q;
        plane.scale.x = voxel_size;
        plane.scale.y = voxel_size;
        plane.scale.z = 0.01;
        plane.color.a = alpha;
        plane.color.r = plane_rgb[0];
        plane.color.g = plane_rgb[1];
        plane.color.b = plane_rgb[2];
        plane.lifetime = ros::Duration();
        voxel_plane.markers.push_back(plane);
    }
    return voxel_plane;
}