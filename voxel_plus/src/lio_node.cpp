#include <ros/ros.h>
#include <queue>
#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/lio_builder.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include "interface/PointCloudWithOdom.h"

struct NodeConfig
{
    std::string lidar_topic;
    std::string imu_topic;
    std::string map_frame;
    std::string body_frame;
    double range_min = 0.5;
    double range_max = 20.0;
    int filter_num = 3;
    bool publish_voxel_map = false;
    int publish_voxel_num = 1000;
    bool use_pointcloud2 = false;
};

struct NodeGroupData
{
    double last_imu_time = 0.0;
    double last_lidar_time = 0.0;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    std::deque<lio::IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    bool lidar_pushed = false;
};

class LIONode
{
public:
    LIONode() : nh("~")
    {
        loadConfig();
        initSubScribers();
        initPublishers();
        map_builder.loadConfig(lio_config);
        main_loop = nh.createTimer(ros::Duration(0.02), &LIONode::mainCB, this);
        voxel_map_loop = nh.createTimer(ros::Duration(5.0), &LIONode::voxelTimerCB, this, false, false);
        if (config.publish_voxel_map)
            voxel_map_loop.start();
    }

    void loadConfig()
    {
        nh.param<bool>("use_pointcloud2", config.use_pointcloud2, false); // 加载新参数

        nh.param<std::string>("lidar_topic", config.lidar_topic, "/livox/lidar");
        nh.param<std::string>("imu_topic", config.imu_topic, "/livox/imu");

        nh.param<std::string>("body_frame", config.body_frame, "body");
        nh.param<std::string>("map_frame", config.map_frame, "map");

        nh.param<int>("filter_num", config.filter_num, 3);
        nh.param<double>("range_min", config.range_min, 0.5);
        nh.param<double>("range_max", config.range_max, 20.0);
        nh.param<bool>("publish_voxel_map", config.publish_voxel_map, false);
        nh.param<int>("publish_voxel_num", config.publish_voxel_num, 1000);

        nh.param<double>("scan_resolution", lio_config.scan_resolution, 0.1);
        nh.param<double>("voxel_size", lio_config.voxel_size, 0.4);

        nh.param<double>("merge_thresh_for_angle", lio_config.merge_thresh_for_angle, 0.1);
        nh.param<double>("merge_thresh_for_distance", lio_config.merge_thresh_for_distance, 0.04);

        nh.param<int>("max_point_thresh", lio_config.max_point_thresh, 100);
        nh.param<int>("update_size_thresh", lio_config.update_size_thresh, 10);

        nh.param<int>("map_capacity", lio_config.map_capacity, 100000);
        nh.param<bool>("gravity_align", lio_config.gravity_align, true);
        nh.param<int>("imu_init_num", lio_config.imu_init_num, 20);
        nh.param<double>("na", lio_config.na, 0.01);
        nh.param<double>("ng", lio_config.ng, 0.01);
        nh.param<double>("nbg", lio_config.nbg, 0.0001);
        nh.param<double>("nba", lio_config.nba, 0.0001);
        nh.param<int>("opti_max_iter", lio_config.opti_max_iter, 5);
        std::vector<double> r_il, p_il;
        nh.param<std::vector<double>>("r_il", r_il, std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
        assert(r_il.size() == 9);
        lio_config.r_il << r_il[0], r_il[1], r_il[2], r_il[3], r_il[4], r_il[5], r_il[6], r_il[7], r_il[8];

        nh.param<std::vector<double>>("p_il", p_il, std::vector<double>{0, 0, 0});
        assert(p_il.size() == 3);
        lio_config.p_il << p_il[0], p_il[1], p_il[2];
    }

    void initSubScribers()
    {
        if (config.use_pointcloud2)
        {
            lidar_sub = nh.subscribe(config.lidar_topic, 10000, &LIONode::lidarCB2, this);
        }
        else
        {
            lidar_sub = nh.subscribe(config.lidar_topic, 10000, &LIONode::lidarCB, this);
        }
        imu_sub = nh.subscribe(config.imu_topic, 10000, &LIONode::imuCB, this);
    }

    void initPublishers()
    {
        odom_pub = nh.advertise<nav_msgs::Odometry>("slam_odom", 1000);
        body_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("body_cloud", 1000);
        world_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("world_cloud", 1000);
        voxel_map_pub = nh.advertise<visualization_msgs::MarkerArray>("voxel_map", 1000);
        pointcloud_with_odom_pub = nh.advertise<interface::PointCloudWithOdom>("cloud_with_odom", 1000);
    }

    void imuCB(const sensor_msgs::Imu::ConstPtr msg)
    {
        std::lock_guard<std::mutex> lock(group_data.imu_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < group_data.last_imu_time)
        {
            ROS_WARN("IMU TIME SYNC ERROR");
            group_data.imu_buffer.clear();
        }
        group_data.last_imu_time = timestamp;
        group_data.imu_buffer.emplace_back(Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                                           Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                           timestamp);
    }

    void lidarCB(const livox_ros_driver2::CustomMsg::ConstPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        livox2pcl(msg, cloud, config.filter_num, config.range_min, config.range_max);
        std::lock_guard<std::mutex> lock(group_data.lidar_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < group_data.last_lidar_time)
        {
            ROS_WARN("LIDAR TIME SYNC ERROR");
            group_data.lidar_buffer.clear();
        }
        group_data.last_lidar_time = timestamp;
        group_data.lidar_buffer.emplace_back(timestamp, cloud);
    }

    void lidarCB2(const sensor_msgs::PointCloud2::ConstPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pointcloud2_to_pcl(msg, cloud, config.filter_num, config.range_min, config.range_max);
        std::lock_guard<std::mutex> lock(group_data.lidar_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < group_data.last_lidar_time)
        {
            ROS_WARN("LIDAR TIME SYNC ERROR");
            group_data.lidar_buffer.clear();
        }
        group_data.last_lidar_time = timestamp;
        group_data.lidar_buffer.emplace_back(timestamp, cloud);
    }

    bool syncPackage()
    {
        if (group_data.imu_buffer.empty() || group_data.lidar_buffer.empty())
            return false;
        // 同步点云数据
        if (!group_data.lidar_pushed)
        {
            sync_pack.cloud = group_data.lidar_buffer.front().second;
            sync_pack.cloud_start_time = group_data.lidar_buffer.front().first;
            sync_pack.cloud_end_time = sync_pack.cloud_start_time + sync_pack.cloud->points.back().curvature / double(1000.0);
            group_data.lidar_pushed = true;
        }
        // 等待IMU的数据
        if (group_data.last_imu_time < sync_pack.cloud_end_time)
            return false;

        sync_pack.imus.clear();

        // 同步IMU的数据
        // IMU的最后一帧数据的时间小于点云最后一个点的时间
        while (!group_data.imu_buffer.empty() && (group_data.imu_buffer.front().timestamp < sync_pack.cloud_end_time))
        {
            sync_pack.imus.push_back(group_data.imu_buffer.front());
            group_data.imu_buffer.pop_front();
        }
        group_data.lidar_buffer.pop_front();
        group_data.lidar_pushed = false;
        return true;
    }

    void mainCB(const ros::TimerEvent &e)
    {
        if (!syncPackage())
            return;
        map_builder.process(sync_pack);
        if (map_builder.status != lio::LIOStatus::LIO_MAPPING)
            return;
        state = map_builder.kf.x();
        br.sendTransform(eigen2Transform(state.rot, state.pos, config.map_frame, config.body_frame, sync_pack.cloud_end_time));
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr body_cloud = map_builder.lidarToBody(sync_pack.cloud);
        publishCloud(body_cloud_pub, body_cloud, config.body_frame, sync_pack.cloud_end_time);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_cloud = map_builder.lidarToWorld(sync_pack.cloud);
        publishCloud(world_cloud_pub, world_cloud, config.map_frame, sync_pack.cloud_end_time);

        publishCloudWithOdom(body_cloud, config.map_frame, config.body_frame, sync_pack.cloud_end_time);
    }
    void voxelTimerCB(const ros::TimerEvent &event)
    {
        std::shared_ptr<lio::VoxelMap> voxel_map = map_builder.map;
        if (voxel_map->cache.size() < 10)
            return;
        if (voxel_map_pub.getNumSubscribers() < 1)
            return;
        voxel_map_pub.publish(voxel2MarkerArray(voxel_map, config.map_frame, ros::Time::now().toSec(), config.publish_voxel_num, voxel_map->voxel_size * 0.8));
    }

    void publishCloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, std::string &frame_id, const double &time)
    {
        if (pub.getNumSubscribers() < 1)
            return;
        pub.publish(pcl2msg(cloud, frame_id, time));
    }

    void publishCloudWithOdom(pcl::PointCloud<pcl::PointXYZINormal>::Ptr _cloud, std::string &_frame_id, std::string &_child_frame, double _timestamp)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pub_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*_cloud, *pub_cloud);

        if (pointcloud_with_odom_pub.getNumSubscribers() < 1)
            return;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*pub_cloud, cloud_msg);
        if (_timestamp < 0)
            cloud_msg.header.stamp = ros::Time().now();
        else
            cloud_msg.header.stamp = ros::Time().fromSec(_timestamp);
        cloud_msg.header.frame_id = _child_frame;

        interface::PointCloudWithOdom msg;
        msg.header.stamp = ros::Time().fromSec(_timestamp);
        msg.header.frame_id = _frame_id;
        msg.cloud = cloud_msg;
        msg.pose.pose.position.x = state.pos.x();
        msg.pose.pose.position.y = state.pos.y();
        msg.pose.pose.position.z = state.pos.z();
        Eigen::Quaterniond q(state.rot);
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg.pose.covariance.data()) = map_builder.kf.P().block<6, 6>(0, 0);
        pointcloud_with_odom_pub.publish(msg);
    }

public:
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;
    NodeConfig config;
    NodeGroupData group_data;
    lio::SyncPackage sync_pack;
    ros::Timer main_loop;
    ros::Timer voxel_map_loop;

    std::string map_frame;
    std::string body_frame;

    ros::Subscriber lidar_sub;
    ros::Subscriber imu_sub;

    ros::Publisher odom_pub;
    ros::Publisher body_cloud_pub;
    ros::Publisher world_cloud_pub;
    ros::Publisher voxel_map_pub;
    ros::Publisher pointcloud_with_odom_pub;

    lio::LIOBuilder map_builder;

    lio::LIOConfig lio_config;

    kf::State state;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_node");
    LIONode lio_node;
    ros::spin();
    return 0;
}