// #include <glog/logging.h>
#include <ros/console.h>
#define LOG(severity) ROS_##severity##_STREAM
#include "ros_bag_reader.h"

namespace meskernel
{

  void BagReader::registerImuCallback(const ImuCallBack &callback)
  {
    imu_callback_ = callback;
  }

  void BagReader::registerLivoxCallback(const LivoxCallBack &callback)
  {
    livox_callback_ = callback;
  }

  void BagReader::registerLivoxCallback2(const LivoxCallBack2 &callback)
  {
    livox_callback2_ = callback;
  }

  void BagReader::init(const std::string &bag_file, const std::vector<std::string> &topics)
  {
    bag_file_ = bag_file;
    topics_ = topics;
    stop_flag_.store(false);
  }

  void BagReader::start()
  {
    bag_ptr_ = std::make_shared<rosbag::Bag>();
    bag_ptr_->open(bag_file_, rosbag::bagmode::Read);
    process_thread_ = std::make_shared<std::thread>(&BagReader::processLoop, this);
  }

    void BagReader::start2()
  {
    bag_ptr_ = std::make_shared<rosbag::Bag>();
    bag_ptr_->open(bag_file_, rosbag::bagmode::Read);
    process_thread_ = std::make_shared<std::thread>(&BagReader::processLoop2, this);
  }

  void BagReader::stop()
  {
    stop_flag_.store(true);
    bag_ptr_->close();
    if (process_thread_->joinable())
    {
      process_thread_->join();
    }
  }

  void BagReader::processLoop()
  {
    rosbag::View view(*bag_ptr_, rosbag::TopicQuery(topics_));

    ros::Time last_msg_time;
    for (const rosbag::MessageInstance &msg : view)
    {
      if (stop_flag_)
      {
        return;
      }

      ros::Time cur_msg_time = msg.getTime();
      if (!last_msg_time.isZero())
      {
        ros::Duration duration = cur_msg_time - last_msg_time;
        std::this_thread::sleep_for(std::chrono::nanoseconds(duration.toNSec() / play_times)); // 倍数读取
      }
      last_msg_time = cur_msg_time;

      if (msg.getTopic() == topics_[0])
      {
        livox_callback_(msg.instantiate<livox_ros_driver2::CustomMsg>());
        // livox_callback_(msg.instantiate<sensor_msgs::PointCloud2>());
        continue;
      }

      if (msg.getTopic() == topics_[1])
      {
        imu_callback_(msg.instantiate<sensor_msgs::Imu>());
        continue;
      }
    }

    ROS_WARN_STREAM("bag reading finished...");
  }
  // void BagReader::processLoop()
  // {
  //   // 1. 创建时间戳记录文件
  //   std::string log_path = "/home/tian/workspace/fast_lio_raw/imu_timestamps.csv";
  //   std::ofstream timestamp_file(log_path);
  //   timestamp_file << std::fixed << std::setprecision(9); // 纳秒级精度
  //   timestamp_file << "timestamp\n";                      // CSV标题

  //   rosbag::View view(*bag_ptr_, rosbag::TopicQuery(topics_));
  //   ros::Time last_msg_time;

  //   for (const rosbag::MessageInstance &msg : view)
  //   {
  //     if (stop_flag_)
  //       break;

  //     // 2. 时间控制（倍速播放）
  //     ros::Time cur_msg_time = msg.getTime();
  //     if (!last_msg_time.isZero())
  //     {
  //       std::this_thread::sleep_for(
  //           std::chrono::nanoseconds((cur_msg_time - last_msg_time).toNSec() / play_times));
  //     }
  //     last_msg_time = cur_msg_time;

  //     if (msg.getTopic() == topics_[0])
  //     {
  //       livox_callback_(msg.instantiate<livox_ros_driver::CustomMsg>());
  //       continue;
  //     }
  //     // 3. 仅记录IMU时间戳
  //     if (msg.getTopic() == topics_[1])
  //     { // 假设topics_[1]是IMU话题
  //       auto imu_msg = msg.instantiate<sensor_msgs::Imu>();
  //       if (imu_msg)
  //       {
  //         timestamp_file << imu_msg->header.stamp.toSec() << "\n";
  //         imu_callback_(imu_msg); // 保持原有回调
  //       }
  //     }
  //   }

  //   timestamp_file.close();
  //   ROS_WARN_STREAM("bag reading finished...");
  //   ROS_WARN_STREAM("IMU timestamps saved to imu_timestamps.csv");
  // }

  void BagReader::processLoop2()
  {
    rosbag::View view(*bag_ptr_, rosbag::TopicQuery(topics_));

    ros::Time last_msg_time;
    for (const rosbag::MessageInstance &msg : view)
    {
      if (stop_flag_)
      {
        return;
      }

      ros::Time cur_msg_time = msg.getTime();
      if (!last_msg_time.isZero())
      {
        ros::Duration duration = cur_msg_time - last_msg_time;
        std::this_thread::sleep_for(std::chrono::nanoseconds(duration.toNSec() / play_times));
      }
      last_msg_time = cur_msg_time;

      if (msg.getTopic() == topics_[0])
      {
        livox_callback2_(msg.instantiate<sensor_msgs::PointCloud2>()); // 使用 livox_callback2_
        continue;
      }

      if (msg.getTopic() == topics_[1])
      {
        imu_callback_(msg.instantiate<sensor_msgs::Imu>());
        continue;
      }
    }

    ROS_WARN_STREAM("bag reading finished...");
  }
}

// namespace meskernel