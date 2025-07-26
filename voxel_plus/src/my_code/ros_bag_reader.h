#pragma once
#include <string>
#include <thread>
#include <atomic>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>

namespace meskernel
{
  class BagReader
  {
  public:
    BagReader(int play_times = 1);
    ~BagReader() = default;

    using ImuCallBack = std::function<void(const sensor_msgs::Imu::ConstPtr &)>; // 使用 std::function 允许绑定任意可调用对象
    using LivoxCallBack = std::function<void(const livox_ros_driver2::CustomMsg::ConstPtr &)>;
    using LivoxCallBack2 = std::function<void(const sensor_msgs::PointCloud2::ConstPtr &)>;

    // 注册IMU回调函数
    void registerImuCallback(const ImuCallBack &callback);

    // 注册Livox回调函数
    void registerLivoxCallback(const LivoxCallBack &callback);
    void registerLivoxCallback2(const LivoxCallBack2 &callback);

    // 初始化函数，设置bag文件路径和订阅的主题
    void init(const std::string &bag_file, const std::vector<std::string> &topics);

    // 启动读取线程
    void start();
    void start2();

    // 停止读取线程
    void stop();

  private:
    // 处理循环函数，读取bag文件中的消息
    void processLoop();
    void processLoop2();

  private:
    int play_times_;
    std::string bag_file_;
    std::vector<std::string> topics_;
    ImuCallBack imu_callback_;
    LivoxCallBack livox_callback_;
    LivoxCallBack2 livox_callback2_;
    std::shared_ptr<rosbag::Bag> bag_ptr_;
    std::shared_ptr<std::thread> process_thread_;
    std::atomic<bool> stop_flag_{false};
  };

} // namespace meskernel