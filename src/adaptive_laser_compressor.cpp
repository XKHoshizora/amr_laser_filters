#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <cmath>

/**
 * @brief AdaptiveLaserCompressor 类用于自适应压缩激光雷达数据
 *
 * 该类订阅原始激光雷达数据，将数据点数量压缩到指定的最大值以下，
 * 同时保持360度全方位覆盖和均匀采样。
 */
class AdaptiveLaserCompressor {
   public:
    /**
     * @brief 构造函数，初始化ROS节点句柄、参数、发布者和订阅者
     */
    AdaptiveLaserCompressor() : nh_("~") {
        // 从参数服务器读取最大输出数据点数量，默认为1440
        nh_.param("max_output_points", max_output_points_, 1440);

        // 设置发布者，发布压缩后的激光扫描数据
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/compressed_scan", 1);
        // 设置订阅者，订阅原始激光扫描数据
        sub_ = nh_.subscribe("/scan", 1, &AdaptiveLaserCompressor::scanCallback,
                             this);

        ROS_INFO(
            "Adaptive Laser Compressor initialized with maximum output points: "
            "%d",
            max_output_points_);
    }

   private:
    /**
     * @brief 处理接收到的激光扫描数据的回调函数
     *
     * @param scan_msg 接收到的原始激光扫描数据
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        // 创建一个新的LaserScan消息，初始化为输入消息的内容
        sensor_msgs::LaserScan compressed_scan = *scan_msg;
        int input_points = scan_msg->ranges.size();

        // 计算压缩因子，确保输出点数不超过且尽可能接近最大值
        double compression_factor = std::max(
            1.0, static_cast<double>(input_points) / max_output_points_);
        int actual_output_points =
            static_cast<int>(std::ceil(input_points / compression_factor));

        // 更新角度增量，保持360度覆盖
        compressed_scan.angle_increment =
            scan_msg->angle_increment * compression_factor;

        // 压缩范围和强度数据
        std::vector<float> compressed_ranges;
        std::vector<float> compressed_intensities;
        compressed_ranges.reserve(actual_output_points);
        compressed_intensities.reserve(actual_output_points);

        // 均匀采样原始数据点
        for (int i = 0; i < input_points; i += compression_factor) {
            // 使用std::min防止数组越界
            int index = std::min(static_cast<int>(i), input_points - 1);
            compressed_ranges.push_back(scan_msg->ranges[index]);
            if (!scan_msg->intensities.empty()) {
                compressed_intensities.push_back(scan_msg->intensities[index]);
            }
        }

        // 更新压缩后的激光扫描消息
        compressed_scan.ranges = compressed_ranges;
        compressed_scan.intensities = compressed_intensities;

        // 更新scan_time以反映新的扫描频率
        compressed_scan.scan_time =
            scan_msg->scan_time *
            (static_cast<float>(compressed_ranges.size()) / input_points);

        // 发布压缩后的激光扫描消息
        pub_.publish(compressed_scan);

        // 输出调试信息
        ROS_DEBUG("Compressed laser scan from %d to %lu points (max: %d)",
                  input_points, compressed_ranges.size(), max_output_points_);
    }

    ros::NodeHandle nh_;   // ROS节点句柄
    ros::Publisher pub_;   // 发布压缩后的激光扫描数据的发布者
    ros::Subscriber sub_;  // 订阅原始激光扫描数据的订阅者
    int max_output_points_;  // 最大输出数据点数量
};

/**
 * @brief 主函数
 *
 * 初始化ROS节点，创建AdaptiveLaserCompressor对象，并运行ROS主循环
 */
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "adaptive_laser_compressor");
    // 创建AdaptiveLaserCompressor对象
    AdaptiveLaserCompressor compressor;
    // 运行ROS主循环
    ros::spin();
    return 0;
}