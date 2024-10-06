#include <ros/ros.h>  // 包含ROS的核心头文件
#include <sensor_msgs/LaserScan.h>  // 包含激光扫描消息类型的头文件
#include <vector>  // 包含vector容器
#include <cmath>  // 包含数学函数

class ScanFilter {
public:
    ScanFilter() : nh_("~") {  // 构造函数，初始化私有节点句柄
        // 订阅原始激光扫描话题
        scan_sub_ = nh_.subscribe("scan", 1, &ScanFilter::scanCallback, this);
        // 发布过滤后的激光扫描话题
        filtered_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);

        // 从参数服务器获取最小和最大范围参数，如果不存在则使用默认值
        nh_.param("min_range", min_range_, 0.05);
        nh_.param("max_range", max_range_, 30.0);

        // 从参数服务器获取角度范围参数
        std::vector<double> angle_ranges;
        nh_.param("angle_ranges", angle_ranges, std::vector<double>());
        if (angle_ranges.size() % 2 != 0) {
            ROS_ERROR("angle_ranges parameter should have an even number of elements");
            return;
        }
        // 将角度范围参数转换为pair存储
        for (size_t i = 0; i < angle_ranges.size(); i += 2) {
            angle_ranges_.push_back(std::make_pair(angle_ranges[i], angle_ranges[i+1]));
        }
    }

    // 激光扫描回调函数
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        sensor_msgs::LaserScan filtered_scan = *scan_msg;  // 复制原始扫描消息

        // 遍历所有激光点
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            // 计算当前激光点的角度
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            angle = normalizeAngle(angle);  // 将角度归一化到[-π, π]范围

            bool in_range = false;
            // 检查当前角度是否在任何指定的角度范围内
            for (const auto& range : angle_ranges_) {
                if (isAngleInRange(angle, range.first, range.second)) {
                    in_range = true;
                    break;
                }
            }

            // 如果角度不在指定范围内，或距离不在指定范围内，将该点设为无效
            if (!in_range || scan_msg->ranges[i] < min_range_ || scan_msg->ranges[i] > max_range_) {
                filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }

        // 发布过滤后的激光扫描消息
        filtered_scan_pub_.publish(filtered_scan);
    }

private:
    ros::NodeHandle nh_;  // ROS节点句柄
    ros::Subscriber scan_sub_;  // 激光扫描订阅者
    ros::Publisher filtered_scan_pub_;  // 过滤后的激光扫描发布者
    double min_range_;  // 最小有效距离
    double max_range_;  // 最大有效距离
    std::vector<std::pair<double, double>> angle_ranges_;  // 有效角度范围

    // 将角度归一化到[-π, π]范围
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    // 检查给定角度是否在指定的范围内
    bool isAngleInRange(double angle, double start, double end) {
        angle = normalizeAngle(angle);
        start = normalizeAngle(start);
        end = normalizeAngle(end);

        if (start <= end) {
            return angle >= start && angle <= end;
        } else {
            // 处理跨越0/2π的情况
            return angle >= start || angle <= end;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_filter");  // 初始化ROS节点
    ScanFilter scan_filter;  // 创建ScanFilter对象
    ros::spin();  // 进入ROS事件循环
    return 0;
}