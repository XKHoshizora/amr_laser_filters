#include <ros/ros.h>  // 包含ROS的核心头文件，用于ROS节点的创建和管理
#include <sensor_msgs/LaserScan.h>  // 包含激光扫描消息类型的头文件，用于处理LaserScan消息
#include <vector>  // 包含标准库中的vector容器，用于存储和操作数据
#include <cmath>  // 包含数学函数，用于角度计算
#include <algorithm>  // 包含标准算法库，用于操作容器中的元素

class ScanFilter {
public:
    ScanFilter() : nh_("~") {  // 构造函数，使用私有命名空间创建节点句柄（以 "~" 开头）
        // 订阅原始激光扫描数据话题 "/scan"，并指定回调函数 scanCallback 处理接收到的数据
        scan_sub_ = nh_.subscribe("/scan", 1, &ScanFilter::scanCallback, this);
        
        // 创建发布器，用于发布过滤后的激光扫描数据到话题 "/filtered_scan"
        filtered_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/filtered_scan", 1);

        // 从参数服务器获取最小和最大有效范围，若未指定则使用默认值（0.05米和30米）
        nh_.param("min_range", min_range_, 0.05);
        nh_.param("max_range", max_range_, 30.0);

        // 从参数服务器获取激光雷达的有效角度范围，存储在 angle_ranges 容器中
        std::vector<double> angle_ranges;
        nh_.param("angle_ranges", angle_ranges, std::vector<double>());

        // 检查是否传入了偶数个角度参数，角度范围必须成对出现
        if (angle_ranges.size() % 2 != 0) {
            ROS_ERROR("angle_ranges parameter should have an even number of elements");
            return;
        }

        // 将角度范围参数按成对方式（起始角度和结束角度）存储到 angle_ranges_ 容器中
        for (size_t i = 0; i < angle_ranges.size(); i += 2) {
            angle_ranges_.push_back(std::make_pair(angle_ranges[i], angle_ranges[i+1]));
        }
    }

    // 激光扫描数据的回调函数，用于处理接收到的扫描数据并进行过滤
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        // 创建一个新的 LaserScan 消息，并将接收到的扫描数据拷贝到 filtered_scan 中
        sensor_msgs::LaserScan filtered_scan = *scan_msg;

        // 获取激光扫描的距离数据，并引用 ranges 数组
        std::vector<float>& ranges = filtered_scan.ranges;

        // 遍历扫描的所有数据点，根据距离和角度范围过滤掉不符合条件的点
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            // 计算当前激光数据点的角度，角度按递增的方式计算
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

            // 将角度归一化到 [-π, π] 的范围
            angle = normalizeAngle(angle);

            // 判断该角度是否在指定的有效角度范围内
            bool in_range = isAngleInRanges(angle);

            // 如果该数据点的角度在无效范围内，或距离不在 [min_range_, max_range_] 范围内，
            // 则将该数据点的距离值设置为 infinity，表示无效
            if (in_range || scan_msg->ranges[i] < min_range_ || scan_msg->ranges[i] > max_range_) {
                ranges[i] = std::numeric_limits<float>::infinity();  // 设置为无效值
            }
        }

        // 发布过滤后的激光扫描数据
        filtered_scan_pub_.publish(filtered_scan);
    }

private:
    ros::NodeHandle nh_;  // ROS节点句柄，用于与ROS系统进行交互
    ros::Subscriber scan_sub_;  // 订阅者，用于接收激光扫描数据
    ros::Publisher filtered_scan_pub_;  // 发布者，用于发布过滤后的激光扫描数据
    double min_range_;  // 最小有效距离（单位：米）
    double max_range_;  // 最大有效距离（单位：米）
    std::vector<std::pair<double, double>> angle_ranges_;  // 有效角度范围，存储为一对起始和结束角度

    // 将角度归一化到 [-π, π] 的范围，确保角度计算在标准范围内
    double normalizeAngle(double angle) {
        // 如果角度大于 π，减去 2π 将其归一化
        while (angle > M_PI) angle -= 2 * M_PI;
        // 如果角度小于 -π，加上 2π 将其归一化
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;  // 返回归一化后的角度
    }

    // 判断给定的角度是否在指定的多个有效角度范围中
    bool isAngleInRanges(double angle) {
        // 遍历所有有效角度范围
        for (const auto& range : angle_ranges_) {
            // 如果角度在任何一个范围内，返回 true
            if (isAngleInRange(angle, range.first, range.second)) {
                return true;
            }
        }
        // 如果不在任何范围内，返回 false
        return false;
    }

    // 判断给定角度是否在特定的起始角度和结束角度范围内
    bool isAngleInRange(double angle, double start, double end) {
        // 首先将角度和范围的起始和结束角度归一化到 [-π, π] 范围内
        angle = normalizeAngle(angle);
        start = normalizeAngle(start);
        end = normalizeAngle(end);

        // 如果起始角度小于等于结束角度，检查角度是否在范围之间
        if (start <= end) {
            return angle >= start && angle <= end;
        } else {
            // 如果起始角度大于结束角度（跨越了 -π 和 π），检查角度是否在两个分段范围之一
            return angle >= start || angle <= end;
        }
    }
};

int main(int argc, char** argv) {
    // 初始化 ROS 节点，节点名称为 "scan_filter"
    ros::init(argc, argv, "scan_filter");

    // 创建 ScanFilter 对象，进入过滤处理流程
    ScanFilter scan_filter;

    // 进入 ROS 事件循环，等待并处理回调函数
    ros::spin();

    return 0;  // 程序正常结束
}
