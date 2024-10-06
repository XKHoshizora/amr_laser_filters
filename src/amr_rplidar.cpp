#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

class AmrRplidar
{
public:
    AmrRplidar() : nh_("~")
    {
        // 订阅原始的 RPLIDAR 数据
        scan_sub_ = nh_.subscribe("/scan", 1, &AmrRplidar::scanCallback, this);
        
        // 发布转换后的数据
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_converted", 1);

        // 获取参数
        nh_.param<std::string>("frame_id", frame_id_, "laser");
        nh_.param<bool>("invert_x", invert_x_, false);
        nh_.param<double>("angle_offset", angle_offset_, 0.0);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        sensor_msgs::LaserScan scan_out = *scan_in;

        // 更新 frame_id
        scan_out.header.frame_id = frame_id_;

        // 添加 180 度（π 弧度）的补偿
        double compensation = M_PI;

        // 调整角度范围和增量
        scan_out.angle_min = -scan_in->angle_max + angle_offset_ + compensation;
        scan_out.angle_max = -scan_in->angle_min + angle_offset_ + compensation;
        scan_out.angle_increment = -scan_in->angle_increment;

        // 反转数据数组
        std::reverse(scan_out.ranges.begin(), scan_out.ranges.end());
        if (!scan_out.intensities.empty()) {
            std::reverse(scan_out.intensities.begin(), scan_out.intensities.end());
        }

        // 发布转换后的数据
        scan_pub_.publish(scan_out);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher scan_pub_;
    std::string frame_id_;
    bool invert_x_;
    double angle_offset_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amr_rplidar");
    AmrRplidar amr_rplidar;
    ros::spin();
    return 0;
}