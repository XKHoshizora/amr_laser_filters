#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>

class MedianFilterNode
{
public:
    MedianFilterNode() : nh_("~")
    {
        // 从参数服务器获取参数
        nh_.param("window_size", window_size_, 5);
        nh_.param("max_range", max_range_, 100.0);
        nh_.param("min_range", min_range_, 0.1);
        nh_.param("max_angle", max_angle_, M_PI);
        nh_.param("min_angle", min_angle_, -M_PI);
        
        // 创建发布者和订阅者
        scan_sub_ = nh_.subscribe("/scan", 1, &MedianFilterNode::scanCallback, this);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/median_scan", 1);
    }

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        sensor_msgs::LaserScan filtered_scan = *scan_msg;
        std::vector<float> filtered_ranges;
        
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            std::vector<float> window;
            
            // 收集窗口内的有效测量值
            for (int j = -window_size_/2; j <= window_size_/2; ++j)
            {
                int index = (i + j + scan_msg->ranges.size()) % scan_msg->ranges.size();
                float range = scan_msg->ranges[index];
                float angle = scan_msg->angle_min + index * scan_msg->angle_increment;
                
                if (range >= min_range_ && range <= max_range_ &&
                    angle >= min_angle_ && angle <= max_angle_)
                {
                    window.push_back(range);
                }
            }
            
            if (!window.empty())
            {
                // 计算中值
                std::nth_element(window.begin(), window.begin() + window.size()/2, window.end());
                filtered_ranges.push_back(window[window.size()/2]);
            }
            else
            {
                // 如果窗口内没有有效值，保留原始值
                filtered_ranges.push_back(scan_msg->ranges[i]);
            }
        }
        
        filtered_scan.ranges = filtered_ranges;
        scan_pub_.publish(filtered_scan);
    }

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher scan_pub_;
    int window_size_;
    double max_range_;
    double min_range_;
    double max_angle_;
    double min_angle_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "median_filter_node");
    MedianFilterNode filter_node;
    ros::spin();
    return 0;
}