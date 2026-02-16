/*
 * map_fusion_node.cpp
 * Subscribes to UGV point cloud and injects into dynamicMap every 30 seconds
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <mutex>

class MapFusionNode {
public:
    MapFusionNode(ros::NodeHandle& nh) : nh_(nh) {
        // Parameters
        nh_.param("fusion_period", fusion_period_, 15.0);
        nh_.param("ugv_cloud_topic", ugv_topic_, std::string("/voxel_grid/output"));
        
        // Subscriber for UGV point cloud
        ugv_sub_ = nh_.subscribe(ugv_topic_, 1, &MapFusionNode::ugvCallback, this);
        
        // Publisher to send cloud for injection
        inject_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_fusion/inject_cloud", 1);
        
        // Timer for periodic fusion
        fusion_timer_ = nh_.createTimer(ros::Duration(fusion_period_), 
                                         &MapFusionNode::fusionCallback, this);
        
        ROS_INFO("[MapFusion] Initialized. Subscribing to: %s", ugv_topic_.c_str());
        ROS_INFO("[MapFusion] Fusion period: %.1f seconds", fusion_period_);
    }

private:
    void ugvCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_ugv_cloud_ = msg;
        has_data_ = true;
    }
    
    void fusionCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        
        if (!has_data_ || !latest_ugv_cloud_) {
            ROS_WARN_THROTTLE(10, "[MapFusion] No UGV data received yet");
            return;
        }
        
        // Publish the cloud for injection
        inject_pub_.publish(latest_ugv_cloud_);
        ROS_INFO("[MapFusion] Published UGV cloud for injection (%u points)", 
                 latest_ugv_cloud_->width * latest_ugv_cloud_->height);
    }
    
    ros::NodeHandle nh_;
    ros::Subscriber ugv_sub_;
    ros::Publisher inject_pub_;
    ros::Timer fusion_timer_;
    
    sensor_msgs::PointCloud2::ConstPtr latest_ugv_cloud_;
    std::mutex cloud_mutex_;
    bool has_data_ = false;
    
    double fusion_period_;
    std::string ugv_topic_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_fusion_node");
    ros::NodeHandle nh("~");
    MapFusionNode node(nh);
    ros::spin();
    return 0;
}