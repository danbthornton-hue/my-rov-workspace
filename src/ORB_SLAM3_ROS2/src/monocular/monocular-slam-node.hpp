#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const ImageMsg::SharedPtr msg);
    void PublishMapPoints();

    ORB_SLAM3::System* m_SLAM;
    
    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_publisher;

    cv_bridge::CvImagePtr m_cvImPtr;
    int m_map_skip_count = 0;
};

#endif
