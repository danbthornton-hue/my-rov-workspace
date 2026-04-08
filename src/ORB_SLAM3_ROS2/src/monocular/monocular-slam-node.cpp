#include "monocular-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
: Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;

    // Initialize Publishers
    m_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("orb_slam3/pose", 10);
    m_map_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/map_points", 10);

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 ROS 2 Node Initialized.");
}

MonocularSlamNode::~MonocularSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    try {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Track frame and get Camera Pose (Tcw)
    double timestamp = msg->header.stamp.sec + (msg->header.stamp.nanosec * 1e-9);
Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, timestamp);
    
    // Twc is the Pose of camera in World (inverse of Tcw)
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    Eigen::Vector3f t = Twc.translation();

    // Publish Pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = -t.y();
    pose_msg.pose.position.z = -t.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    m_pose_publisher->publish(pose_msg);

    // Publish Map Points every 5 frames to save CPU
    if (++m_map_skip_count >= 5) {
        PublishMapPoints();
        m_map_skip_count = 0;
    }
}

void MonocularSlamNode::PublishMapPoints()
{
    std::vector<ORB_SLAM3::MapPoint*> all_points = m_SLAM->GetTrackedMapPoints();
    if (all_points.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "world";
    cloud_msg.height = 1;
    cloud_msg.width = all_points.size();
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(all_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (auto pMP : all_points) {
        if (!pMP || pMP->isBad()) continue;
        Eigen::Vector3f pos = pMP->GetWorldPos();
        *iter_x = pos.z();
        *iter_y = pos.x();
        *iter_z = pos.y();
        ++iter_x; ++iter_y; ++iter_z;
    }

    m_map_publisher->publish(cloud_msg);
}
