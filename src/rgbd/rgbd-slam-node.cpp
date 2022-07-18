#include "rgbd-slam-node.hpp"
#include<opencv2/core/core.hpp>
#include <tf2/convert.h>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM2::System* pSLAM)
:   Node("orbslam"),
    m_SLAM(pSLAM)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/d435i_L/color/image_raw");  ////// camera/rgb
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/d435i_L/depth/image_rect_raw");   ////// camera/depth

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    pose_pub = rclcpp::Node::create_publisher<geometry_msgs::msg::PoseStamped>("orb_pose", 10);
    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat Tcw = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, msgRGB->header.stamp.sec);

    if (Tcw.empty())
      return;

    cv::Mat R_,t_;
    tf2::Transform transform;
    float scale_factor=1.0;
    vector<float> q;
    if (pub_tf || pub_pose) {
        R_ = Tcw.rowRange(0,3).colRange(0,3).t();
        t_ = -R_*Tcw.rowRange(0,3).col(3);
        q = ORB_SLAM2::Converter::toQuaternion(R_);
        transform.setOrigin(tf2::Vector3(t_.at<float>(0, 0)*scale_factor, t_.at<float>(0, 1)*scale_factor, t_.at<float>(0, 2)*scale_factor));
        tf2::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);
        transform.setRotation(tf_quaternion);
    }

    if (pub_tf)
    {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = msgRGB->header.stamp;
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = "base";
      transformStamped.transform.translation.x = t_.at<float>(0, 0)*scale_factor;
      transformStamped.transform.translation.y = t_.at<float>(0, 1)*scale_factor;
      transformStamped.transform.translation.z = t_.at<float>(0, 2)*scale_factor;
      transformStamped.transform.rotation = tf2::toMsg(transform.getRotation());
      br_->sendTransform(transformStamped);
    }

    if (pub_pose)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = msgRGB->header.stamp;
      pose.header.frame_id ="base";
      pose.pose.position.x = t_.at<float>(0, 0)*scale_factor;
      pose.pose.position.y = t_.at<float>(0, 1)*scale_factor;
      pose.pose.position.z = t_.at<float>(0, 2)*scale_factor;
      pose.pose.orientation.x = q[0];
      pose.pose.orientation.y = q[1];
      pose.pose.orientation.z = q[2];
      pose.pose.orientation.w = q[3];
      pose_pub->publish(pose);
    }
}

