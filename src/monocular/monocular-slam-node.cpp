#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM2::System* pSLAM)
:   Node("orbslam"), 
    m_SLAM(pSLAM)
{
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/d435i_L/color/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

  pose_pub = rclcpp::Node::create_publisher<geometry_msgs::msg::PoseStamped>("orb_pose", 10);
  br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, msg->header.stamp.sec);

    if (Tcw.empty())
      return;

    cv::Mat R_,t_, t_L_color_to_base, t_world_to_base;  // t_: world -> L_color
    tf2::Transform transform;  // transform : world -> L_color -> base
    float scale_factor=1.0;
    vector<float> q;
    vector<float> q_L_color_to_base;
    q_L_color_to_base = {0.706, -0.338, -0.018, 0.622};
    t_L_color_to_base = cv::Mat(3, 1, CV_32F, {-0.306, 0.138, -0.181});

    tf2::Quaternion tf_quaternion_world_to_base;

    if (pub_tf || pub_pose) {
      R_ = Tcw.rowRange(0,3).colRange(0,3).t();
      t_ = -R_*Tcw.rowRange(0,3).col(3);
      q = ORB_SLAM2::Converter::toQuaternion(R_);
      // transform.setOrigin(tf2::Vector3(t_.at<float>(0, 0)*scale_factor, t_.at<float>(0, 1)*scale_factor, t_.at<float>(0, 2)*scale_factor));
      tf2::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);
      // transform.setRotation(tf_quaternion);

      tf2::Quaternion tf_quaternion_L_color_to_base(q_L_color_to_base[0], q_L_color_to_base[1], q_L_color_to_base[2], q_L_color_to_base[3]);
      tf_quaternion_world_to_base = tf_quaternion * tf_quaternion_L_color_to_base;
      tf_quaternion_world_to_base.normalize();
      t_world_to_base = t_ + R_ * t_L_color_to_base;
      transform.setOrigin(tf2::Vector3(t_world_to_base.at<float>(0, 0)*scale_factor, t_world_to_base.at<float>(0, 1)*scale_factor, t_world_to_base.at<float>(0, 2)*scale_factor));
      transform.setRotation(tf_quaternion_world_to_base);
    }

    if (pub_tf)
    {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = msg->header.stamp;
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = "base";
      transformStamped.transform.translation.x = t_world_to_base.at<float>(0, 0)*scale_factor;
      transformStamped.transform.translation.y = t_world_to_base.at<float>(0, 1)*scale_factor;
      transformStamped.transform.translation.z = t_world_to_base.at<float>(0, 2)*scale_factor;
      transformStamped.transform.rotation = tf2::toMsg(transform.getRotation());
      br_->sendTransform(transformStamped);
    }

    if (pub_pose)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = msg->header.stamp;
      pose.header.frame_id ="base";
      pose.pose.position.x = t_world_to_base.at<float>(0, 0)*scale_factor;
      pose.pose.position.y = t_world_to_base.at<float>(0, 1)*scale_factor;
      pose.pose.position.z = t_world_to_base.at<float>(0, 2)*scale_factor;
      pose.pose.orientation.x = tf_quaternion_world_to_base.getAxis()[0];
      pose.pose.orientation.y = tf_quaternion_world_to_base.getAxis()[1];
      pose.pose.orientation.z = tf_quaternion_world_to_base.getAxis()[2];
      pose.pose.orientation.w = tf_quaternion_world_to_base.getW();
      pose_pub->publish(pose);
    }
}
