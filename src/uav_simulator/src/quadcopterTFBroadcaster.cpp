#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

std::string parent_frame;  // e.g. "map"
std::string child_frame;   // e.g. "uav1/base_link"

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped tf;
  tf.header.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  tf.header.frame_id = parent_frame.empty() ? msg->header.frame_id : parent_frame;
  tf.child_frame_id  = child_frame;

  tf.transform.translation.x = msg->pose.position.x;
  tf.transform.translation.y = msg->pose.position.y;
  tf.transform.translation.z = msg->pose.position.z;
  tf.transform.rotation      = msg->pose.orientation;

  br.sendTransform(tf);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadcopter_tf_broadcaster");
  ros::NodeHandle nh("~");                    // private ns for params

  std::string poseTopic = "/CERLAB/quadcopter/pose";
  if (argc > 1) poseTopic = argv[1];

  nh.param<std::string>("parent_frame", parent_frame, std::string("map"));
  nh.param<std::string>("child_frame",  child_frame,  std::string("uav1/base_link"));

  ros::Subscriber sub = nh.subscribe(poseTopic, 1, poseCallback);
  ros::spin();
  return 0;
}
