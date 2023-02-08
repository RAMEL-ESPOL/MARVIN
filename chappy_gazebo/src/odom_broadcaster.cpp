#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

 geometry_msgs::TransformStamped odom_trans;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);


  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = msg->pose.pose.position.x;
  odom_trans.transform.translation.y = msg->pose.pose.position.y;
  odom_trans.transform.translation.z = msg->pose.pose.position.z;
  odom_trans.transform.rotation = msg->pose.pose.orientation;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_listener");


  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  ros::Rate r(15.0);

    tf::TransformBroadcaster odom_broadcaster;


    //since all odometry is 6DOF we'll need a quaternion created from yaw
geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

//first, we'll publish the transform over tf

odom_trans.header.stamp = ros::Time::now();
odom_trans.header.frame_id = "odom";
odom_trans.child_frame_id = "base_link";

odom_trans.transform.translation.x = 0.0;
odom_trans.transform.translation.y = 0.0;
odom_trans.transform.translation.z = 0.0;
odom_trans.transform.rotation = odom_quat;

//send the transform
odom_broadcaster.sendTransform(odom_trans);

   while(ros::ok()){
     ros::spinOnce();
     odom_broadcaster.sendTransform(odom_trans);

         r.sleep();
   }

  return 0;
}
