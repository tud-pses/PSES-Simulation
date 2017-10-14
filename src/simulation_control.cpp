#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <pses_simulation/CarModel.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

typedef geometry_msgs::Twist twist_msg;
typedef std_msgs::Int16 int16_msg;

void motionCommand(const twist_msg::ConstPtr& cmdIn, twist_msg* cmdOut,
                   bool* isTwistCmd)
{
  *cmdOut = *cmdIn;
  *isTwistCmd = true;
}

void steeringCommand(const int16_msg::ConstPtr& cmdIn, int* cmdOut,
                     bool* isTwistCmd)
{
  *cmdOut = cmdIn->data;
  *isTwistCmd = false;
}

void motorCommand(const int16_msg::ConstPtr& cmdIn, int* cmdOut,
                  bool* isTwistCmd)
{
  *cmdOut = cmdIn->data;
  *isTwistCmd = false;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "simulation");
  ros::NodeHandle nh;
  ros::Time currentTime = ros::Time::now();

  CarModel car(0.25, currentTime);
  std::vector<double> simPose;
  twist_msg motionCmd;
  int steeringCmd, motorCmd;
  bool isTwistCmd = true;
  tf::TransformBroadcaster odom_broadcaster;

  ros::Subscriber motionControl = nh.subscribe<twist_msg>(
      "cmd_vel", 10, boost::bind(motionCommand, _1, &motionCmd, &isTwistCmd));
  ros::Subscriber steeringControl = nh.subscribe<int16_msg>(
      "/uc_bridge/set_steering_level_msg", 1,
      boost::bind(steeringCommand, _1, &steeringCmd, &isTwistCmd));
  ros::Subscriber motorControl = nh.subscribe<int16_msg>(
      "/uc_bridge/set_motor_level_msg", 1,
      boost::bind(motorCommand, _1, &motorCmd, &isTwistCmd));
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("odom", 10);

  // Loop starts here:
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    currentTime = ros::Time::now();
    if (isTwistCmd==false)
    {
      simPose = *car.getUpdate(steeringCmd / 20, motorCmd / 50, currentTime);
    }
    else
    {
      simPose = *car.getUpdateTwist(motionCmd, currentTime);
    }

    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(simPose[2]);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = simPose[1];
    odom_trans.transform.translation.y = -simPose[0];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";
    // set the position
    odom.pose.pose.position.x = simPose[1];
    odom.pose.pose.position.y = -simPose[0];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = car.getVelocity() * std::cos(simPose[2]);
    odom.twist.twist.linear.y = car.getVelocity() * std::sin(simPose[2]);
    odom.twist.twist.angular.z = car.getAngularVelocity();

    // publish the messages
    odomPub.publish(odom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
}
