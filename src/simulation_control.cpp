#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <pses_simulation/CarModel.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace simulation
{

typedef geometry_msgs::Twist twist_msg;
typedef std_msgs::Int16 int16_msg;
typedef sensor_msgs::Imu imu_msg;
typedef nav_msgs::Odometry odom_msg;

static const double DEFAULT_CAR_WHEELBASE = 0.25;
static const double DEFAULT_LOOP_RATE = 100.0;
static const std::string DEFAULT_ODOM_FRAME_ID = "odom";
static const std::string DEFAULT_ODOM_CHILD_FRAME_ID = "base_footprint";
static const std::string DEFAULT_MOTION_COMMAND_TOPIC = "cmd_vel";
static const std::string DEFAULT_STEERING_COMMAND_TOPIC =
    "/uc_bridge/set_steering_level_msg";
static const std::string DEFAULT_MOTOR_COMMAND_TOPIC =
    "/uc_bridge/set_motor_level_msg";
static const std::string DEFAULT_ODOM_TOPIC = "odom";
static const std::string DEFAULT_IMU_TOPIC = "/uc_bridge/imu";

void fetchWheelBase(ros::NodeHandle* nh, double& wheelBase)
{
  if (nh->hasParam("pses_simulation/wheel_base"))
  {
    nh->getParam("pses_simulation/wheel_base", wheelBase);
  }
  else
  {
    wheelBase = DEFAULT_CAR_WHEELBASE;
  }
}

void fetchLoopRate(ros::NodeHandle* nh, double& loopRate)
{
  if (nh->hasParam("pses_simulation/loop_rate"))
  {
    nh->getParam("pses_simulation/loop_rate", loopRate);
  }
  else
  {
    loopRate = DEFAULT_LOOP_RATE;
  }
}

void fetchOdomFrameID(ros::NodeHandle* nh, std::string& odomFrameID)
{
  if (nh->hasParam("pses_simulation/odom_frame_id"))
  {
    nh->getParam("pses_simulation/odom_frame_id", odomFrameID);
  }
  else
  {
    odomFrameID = DEFAULT_ODOM_FRAME_ID;
  }
}

void fetchOdomChildFrameID(ros::NodeHandle* nh, std::string& odomChildFrameID)
{
  if (nh->hasParam("pses_simulation/odom_child_frame_id"))
  {
    nh->getParam("pses_simulation/odom_child_frame_id", odomChildFrameID);
  }
  else
  {
    odomChildFrameID = DEFAULT_ODOM_CHILD_FRAME_ID;
  }
}

void fetchMotionCommandTopic(ros::NodeHandle* nh,
                             std::string& motionCommandTopic)
{
  if (nh->hasParam("pses_simulation/motion_command_topic"))
  {
    nh->getParam("pses_simulation/motion_command_topic", motionCommandTopic);
  }
  else
  {
    motionCommandTopic = DEFAULT_MOTION_COMMAND_TOPIC;
  }
}

void fetchSteeringCommandTopic(ros::NodeHandle* nh,
                               std::string& steeringCommandTopic)
{
  if (nh->hasParam("pses_simulation/steering_command_topic"))
  {
    nh->getParam("pses_simulation/steering_command_topic",
                 steeringCommandTopic);
  }
  else
  {
    steeringCommandTopic = DEFAULT_STEERING_COMMAND_TOPIC;
  }
}

void fetchMotorCommandTopic(ros::NodeHandle* nh, std::string& motorCommandTopic)
{
  if (nh->hasParam("pses_simulation/motor_command_topic"))
  {
    nh->getParam("pses_simulation/motor_command_topic", motorCommandTopic);
  }
  else
  {
    motorCommandTopic = DEFAULT_MOTOR_COMMAND_TOPIC;
  }
}

void fetchOdomTopic(ros::NodeHandle* nh, std::string& odomTopic)
{
  if (nh->hasParam("pses_simulation/odom_topic"))
  {
    nh->getParam("pses_simulation/odom_topic", odomTopic);
  }
  else
  {
    odomTopic = DEFAULT_ODOM_TOPIC;
  }
}

void fetchImuTopic(ros::NodeHandle* nh, std::string& imuTopic){
  if (nh->hasParam("pses_simulation/imu_topic"))
  {
    nh->getParam("pses_simulation/imu_topic", imuTopic);
  }
  else
  {
    imuTopic = DEFAULT_IMU_TOPIC;
  }
}

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
}

using namespace simulation;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "simulation_control");
  ros::NodeHandle nh;

  double loopRate, wheelBase;
  std::string motionCmdTopic, steeringCmdTopic, motorCmdTopic, odomTopic,
      odomFrameID, odomChildFrameID, imuTopic;
  fetchLoopRate(&nh, loopRate);
  fetchWheelBase(&nh, wheelBase);
  fetchMotionCommandTopic(&nh, motionCmdTopic);
  fetchSteeringCommandTopic(&nh, steeringCmdTopic);
  fetchMotorCommandTopic(&nh, motorCmdTopic);
  fetchOdomTopic(&nh, odomTopic);
  fetchOdomFrameID(&nh, odomFrameID);
  fetchOdomChildFrameID(&nh, odomChildFrameID);
  fetchImuTopic(&nh, imuTopic);

  ros::Time currentTime = ros::Time::now();
  CarModel car(wheelBase, currentTime);

  std::vector<double> simPose;
  twist_msg motionCmd;
  int steeringCmd, motorCmd;
  bool isTwistCmd = true;
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  odom_msg odom;
  imu_msg imu;

  ros::Subscriber motionControl = nh.subscribe<twist_msg>(
      motionCmdTopic, 10,
      boost::bind(motionCommand, _1, &motionCmd, &isTwistCmd));
  ros::Subscriber steeringControl = nh.subscribe<int16_msg>(
      steeringCmdTopic, 1,
      boost::bind(steeringCommand, _1, &steeringCmd, &isTwistCmd));
  ros::Subscriber motorControl = nh.subscribe<int16_msg>(
      motorCmdTopic, 1,
      boost::bind(motorCommand, _1, &motorCmd, &isTwistCmd));
  ros::Publisher odomPub = nh.advertise<odom_msg>(odomTopic, 10);
  ros::Publisher imuPub = nh.advertise<imu_msg>(imuTopic, 10);

  // Loop starts here:
  ros::Rate loop_rate(loopRate);
  while (ros::ok())
  {
    currentTime = ros::Time::now();
    if (isTwistCmd == false)
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

    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = odomFrameID;
    odom_trans.child_frame_id = odomChildFrameID;

    odom_trans.transform.translation.x = simPose[1];
    odom_trans.transform.translation.y = -simPose[0];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    odom.header.stamp = currentTime;
    odom.header.frame_id = odomFrameID;
    // set the position
    odom.pose.pose.position.x = simPose[1];
    odom.pose.pose.position.y = -simPose[0];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // set the velocity
    odom.child_frame_id = odomChildFrameID;
    odom.twist.twist.linear.x = car.getVx();
    odom.twist.twist.linear.y = car.getVy();
    odom.twist.twist.angular.z = car.getAngularVelocity();

    // create imu sensor message
    imu.header.stamp = currentTime;
    imu.header.frame_id = "robot_simulation";
    imu.angular_velocity.z = car.getAngularVelocity();
    imu.linear_acceleration.x = car.getAx();
    imu.linear_acceleration.y = car.getAy();

    // publish the messages
    odomPub.publish(odom);
    imuPub.publish(imu);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
}
