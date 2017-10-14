#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <string>
#include <stdexcept>
#include <sensor_msgs/Range.h>
#include <yaml-cpp/yaml.h>
#include <dynamic_reconfigure/server.h>
#include <pses_simulation/RangeSensorConfig.h>
#include <pses_simulation/RangeSensor.h>

namespace YAML
{
template <> struct convert<geometry_msgs::Pose>
{
  static Node encode(const geometry_msgs::Pose& rhs)
  {
    Node node;
    node.push_back(rhs.position.x);
    node.push_back(rhs.position.y);
    node.push_back(rhs.position.z);
    return node;
  }

  static bool decode(const Node& node, geometry_msgs::Pose& rhs)
  {
    if (!node.IsSequence() || node.size() != 3)
    {
      return false;
    }

    rhs.position.x = node[0].as<double>();
    rhs.position.y = node[1].as<double>();
    rhs.position.z = node[2].as<double>();
    return true;
  }
};
}

namespace laserscan
{

typedef std::shared_ptr<sensor_msgs::LaserScan> scan_msg_ptr;
typedef std::shared_ptr<geometry_msgs::Pose> pose_msg_ptr;

cv::Point setGridPosition(geometry_msgs::Pose& laser,
                          nav_msgs::MapMetaData& mapInfo)
{
  unsigned int grid_x =
      (unsigned int)((laser.position.x - mapInfo.origin.position.x) /
                     mapInfo.resolution);
  unsigned int grid_y =
      (unsigned int)((-laser.position.y - mapInfo.origin.position.y) /
                     mapInfo.resolution);
  return cv::Point(grid_x, grid_y);
}

void getPositionInfo(const std::string& base_frame,
                     const std::string& target_frame,
                     const tf::TransformListener& listener,
                     geometry_msgs::Pose* position, std::vector<double>* rpy)
{
  bool transformReady = false;
  tf::StampedTransform stf;
  tf::Stamped<tf::Pose> tmp;
  geometry_msgs::PoseStamped tmp2;
  tf::Quaternion q;

  try
  {
    transformReady = listener.waitForTransform(
        base_frame, target_frame, ros::Time(0), ros::Duration(0.01));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  if (transformReady)
  {
    try
    {
      listener.lookupTransform(base_frame, target_frame, ros::Time(0), stf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    tmp = tf::Stamped<tf::Pose>(stf, stf.stamp_, base_frame);
    tf::poseStampedTFToMsg(tmp, tmp2);
    *position = tmp2.pose;

    try
    {
      tf::quaternionMsgToTF(tmp2.pose.orientation, q);
      tf::Matrix3x3(q).getRPY((*rpy)[0], (*rpy)[1], (*rpy)[2]);
    }
    catch (std::exception ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}

void configCallback(pses_simulation::RangeSensorConfig& config, uint32_t level,
                    rs::RangeSensor* sensor,
                    pses_simulation::RangeSensorConfig* dynConfig)
{
  sensor->setConfig(config);
  *dynConfig = config;
  ROS_DEBUG("Config was set");
}

static const double DEFAULT_LOOP_RATE = 30.0;
static const std::string DEFAULT_LASER_SCAN_TOPIC = "scan";
static const std::string DEFAULT_MAP_LOCATION =
    ros::package::getPath("pses_simulation") + "/data/map/map.pgm";
static const std::string DEFAULT_MAP_METADATA_LOCATION =
    ros::package::getPath("pses_simulation") + "/data/map/map.yaml";
static const std::string DEFAULT_LASERSCAN_FRAME_ID = "scan";
static const std::string DEFAULT_MAP_FRAME_ID = "map";

void fetchLoopRate(ros::NodeHandle* nh, double& loopRate)
{
  if (nh->hasParam("laser_sim/loop_rate_laserscan"))
  {
    nh->getParam("laser_sim/loop_rate_laserscan", loopRate);
  }
  else
  {
    loopRate = DEFAULT_LOOP_RATE;
  }
}

void fetchLaserScanTopic(ros::NodeHandle* nh, std::string& laserScanTopic)
{
  if (nh->hasParam("laser_sim/laser_scan_topic"))
  {
    nh->getParam("laser_sim/laser_scan_topic", laserScanTopic);
  }
  else
  {
    laserScanTopic = DEFAULT_LASER_SCAN_TOPIC;
  }
}

void fetchMapLocation(ros::NodeHandle* nh, std::string& mapLocation)
{
  if (nh->hasParam("laser_sim/map_location"))
  {
    nh->getParam("laser_sim/map_location", mapLocation);
  }
  else
  {
    mapLocation = DEFAULT_MAP_LOCATION;
  }
}

void fetchMapMetadataLocation(ros::NodeHandle* nh,
                              std::string& mapMetadataLocation)
{
  if (nh->hasParam("laser_sim/map_metadata_location"))
  {
    nh->getParam("laser_sim/map_metadata_location", mapMetadataLocation);
  }
  else
  {
    mapMetadataLocation = DEFAULT_MAP_METADATA_LOCATION;
  }
}

void fetchLaserscanFrameID(ros::NodeHandle* nh, std::string& laserscanFrameID)
{
  if (nh->hasParam("laser_sim/laserscan_frame_id"))
  {
    nh->getParam("laser_sim/laserscan_frame_id", laserscanFrameID);
  }
  else
  {
    laserscanFrameID = DEFAULT_LASERSCAN_FRAME_ID;
  }
}

void fetchMapFrameID(ros::NodeHandle* nh, std::string& mapFrameID)
{
  if (nh->hasParam("laser_sim/map_frame_id"))
  {
    nh->getParam("laser_sim/map_frame_id", mapFrameID);
  }
  else
  {
    mapFrameID = DEFAULT_MAP_FRAME_ID;
  }
}
}

using namespace laserscan;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "simulation_laserscan");
  ros::NodeHandle nh;
  double loopRate;
  std::string laserScanTopic, mapLocation, mapMetadataLocation,
      laserscanFrameID, mapFrameID;

  fetchLoopRate(&nh, loopRate);
  fetchLaserScanTopic(&nh, laserScanTopic);
  fetchMapLocation(&nh, mapLocation);
  fetchMapMetadataLocation(&nh, mapMetadataLocation);
  fetchLaserscanFrameID(&nh, laserscanFrameID);
  fetchMapFrameID(&nh, mapFrameID);

  // create dynamic reconfigure object
  dynamic_reconfigure::Server<pses_simulation::RangeSensorConfig> server;
  dynamic_reconfigure::Server<pses_simulation::RangeSensorConfig>::CallbackType
      f;
  pses_simulation::RangeSensorConfig dynConfig;

  ros::Publisher scanPub = nh.advertise<sensor_msgs::LaserScan>(laserScanTopic, 10);

  sensor_msgs::LaserScan scan;

  // get "god" map meta info
  nav_msgs::MapMetaData mapInfo;
  YAML::Node imgMetaInfo = YAML::LoadFile(mapMetadataLocation);
  double resolution = imgMetaInfo["resolution"].as<double>();
  geometry_msgs::Pose origin = imgMetaInfo["origin"].as<geometry_msgs::Pose>();
  mapInfo.origin = origin;
  mapInfo.resolution = resolution;

  // get "god" map
  cv::Mat map = cv::imread(mapLocation, 1);
  cv::cvtColor(map, map, CV_RGB2GRAY);

  // create sensor object and link with config object
  rs::RangeSensor rs_laser(rs::laser);
  rs_laser.setMap(map);
  rs_laser.setMapMetaData(mapInfo);

  f = boost::bind(&configCallback, _1, _2, &rs_laser, &dynConfig);
  server.setCallback(f);

  // variables needed for transformations
  tf::TransformListener listener;
  geometry_msgs::Pose position;
  std::vector<double> rpy(3, 0.0);
  cv::Point gridPose;

  ros::Time currentTime = ros::Time::now();

  // Loop starts here:
  ros::Rate loop_rate(loopRate);
  while (ros::ok())
  {
    currentTime = ros::Time::now();
    scan.header.stamp = currentTime;
    scan.header.frame_id = laserscanFrameID;
    scan.angle_increment = rs::degToRad(dynConfig.laser_angular_resolution);
    scan.range_min = dynConfig.laser_min_sensor_distance;
    scan.range_max = dynConfig.laser_max_sensor_distance;
    scan.angle_min = rs::degToRad(
        rs::correctYawAngle(0, -dynConfig.laser_field_of_view / 2));
    scan.angle_max =
        rs::degToRad(rs::correctYawAngle(0, dynConfig.laser_field_of_view / 2));

    // get laser range infos
    getPositionInfo(mapFrameID, laserscanFrameID, listener, &position, &rpy);
    gridPose = setGridPosition(position, mapInfo);
    scan.ranges = *rs_laser.getLaserScan(gridPose, rs::radToDeg(rpy[2]));

    // publish sensor msgs
    scanPub.publish(scan);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
}
