#ifndef ROSBAG_PUBLISHER_H
#define ROSBAG_PUBLISHER_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>


#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Header.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>


/* Is this needed?*/
// Custom message includes. Auto-generated from msg/ directory.
#include <node_example/NodeExampleData.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <node_example/nodeExampleConfig.h>


#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>

#include <algorithm>

//namespace fs = std::experimental::filesystem;

namespace fs = boost::filesystem;

typedef pcl::PointXYZI  PointType;

namespace node_example  // in the future, Rename it.
{

//class ExampleTalker
  class BagPublisher
  {
  public:
    //! Constructor.
    //explicit ExampleTalker(ros::NodeHandle nh);
    explicit BagPublisher(ros::NodeHandle nh);
  private:
    //! Callback function for dynamic reconfigure server.
    void configCallback(node_example::nodeExampleConfig &config, uint32_t level);

    //! Timer callback for publishing message.
    void timerCallback(const ros::TimerEvent &event);

    //! Turn on publisher.
    void start();

    //! Turn off publisher.
    void stop();

    bool loadPCD(size_t i);
    void preparePath();
    void publishData();
    void bagRecord();
    void pcdTransform();

    std::string strSplit(std::string str, char delimiter);
    std::string strTrim(std::string strInput, const char delimiter, bool bFrontBack);
    std::string strReplace(std::string strInput, std::string strFrom, char *charTo);

    //! ROS node handle.
    ros::NodeHandle nh_;

    //! The timer variable used to go to callback function at specified rate.
    ros::Timer timer_;

    //! Message publisher.
    ros::Publisher pub_;

    //! Dynamic reconfigure server.
    dynamic_reconfigure::Server<node_example::nodeExampleConfig> dr_srv_;

    //! The actual message.
    std::string message_;

    std::string pointCloudTopic;
    std::string folderPath;
    std::string deviceName;
    std::string fileType;

    std::string tmpString;

    rosbag::Bag bag;

    std::vector<std::string> pcdPaths;
    std::vector<std::string> pcdFilenames;
    std::vector<std::string> pcdTimestamp;


    uint32_t numFiles;
    uint32_t cntFiles;


    //! The first integer to use in addition.
    int a_;

    //! The second integer to use in addition.
    int b_;

    //! Flag to set whether the node should do any work at all.
    bool enable_;

    pcl::PointCloud<PointType>::Ptr groupCloud;



  };
}

#endif  // ROSBAG_PUBLISHER_H
