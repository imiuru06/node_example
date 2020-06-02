#include <node_example/rosbag_publisher.h>

namespace node_example
{
  BagPublisher::BagPublisher(ros::NodeHandle nh) : nh_(nh), enable(true)// : nh_(nh), message_("hello"), a_(1), b_(2), enable_(true)
  {
    // Set up a dynamic reconfigure server.
    // Do this before parameter server, else some of the parameter server values can be overwritten.
    //dynamic_reconfigure::Server<node_example::nodeExampleConfig>::CallbackType cb;
    //cb = boost::bind(&BagPublisher::configCallback, this, _1, _2);

    // didn't user gui
    //r_srv_.setCallback(cb);

    // Declare variables that can be modified by launch file or command line.
    double rate = 10.0;//1.0;

    groupCloud.reset(new pcl::PointCloud<PointType>());

    // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
    // of the node can be run simultaneously while using different parameters.
    ros::NodeHandle pnh("~");

    bool enable_ = true;

    pnh.param("a", a_, a_);
    pnh.param("b", b_, b_);
    pnh.param("message", message_, message_);
    pnh.param("rate", rate, rate);
    pnh.param("enable", enable_);

    pointCloudTopic = "/velodyne_points";


    folderPath = "/mnt/e/pointclouds_data/";


    pnh.param("pointCloudTopic", pointCloudTopic);

    pnh.param("folderPath", folderPath);

    // Create a publisher and name the topic.
    if (enable_)
    {
      preparePath();
      start();
    }

    // Create timer.
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &BagPublisher::timerCallback, this);
  }

  void BagPublisher::start()
  {
    //pub_ = nh_.advertise<node_example::NodeExampleData>("example", 10);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointCloudTopic, 1);
  }

  void BagPublisher::stop()
  {
    pub_.shutdown();
  }

  void BagPublisher::timerCallback(const ros::TimerEvent &event __attribute__((unused)))
  {

    if (numFiles==cntFiles)
    {
      enable_ = false;
      return;
    }

    sensor_msgs::PointCloud2 laserCloudMsg;

    // when timerCallback. :
    loadPCD();

    publishData();

  }

  void BagPublisher::loadPCD()
  {
    if(pcl::io::loadPCDFile<PointType> (pcdPaths[cntFiles++], *groupCloud) == -1)
    {
        PCL_ERROR ("Couldnt read file *.pcd \n");
        return ;
    }

/*
    std::cout << "Loaded "
            << groupCloud->width * groupCloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

            for (std::size_t i = 0; i < groupCloud->points.size (); ++i)
                std::cout << " " << groupCloud->points[i].x
                          << " " << groupCloud->points[i].y
                          << " " << groupCloud->points[i].z << std::endl;
                          */
  }

  void BagPublisher::publishData(){
    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(*groupCloud, laserCloudMsg);
    //laserCloudMsg.header.stamp.sec = 1;//cloudHeader.stamp;
    //laserCloudMsg.header.stamp.nsec = 2;//cloudHeader.stamp;
    laserCloudMsg.header.stamp = ros::Time::now();
    laserCloudMsg.header.frame_id = "velodyne";
    pub_.publish(laserCloudMsg);
    ROS_INFO("======%d / %d ========", cntFiles, numFiles);
  }

  void BagPublisher::preparePath()
  {
      numFiles = 0;
      cntFiles = 0;
      pcdPaths.clear();
      //uint32_t idx=0;
      for(auto& p : fs::directory_iterator(folderPath)){
        pcdPaths.push_back(p.path().string());
        //idx++;
      }
      numFiles = pcdPaths.size();
      std::cout << "How many : " << pcdPaths.size() << std::endl;

  }

}

/*
  void BagPublisher::configCallback(node_example::nodeExampleConfig &config, uint32_t level __attribute__((unused)))
  {
    // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
    message_ = config.message;
    a_ = config.a;
    b_ = config.b;

    // Check if we are changing enabled state.
    if (enable_ != config.enable)
    {
      if (config.enable)
      {
        start();
      }
      else
      {
        stop();
      }
    }
    enable_ = config.enable;
  }
}
*/
