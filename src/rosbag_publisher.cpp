#include <node_example/rosbag_publisher.h>

namespace node_example
{
  BagPublisher::BagPublisher(ros::NodeHandle nh) : nh_(nh), enable_(true)// : nh_(nh), message_("hello"), a_(1), b_(2), enable_(true)
  {
    // Set up a dynamic reconfigure server.
    // Do this before parameter server, else some of the parameter server values can be overwritten.
    //dynamic_reconfigure::Server<node_example::nodeExampleConfig>::CallbackType cb;
    //cb = boost::bind(&BagPublisher::configCallback, this, _1, _2);

    // didn't user gui
    //r_srv_.setCallback(cb);

    // Declare variables that can be modified by launch file or command line.
    double rate = 50.0;//1.0;

    groupCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
    // of the node can be run simultaneously while using different parameters.
    ros::NodeHandle pnh("~");

    bool enable_ = true;

//    pnh.param("a", a_, a_);
//    pnh.param("b", b_, b_);
    pnh.param("message", message_, message_);
    pnh.param("rate", rate, rate);
    pnh.param("enable", enable_);

    pointCloudTopic = "/velodyne_points";
    folderPath = "/mnt/e/pointclouds_data/";
    deviceName = "lidar0";
    fileType = ".pcd";

    pnh.param("pointCloudTopic", pointCloudTopic);

    pnh.param("folderPath", folderPath);

    pnh.param("deviceName", deviceName);

    // Create a publisher and name the topic.
    if (enable_)
    {
      preparePath();
      bagRecord();
      // start();
    }

    // Create timer.
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &BagPublisher::timerCallback, this);
  }

  void BagPublisher::start()
  {
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
      return;
    }

    sensor_msgs::PointCloud2 laserCloudMsg;

    // when timerCallback. :
    //loadPCD();

    //publishData();



  }

  bool BagPublisher::loadPCD(size_t i)
  {
    if(pcl::io::loadPCDFile<PointType> (pcdPaths[i], *groupCloud) == -1)
    {
        PCL_ERROR ("Couldnt read file *.pcd \n");
        return true;
    }

    return false;
  }

  void BagPublisher::publishData(){
    sensor_msgs::PointCloud2 laserCloudMsg;

    pcl::toROSMsg(*groupCloud, laserCloudMsg);

    std::string strS = pcdTimestamp[cntFiles].substr(0, 10);
    std::string strNs = pcdTimestamp[cntFiles++].substr(10, 6);
    strNs = strNs + "000";
    uint32_t s = std::atoi(strS.c_str());
    uint32_t ns = std::atoi(strNs.c_str());
    ros::Time velodyneTime(s, ns);

    laserCloudMsg.header.stamp = velodyneTime;
    laserCloudMsg.header.frame_id = "velodyne";
    pub_.publish(laserCloudMsg);
    ROS_INFO("======%d / %d ========", cntFiles-1, numFiles);
  }

  void BagPublisher::preparePath()
  {
      numFiles = 0;
      cntFiles = 0;
      pcdPaths.reserve(100);
      pcdPaths.clear();

//      pcdFilenames.reserve(50);
//      pcdFilenames.clear();
      pcdTimestamp.reserve(50000);
      pcdTimestamp.clear();

      //uint32_t idx=0;
      for(auto& p : fs::directory_iterator(folderPath)){
        pcdPaths.push_back(p.path().string());

        tmpString = p.path().filename().string();
        tmpString = strTrim(tmpString, '_', true);
        tmpString = strTrim(tmpString, '.', false);

        pcdTimestamp.push_back(tmpString);
      }

      sort(pcdPaths.begin(), pcdPaths.end());
      sort(pcdTimestamp.begin(), pcdTimestamp.end());

      if (pcdPaths.size() == pcdTimestamp.size()){
        numFiles = pcdPaths.size();
      }
      else{
        ROS_ERROR ("Something wrong when Preparing Path of PCD \n");
      }

  }


/*
  std::string BagPublisher::strSplit(std::string str, char delimiter)
  {

    //std::vector<std::string> internal;

    std::stringstream ss(str);
    std::string tmp;
    //getline(ss, tmp, delimiter);
    //internal.push_back(tmp);
    //while(){

    //}
    std::getline(ss, tmp, delimiter);
    return tmp;
  }*/

  // bFrontBack is parameter to select which side of string left
  // false is front and true is back

  std::string BagPublisher::strTrim(std::string strInput, const char delimiter, bool bFrontBack = false)
  {

    std::vector<char> strWritable(strInput.begin(), strInput.end());
    strWritable.push_back('\0');

    char* str = &strWritable[0];
    char* token = NULL;

    token = strtok(str, &delimiter);
    std::string strOutput(token);


    if (!bFrontBack){
      return strOutput;
    }
    else{
      //strReplace(strInput, strOutput, " ");
      strInput.erase(0, strOutput.size()+1);
      return strInput;
       //+ std::string(delimiter)
    }
  }

  std::string BagPublisher::strReplace(std::string strInput, std::string strFrom, char *charTo)
  {

    std::string strTo = std::string(charTo);
    int strFromSize = strFrom.size();
    int strToSize = strTo.size();

    std::string tmpStr = strInput;

    tmpStr.replace(tmpStr.find(strFrom), strFromSize, strTo);

    return tmpStr;
  }

  void BagPublisher::bagRecord(){
    bag.open("test2.bag", rosbag::bagmode::Write);

    sensor_msgs::PointCloud2 laserCloudMsg;

    std::string strS;
    std::string strNs;
    strNs = strNs + "000";
    uint32_t s;
    uint32_t ns;

    for(size_t i=0 ; i< numFiles ; i++){

      if(loadPCD(i)){
        continue ;
      }
      pcdTransform();

      pcl::toROSMsg(*groupCloud, laserCloudMsg);

      strS = pcdTimestamp[i].substr(0, 10);
      strNs = pcdTimestamp[i].substr(10, 6);
      strNs = strNs + "000";
      s = std::atoi(strS.c_str());
      ns = std::atoi(strNs.c_str());
      ros::Time velodyneTime(s, ns);
      laserCloudMsg.header.stamp = velodyneTime;
      laserCloudMsg.header.frame_id = "velodyne";

      bag.write("/velodyne_points", velodyneTime, laserCloudMsg);
      //std::cout << "sec : " << std::atoi(pcdTimestamp[i].substr(0, 7)) << std::endl;
      //std::cout << "nsec : " << std::atoi(pcdTimestamp[i].substr(7, 9)) << std::endl;
      //std::cout << "sec : " << pcdTimestamp[i].substr(0, 10) << std::endl;
      //std::cout << "nsec : " << pcdTimestamp[i].substr(10, 6) << std::endl;

      ROS_INFO("====== %d / %d ========", i, numFiles);
    }

    std::cout << "Total number of files : " << pcdTimestamp.size() << std::endl;
    bag.close();


  }

  void BagPublisher::pcdTransform(){
    //pcl::PointCloud<PointType>::Ptr tmpCloud (new pcl::PointCloud<PointType>());
    PointType thisPoint;
    float range;
    size_t cloudSize;

    cloudSize = groupCloud->points.size();

    for(size_t i = 0; i<cloudSize; ++i){
      thisPoint.x = -groupCloud->points[i].y;
      thisPoint.y = -groupCloud->points[i].x;
      thisPoint.z = -groupCloud->points[i].z;

      range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

      thisPoint.intensity = range;// (float)rowIdn + (float)columnIdn / 10000.0;
      groupCloud->points[i] = thisPoint;

    }
    //groupCloud = tmpCloud;

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
