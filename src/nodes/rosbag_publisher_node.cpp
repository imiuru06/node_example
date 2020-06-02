#include <node_example/rosbag_publisher.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "rosbag_publisher");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  node_example::BagPublisher node(nh);

  /*
  ros::Rate rate(100);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }*/

  ros::spin();
  // Let ROS handle all callbacks.
  //ros::spin();

  return 0;
}  // end main()
