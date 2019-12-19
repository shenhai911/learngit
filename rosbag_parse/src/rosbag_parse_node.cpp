#include "rosbag_parse.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_parse");

  RosbagParse node;

  node.run();

  return 0;
}
