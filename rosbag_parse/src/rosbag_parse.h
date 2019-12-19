#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <string>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sirius_msgs/Pose.h"

namespace hesai {
  struct PointXYZITR {
      PCL_ADD_POINT4D
      uint8_t intensity;
      double timestamp;
      uint16_t ring;                   ///< laser ring number
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
    hesai::PointXYZITR,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))


class RosbagParse
{
  public:
    RosbagParse();
    ~RosbagParse();

    void onInit();
    void run();

  private:

    /* functions, callback */
    void pointcloudCallback(const sensor_msgs::PointCloud2Ptr& cloud_msg);
    void sbgINSPoseCallback(const sirius_msgs::Pose& msg);

    ros::NodeHandle nh_;
    ros::Rate *rate_;

    /* topic */
    std::string point_cloud_topic_;
    std::string sbg_ins_pose_topic_;


    /* subscribers */
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber sbg_ins_pose_sub_;

    int frame_num_;

    /* files path */
    std::string bin_file_path_;
    std::string frame_timestamp_path_;

    std::string save_path_name_;
    std::string bag_prefix_;

    std::string common_prefix_;
    std::string pose_folder_;
    std::string lidar_point_folder_;
    std::string timestamp_folder_;

    /* ofstream */
    std::ofstream global_pose_txt_;
    std::ofstream frame_timestamp_txt_;
};
