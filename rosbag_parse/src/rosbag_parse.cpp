#include "rosbag_parse.h"
#include <sys/stat.h>
#include <unistd.h>

RosbagParse::RosbagParse()
{
    onInit();
}

RosbagParse::~RosbagParse()
{
    global_pose_txt_.close();
    frame_timestamp_txt_.close();
}

void RosbagParse::onInit() {

    ros::Time::init();
    rate_ = new ros::Rate(10000);
    nh_ = ros::NodeHandle("~");

    /* ros topic */
    point_cloud_topic_ = nh_.param<std::string>("point_cloud_topic", "/center_pandar40p/pandar40p_driver/lidar_points");
    sbg_ins_pose_topic_ = nh_.param<std::string>("sbg_ins_pose_topic", "/sirius/sensor/ins/global_pose");

    save_path_name_ = nh_.param<std::string>("save_path_name", "/home/jwz/my_files/debug_ws/result_data/");
    bag_prefix_ = nh_.param<std::string>("bag_prefix", "2019-12-13-14-13-01");

    common_prefix_ = save_path_name_+bag_prefix_;
    const char* save_path = common_prefix_.c_str();
    if (access(save_path, 0) == -1)
        if (mkdir(save_path, 0744) == -1)
            std::cout << "The data folder create error!" << std::endl << save_path << std::endl;

    pose_folder_ = common_prefix_ + "/GlobalPose";
    const char* save_pose_path = pose_folder_.c_str();
    if (access(save_pose_path, 0) == -1)
        if (mkdir(save_pose_path, 0744) == -1)
            std::cout << "The data folder create error!" << std::endl << save_pose_path << std::endl;

    lidar_point_folder_ = common_prefix_ + "/Lidar40p";
    const char* save_lidar_path = lidar_point_folder_.c_str();
    if (access(save_lidar_path, 0) == -1)
        if (mkdir(save_lidar_path, 0744) == -1)
            std::cout << "The data folder create error!" << std::endl << save_lidar_path << std::endl;

    timestamp_folder_ = common_prefix_ + "/TimeStamp";
    const char* save_times_path = timestamp_folder_.c_str();
    if (access(save_times_path, 0) == -1)
        if (mkdir(save_times_path, 0744) == -1)
            std::cout << "The data folder create error!" << std::endl << save_times_path << std::endl;


    global_pose_txt_.open(pose_folder_+"/global_pose.txt", std::ios::app);

    frame_timestamp_txt_.open(timestamp_folder_+"/frame_timestamp.txt", std::ios::app);


    point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 1, &RosbagParse::pointcloudCallback, this);
    sbg_ins_pose_sub_ = nh_.subscribe(sbg_ins_pose_topic_, 1, &RosbagParse::sbgINSPoseCallback, this);

    frame_num_ = 1;
}

void RosbagParse::run() {

    while (ros::ok()) {

        ros::spinOnce();

        rate_->sleep();
    }
}


void RosbagParse::pointcloudCallback(const sensor_msgs::PointCloud2Ptr &cloud_msg) {

    ROS_INFO("Point_cloud_callback!");

    boost::shared_ptr<pcl::PointCloud<hesai::PointXYZITR> > input_cloud(new pcl::PointCloud<hesai::PointXYZITR>);

    pcl::fromROSMsg(*cloud_msg, *input_cloud);

    ROS_INFO("### std::to_string(static_cast<long int>(input_cloud->header.stamp)) = %f ###\n", ros::Time(cloud_msg->header.stamp).toSec() * 1000);
    std::string file_name = lidar_point_folder_ + "/pandar40p_" + std::to_string(static_cast<long unsigned int>(cloud_msg->header.stamp.toSec() * 1000)) + ".bin";
    std::ofstream output_bin_file(file_name, std::ios::binary);

    int points_count = 0;

    for (size_t i=0; i<input_cloud->size(); i++) {
        hesai::PointXYZITR pt = input_cloud->points[i];

//        ROS_INFO("########### pt.ring = %d ############\n ", pt.ring);
        float temp_point[4] = {0};
        temp_point[0] = pt.ring;
        temp_point[1] = -1*pt.x;
        temp_point[2] = -1*pt.y;
        temp_point[3] = pt.z;
        output_bin_file.write((char*)(&temp_point), sizeof(temp_point));

        points_count++;
    }

    frame_timestamp_txt_ << frame_num_ << " "
                         << static_cast<long unsigned int>(cloud_msg->header.stamp.toSec()*1000) << " "
                         << points_count
                         << std::endl;

    output_bin_file.close();
    frame_num_++;
}


void RosbagParse::sbgINSPoseCallback(const sirius_msgs::Pose& msg) {
    ROS_INFO("SBG_INS_pose_callback!");

//    ros::Time st = ros::Time::now();

    global_pose_txt_ << static_cast<long unsigned int>(msg.header.stamp.toSec()*1000) << " "
                     << msg.x << " "
                     << msg.y << " "
                     << msg.z << " "
                     << msg.yaw << " "
                     << msg.pitch << " "
                     << msg.roll
                     << std::endl;
}

