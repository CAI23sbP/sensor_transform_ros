#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher unmatchpose_array_pub;
ros::Publisher matchpose_array_pub;
std::string robot_tf_id_; 

void unmatchpointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::string ns = ros::this_node::getNamespace();

    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header.stamp = ros::Time::now();
    pose_array_msg.header.frame_id = robot_tf_id_ + "/base_footprint";


    std::vector<float> ranges;
    for (const pcl::PointXYZ& point : cloud.points)
    {   
        float range = sqrt(point.x * point.x + point.y * point.y);

        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = point.x;
        pose_msg.position.y = point.y;
        pose_msg.position.z = range;
        pose_array_msg.poses.push_back(pose_msg);

    }

    unmatchpose_array_pub.publish(pose_array_msg);
}

void matchpointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // LaserScan 메시지 초기화
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header.stamp = ros::Time::now();
    std::string ns = ros::this_node::getNamespace();
    pose_array_msg.header.frame_id = robot_tf_id_ + "/base_footprint";


    // LaserScan 데이터 계산
    for (const pcl::PointXYZ& point : cloud.points)
    {   
        float range = sqrt(point.x * point.x + point.y * point.y);

        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = point.x;
        pose_msg.position.y = point.y;
        pose_msg.position.z = range;

        // Pose 메시지를 PoseArray에 추가
        pose_array_msg.poses.push_back(pose_msg);
    }
    matchpose_array_pub.publish(pose_array_msg);
} 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_laserscan_node");
    ros::NodeHandle nh;
    nh.param<std::string>("/sensor_transform_node/robot_tf_id", robot_tf_id_, "sim_1_burger");
    ROS_WARN("[Sensor_Transform]: robot_tf_id:%s", robot_tf_id_.c_str());
    ros::Subscriber unpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan_unmatched", 1, unmatchpointcloudCallback);
    ros::Subscriber matchpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan_matched", 1, matchpointcloudCallback);
    unmatchpose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/dynamic_pose_data", 1);
    matchpose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/static_pose_data", 1);


    ros::spin();

    return 0;
}
