#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>

#include "KeyframeManager.h"
#include "utils/HashRegObj.h"
#include "utils/Hlp.h"

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

namespace {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorClusterList(
    const std::vector<Cluster> &clusters, uint8_t r, uint8_t g, uint8_t b)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &cluster : clusters) {
        for (const auto &pt : cluster.points_.points) {
            pcl::PointXYZRGB out;
            out.x = pt.x; out.y = pt.y; out.z = pt.z;
            out.r = r; out.g = g; out.b = b;
            colored->push_back(out);
        }
    }
    colored->width = colored->size();
    colored->height = 1;
    colored->is_dense = true;
    return colored;
}

template <typename PointT>
void publishCloud(const ros::Publisher &pub,
                  const pcl::PointCloud<PointT> &cloud,
                  const std_msgs::Header &header)
{
    if (!pub || cloud.empty()) return;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header = header;
    pub.publish(msg);
}

geometry_msgs::Point toRosPoint(const Eigen::Vector3d &p)
{
    geometry_msgs::Point out;
    out.x = p.x(); out.y = p.y(); out.z = p.z();
    return out;
}

class OnlineTrunkTest
{
public:
    explicit OnlineTrunkTest(ros::NodeHandle &pnh)
        : pnh_(pnh), kf_count_(0)
    {
        pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
        pnh_.param<std::string>("config_path", config_path_, "");
        pnh_.param<int>("print_center_limit", print_center_limit_, 20);
        pnh_.param<int>("max_visualized_descriptors", max_visualized_descriptors_, -1);
        pnh_.param<double>("descriptor_line_width", descriptor_line_width_, 0.5);
        pnh_.param<bool>("publish_debug_clouds", publish_debug_clouds_, true);

        // Keyframe thresholds (same as main pipeline)
        pnh_.param<double>("keyframe_min_distance", kf_min_distance_, 1.0);
        pnh_.param<double>("keyframe_min_angle", kf_min_angle_, 10.0);
        pnh_.param<double>("keyframe_min_time", kf_min_time_, 5.0);

        // Load YAML config
        if (!config_path_.empty()) {
            ReadParas(config_path_, config_);
        }

        // Initialize core components (same as main pipeline)
        kf_manager_ = std::make_unique<KeyframeManager>(kf_min_distance_,
                                                         kf_min_angle_,
                                                         kf_min_time_);
        desc_manager_ = std::make_unique<HashRegDescManager>(config_);

        // Subscribers
        sub_cloud_ = pnh_.subscribe<sensor_msgs::PointCloud2>(
            cloud_topic_, 100, &OnlineTrunkTest::cloudCallback, this);
        sub_odom_ = pnh_.subscribe<nav_msgs::Odometry>(
            odom_topic_, 100, &OnlineTrunkTest::odomCallback, this);

        // Publishers
        pub_ground_ = pnh_.advertise<sensor_msgs::PointCloud2>("ground", 1);
        pub_nonground_ = pnh_.advertise<sensor_msgs::PointCloud2>("nonground", 1);
        pub_clusters_ = pnh_.advertise<sensor_msgs::PointCloud2>("clusters", 1);
        pub_trunks_ = pnh_.advertise<sensor_msgs::PointCloud2>("trunks", 1);
        pub_discarded_ = pnh_.advertise<sensor_msgs::PointCloud2>("discarded", 1);
        pub_trunk_centers_ = pnh_.advertise<sensor_msgs::PointCloud2>("trunk_centers", 1);
        pub_tri_descriptors_ = pnh_.advertise<visualization_msgs::MarkerArray>("tri_descriptors", 1);

        ROS_INFO_STREAM("\033[32m[online_trunk_test] cloud=" << cloud_topic_
                        << ", odom=" << odom_topic_
                        << ", kf_dist=" << kf_min_distance_
                        << ", kf_time=" << kf_min_time_
                        << ", voxel=" << config_.side_resolution
                        << ", cluster_tol=" << config_.tolorance
                        << ", trunk_merge_dist=" << config_.trunk_merge_dist
                        << "\033[0m");
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        cloud_buffer_.push(msg);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_buffer_.push(msg);
    }

public:
    void processFrame()
    {
        // Drain odom buffer to latest
        nav_msgs::Odometry::ConstPtr odom_msg;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (odom_buffer_.empty()) return;
            while (odom_buffer_.size() > 1) odom_buffer_.pop();
            odom_msg = odom_buffer_.front();
            odom_buffer_.pop();
        }

        // Get cloud closest to odom time (before it)
        sensor_msgs::PointCloud2::ConstPtr cloud_msg;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            double odom_time = odom_msg->header.stamp.toSec();
            while (!cloud_buffer_.empty() &&
                   cloud_buffer_.front()->header.stamp.toSec() <= odom_time)
            {
                cloud_msg = cloud_buffer_.front();
                cloud_buffer_.pop();
            }
        }
        if (!cloud_msg) return;

        // Convert cloud
        PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI);
        pcl::fromROSMsg(*cloud_msg, *cloud_world);
        if (cloud_world->empty()) return;

        // Extract odometry
        Eigen::Vector3d position(odom_msg->pose.pose.position.x,
                                  odom_msg->pose.pose.position.y,
                                  odom_msg->pose.pose.position.z);
        Eigen::Quaterniond orientation(odom_msg->pose.pose.orientation.w,
                                        odom_msg->pose.pose.orientation.x,
                                        odom_msg->pose.pose.orientation.y,
                                        odom_msg->pose.pose.orientation.z);
        double timestamp = odom_msg->header.stamp.toSec();

        // Keyframe selection (same as main pipeline)
        int frame_id = kf_count_++;
        if (!kf_manager_->addKeyFrame(frame_id, timestamp,
                                       position, orientation, cloud_world))
        {
            return; // Not a keyframe, skip processing
        }

        // Process keyframe (same as main pipeline: GenTriDescs)
        KeyFrame kf;
        kf_manager_->getLastKeyFrame(kf);

        // Build world-to-body transform
        Eigen::Matrix4d T_world_kf = Eigen::Matrix4d::Identity();
        T_world_kf.block<3, 3>(0, 0) = kf.rotation;
        T_world_kf.block<3, 1>(0, 3) = kf.position;

        // Generate descriptors (full pipeline: voxel -> SOR -> PatchWork++ -> FEC -> PCA -> merge -> build_stdesc)
        FrameInfo frame_info;
        desc_manager_->GenTriDescs(cloud_world, frame_info, kf.id, T_world_kf);

        if (frame_info.desc_.empty()) {
            ROS_WARN("[KF #%d] No descriptors generated", kf.id);
            return;
        }

        // Add to hash table
        desc_manager_->AddTriDescs(frame_info);

        // Publish debug clouds (extracted from HashRegDescManager internals)
        std_msgs::Header out_header = cloud_msg->header;
        if (out_header.frame_id.empty()) out_header.frame_id = "map";

        if (publish_debug_clouds_) {
            // Trunk centers
            pcl::PointCloud<pcl::PointXYZINormal> trunk_centers;
            for (const auto &cluster : desc_manager_->obj_clusters) {
                trunk_centers.push_back(cluster.p_center_);
            }
            trunk_centers.width = trunk_centers.size();
            trunk_centers.height = 1;
            trunk_centers.is_dense = true;
            publishCloud(pub_trunk_centers_, trunk_centers, out_header);

            // Trunk point clouds
            publishCloud(pub_trunks_,
                         *colorClusterList(desc_manager_->obj_clusters, 36, 180, 94),
                         out_header);
            publishCloud(pub_discarded_,
                         *colorClusterList(desc_manager_->discarded_clusters, 150, 150, 150),
                         out_header);

            // Print trunk center info
            for (size_t i = 0; i < desc_manager_->obj_clusters.size(); ++i) {
                if (print_center_limit_ >= 0 &&
                    (int)i >= print_center_limit_) break;
                const auto &c = desc_manager_->obj_clusters[i];
                ROS_INFO_STREAM("[KF #" << kf.id << "] trunk[" << i
                                << "] root=(" << c.p_center_.x << ", "
                                << c.p_center_.y << ", " << c.p_center_.z << ")"
                                << " height=" << (c.maxZ - c.minZ)
                                << " linearity=" << c.linearity_);
            }
        }

        // Publish triangle descriptors
        publishTriDescriptors(frame_info, out_header);

        ROS_INFO_STREAM("\033[32m[KF #" << kf.id << "] trunks="
                        << desc_manager_->obj_clusters.size()
                        << ", tridesc=" << frame_info.desc_.size()
                        << "\033[0m");
    }

    void publishTriDescriptors(const FrameInfo &frame_info,
                               const std_msgs::Header &header)
    {
        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "tri_descriptors";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear_marker);

        visualization_msgs::Marker line_marker;
        line_marker.header = header;
        line_marker.ns = "tri_descriptors";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = descriptor_line_width_;
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.78;
        line_marker.color.b = 0.05;
        line_marker.color.a = 0.95;

        const size_t desc_num = max_visualized_descriptors_ < 0
                                    ? frame_info.desc_.size()
                                    : std::min(frame_info.desc_.size(),
                                               static_cast<size_t>(max_visualized_descriptors_));

        for (size_t i = 0; i < desc_num; ++i) {
            const TriDesc &desc = frame_info.desc_[i];
            geometry_msgs::Point a = toRosPoint(desc.vertex_A_);
            geometry_msgs::Point b = toRosPoint(desc.vertex_B_);
            geometry_msgs::Point c = toRosPoint(desc.vertex_C_);

            line_marker.points.push_back(a);
            line_marker.points.push_back(b);
            line_marker.points.push_back(b);
            line_marker.points.push_back(c);
            line_marker.points.push_back(c);
            line_marker.points.push_back(a);
        }

        if (!line_marker.points.empty()) {
            markers.markers.push_back(line_marker);
        }
        pub_tri_descriptors_.publish(markers);
    }

    ros::NodeHandle pnh_;
    ConfigSetting config_;
    std::unique_ptr<KeyframeManager> kf_manager_;
    std::unique_ptr<HashRegDescManager> desc_manager_;

    std::string cloud_topic_;
    std::string odom_topic_;
    std::string config_path_;
    double kf_min_distance_ = 1.0;
    double kf_min_angle_ = 10.0;
    double kf_min_time_ = 5.0;
    int print_center_limit_ = 20;
    int max_visualized_descriptors_ = 500;
    double descriptor_line_width_ = 0.05;
    int kf_count_ = 0;
    bool publish_debug_clouds_ = true;

    std::mutex cloud_mutex_;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> cloud_buffer_;
    std::mutex odom_mutex_;
    std::queue<nav_msgs::Odometry::ConstPtr> odom_buffer_;

    ros::Subscriber sub_cloud_;
    ros::Subscriber sub_odom_;

    ros::Publisher pub_ground_;
    ros::Publisher pub_nonground_;
    ros::Publisher pub_clusters_;
    ros::Publisher pub_trunks_;
    ros::Publisher pub_discarded_;
    ros::Publisher pub_trunk_centers_;
    ros::Publisher pub_tri_descriptors_;
};

}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_trunk_test");
    ros::NodeHandle pnh("~");
    OnlineTrunkTest node(pnh);

    ros::Rate rate(100);
    while (ros::ok()) {
        node.processFrame();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
