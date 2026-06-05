#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <type_traits>

#include "KeyframeManager.h"
#include "utils/HashRegObj.h"
#include "utils/Hlp.h"
#include "gtsam_opti/gtsamOpti.h"
#include <yaml-cpp/yaml.h>

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

// ============================================================
// Configuration
// ============================================================
struct LoopDetectorConfig {
    std::string config_path;
    std::string cloud_topic;
    std::string odom_topic;

    double keyframe_min_distance = 1.0;
    double keyframe_min_angle = 10.0;
    double keyframe_min_time = 5.0;
    double accumulation_window_sec = 1.0;
    double voxel_size = 0.1;
    int submap_window_size = 5;
    double submap_voxel_size = 0.1;
    double icp_corr_distance = 0.5;
    double icp_threshold = 0.15;
    double fitness_threshold = 0.3;
    double pgo_optimize_interval = 10.0;
};

static void LoadLoopDetectorConfigFromYaml(const std::string &file_path, LoopDetectorConfig &cfg)
{
    if (file_path.empty()) return;

    YAML::Node yaml_node;
    try {
        yaml_node = YAML::LoadFile(file_path);
    } catch (const std::exception &e) {
        std::cerr << RED << "Failed to load YAML: " << file_path
                  << " — " << e.what() << RESET << std::endl;
        return;
    }

    auto load = [&](const std::string &key, auto &val) {
        if (yaml_node[key]) {
            val = yaml_node[key].as<std::remove_reference_t<decltype(val)>>(val);
        }
    };

    load("cloud_topic", cfg.cloud_topic);
    load("odom_topic", cfg.odom_topic);
    load("keyframe_min_distance", cfg.keyframe_min_distance);
    load("keyframe_min_angle", cfg.keyframe_min_angle);
    load("keyframe_min_time", cfg.keyframe_min_time);
    load("accumulation_window_sec", cfg.accumulation_window_sec);
    load("voxel_size", cfg.voxel_size);
    load("submap_window_size", cfg.submap_window_size);
    load("submap_voxel_size", cfg.submap_voxel_size);
    load("icp_corr_distance", cfg.icp_corr_distance);
    load("icp_threshold", cfg.icp_threshold);
    load("fitness_threshold", cfg.fitness_threshold);
    load("pgo_optimize_interval", cfg.pgo_optimize_interval);
}

// ============================================================
// Loop constraint storage
// ============================================================
struct LoopConstraint {
    int kf_curr_id;
    int kf_loop_id;
    Eigen::Matrix4d T_loop; // T_loop_curr = inv(T_W_loop) * T_W_curr
    double fitness;
};

// ============================================================
// LoopDetectorNode class
// ============================================================
class LoopDetectorNode
{
public:
    LoopDetectorNode(ros::NodeHandle &nh);
    ~LoopDetectorNode();
    void run();

private:
    // ROS subscribers
    ros::Subscriber sub_cloud_;
    ros::Subscriber sub_odom_;

    // ROS publishers
    ros::Publisher pub_loop_markers_;
    ros::Publisher pub_corrected_path_;
    ros::Publisher pub_global_map_;
    // Debug publishers
    ros::Publisher pub_trunks_;
    ros::Publisher pub_trunk_centers_;
    ros::Publisher pub_tri_descriptors_;
    ros::Publisher pub_ground_;
    ros::Publisher pub_nonground_;
    ros::Publisher pub_discarded_;

    // Callbacks
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    // Processing threads
    void loopDetectionThread();
    void pgoThread();

    // Core pipeline
    void processKeyFrame(const KeyFrame &kf);
    bool accumulateCloud(const KeyFrame &kf, pcl::PointCloud<pcl::PointXYZI>::Ptr &accumulated);
    bool icpRefinement(const KeyFrame &kf_curr, const KeyFrame &kf_loop,
                       Eigen::Matrix4d &T_icp, double &fitness);
    void publishDebugClouds(const KeyFrame &kf, const FrameInfo &frame_info);

    // PGO
    void runPGO();

    // Helpers
    bool convertCloud(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud,
                      PointCloudXYZI::Ptr &pcl_cloud);

    // Config
    LoopDetectorConfig cfg_;
    ConfigSetting config_setting_;

    // Core components
    std::unique_ptr<KeyframeManager> kf_manager_;
    std::unique_ptr<HashRegDescManager> hash_reg_;

    // Buffer for incoming data
    std::mutex cloud_mutex_;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> cloud_buffer_;
    std::mutex odom_mutex_;
    std::queue<nav_msgs::Odometry::ConstPtr> odom_buffer_;

    std::thread loop_thread_;
    std::thread pgo_thread_;
    std::atomic<bool> running_;

    double last_pgo_time_;
    int keyframe_counter_;

    // Loop constraints
    std::mutex loop_mutex_;
    std::vector<LoopConstraint> loop_constraints_;
};

LoopDetectorNode::LoopDetectorNode(ros::NodeHandle &nh)
    : last_pgo_time_(0), keyframe_counter_(0), running_(true)
{
    // Load ROS params
    nh.param<std::string>("config_path", cfg_.config_path, "");
    nh.param<std::string>("cloud_topic", cfg_.cloud_topic, "/cloud_registered");
    nh.param<std::string>("odom_topic", cfg_.odom_topic, "/Odometry");
    nh.param<double>("keyframe_min_distance", cfg_.keyframe_min_distance, 1.0);
    nh.param<double>("keyframe_min_angle", cfg_.keyframe_min_angle, 10.0);
    nh.param<double>("keyframe_min_time", cfg_.keyframe_min_time, 5.0);
    nh.param<double>("accumulation_window_sec", cfg_.accumulation_window_sec, 1.0);
    nh.param<double>("voxel_size", cfg_.voxel_size, 0.1);
    nh.param<int>("submap_window_size", cfg_.submap_window_size, 5);
    nh.param<double>("submap_voxel_size", cfg_.submap_voxel_size, 0.1);
    nh.param<double>("icp_corr_distance", cfg_.icp_corr_distance, 0.5);
    nh.param<double>("fitness_threshold", cfg_.fitness_threshold, 0.5);
    nh.param<double>("pgo_optimize_interval", cfg_.pgo_optimize_interval, 10.0);

    // Load YAML config
    if (!cfg_.config_path.empty()) {
        LoadLoopDetectorConfigFromYaml(cfg_.config_path, cfg_);
        ReadParas(cfg_.config_path, config_setting_);
    }

    // Initialize core components
    kf_manager_ = std::make_unique<KeyframeManager>(cfg_.keyframe_min_distance,
                                                     cfg_.keyframe_min_angle,
                                                     cfg_.keyframe_min_time);
    hash_reg_ = std::make_unique<HashRegDescManager>(config_setting_);

    // Subscribers
    sub_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>(
        cfg_.cloud_topic, 100, &LoopDetectorNode::cloudCallback, this);
    sub_odom_ = nh.subscribe<nav_msgs::Odometry>(
        cfg_.odom_topic, 100, &LoopDetectorNode::odomCallback, this);

    // Publishers
    pub_loop_markers_ = nh.advertise<visualization_msgs::MarkerArray>("/loop_markers", 10);
    pub_corrected_path_ = nh.advertise<nav_msgs::Path>("/corrected_path", 1);
    pub_global_map_ = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);

    // Debug publishers (for RViz visualization of trunk extraction and descriptors)
    pub_trunks_ = nh.advertise<sensor_msgs::PointCloud2>("/trunk_clouds", 1);
    pub_trunk_centers_ = nh.advertise<sensor_msgs::PointCloud2>("/trunk_centers", 1);
    pub_tri_descriptors_ = nh.advertise<visualization_msgs::MarkerArray>("/tri_descriptors", 1);
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
    pub_nonground_ = nh.advertise<sensor_msgs::PointCloud2>("/nonground_cloud", 1);
    pub_discarded_ = nh.advertise<sensor_msgs::PointCloud2>("/discarded_cloud", 1);

    // Start threads
    loop_thread_ = std::thread(&LoopDetectorNode::loopDetectionThread, this);
    pgo_thread_ = std::thread(&LoopDetectorNode::pgoThread, this);

    ROS_INFO("\033[32m[LoopDetector] Initialized successfully\033[0m");
}

LoopDetectorNode::~LoopDetectorNode()
{
    running_ = false;
    if (loop_thread_.joinable()) loop_thread_.join();
    if (pgo_thread_.joinable()) pgo_thread_.join();
}

void LoopDetectorNode::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_buffer_.push(msg);
}

void LoopDetectorNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer_.push(msg);
}

bool LoopDetectorNode::convertCloud(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud,
                                     PointCloudXYZI::Ptr &pcl_cloud)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*ros_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);
    return !pcl_cloud->empty();
}

void LoopDetectorNode::loopDetectionThread()
{
    ROS_INFO("\033[32m[LoopDetector] Loop detection thread started\033[0m");

    while (running_ && ros::ok())
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_msg;
        nav_msgs::Odometry::ConstPtr odom_msg;

        {
            std::lock_guard<std::mutex> lock_c(cloud_mutex_);
            std::lock_guard<std::mutex> lock_o(odom_mutex_);

            if (cloud_buffer_.empty() || odom_buffer_.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Take the latest odom
            while (odom_buffer_.size() > 1)
                odom_buffer_.pop();
            odom_msg = odom_buffer_.front();
            odom_buffer_.pop();

            // Take cloud closest in time that's before this odom
            double odom_time = odom_msg->header.stamp.toSec();
            while (!cloud_buffer_.empty() &&
                   cloud_buffer_.front()->header.stamp.toSec() <= odom_time)
            {
                cloud_msg = cloud_buffer_.front();
                cloud_buffer_.pop();
            }
            if (!cloud_msg)
                continue;
        }

        // Convert cloud
        PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI);
        if (!convertCloud(cloud_msg, cloud_world))
            continue;

        // Extract odometry
        Eigen::Vector3d position(odom_msg->pose.pose.position.x,
                                  odom_msg->pose.pose.position.y,
                                  odom_msg->pose.pose.position.z);
        Eigen::Quaterniond orientation(odom_msg->pose.pose.orientation.w,
                                        odom_msg->pose.pose.orientation.x,
                                        odom_msg->pose.pose.orientation.y,
                                        odom_msg->pose.pose.orientation.z);

        double timestamp = odom_msg->header.stamp.toSec();

        // Try to save as keyframe
        if (kf_manager_->addKeyFrame(keyframe_counter_++, timestamp,
                                      position, orientation, cloud_world))
        {
            KeyFrame kf;
            kf_manager_->getLastKeyFrame(kf);
            processKeyFrame(kf);
        }

        ros::spinOnce();
    }
}

void LoopDetectorNode::processKeyFrame(const KeyFrame &kf)
{
    ROS_INFO("\033[33m[ProcessKF] KF #%d (cloud size: %zu)\033[0m", kf.id, kf.cloud_world->size());

    // 1. Accumulate point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>);
    if (!accumulateCloud(kf, accumulated))
    {
        ROS_WARN("Failed to accumulate cloud for KF #%d", kf.id);
        return;
    }

    ROS_INFO("[ProcessKF] Accumulated cloud: %zu points", accumulated->size());

    // 2. Build world-to-body transform for this keyframe
    Eigen::Matrix4d T_world_kf = Eigen::Matrix4d::Identity();
    T_world_kf.block<3, 3>(0, 0) = kf.rotation;
    T_world_kf.block<3, 1>(0, 3) = kf.position;

    // 3. Generate triangle descriptors in world frame
    FrameInfo curr_frame_info;
    hash_reg_->GenTriDescs(accumulated, curr_frame_info, kf.id, T_world_kf);

    if (curr_frame_info.desc_.empty())
    {
        ROS_WARN("KF #%d: Degenerate, 0 descriptors", kf.id);
        return;
    }

    // 4. Add to hash table
    hash_reg_->AddTriDescs(curr_frame_info);

    // Publish debug clouds and triangle descriptors (RViz visualization)
    publishDebugClouds(kf, curr_frame_info);

    // 5. Search for loop candidates
    std::vector<std::pair<int, double>> loop_results;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> loop_transforms;
    std::vector<std::vector<std::pair<TriDesc, TriDesc>>> loop_std_pairs;

    hash_reg_->SearchMultiPosition(curr_frame_info, loop_results, loop_transforms, loop_std_pairs);

    ROS_INFO("[ProcessKF] Loop search results: %zu candidates", loop_results.size());

    // 6. Verify top candidates
    for (size_t i = 0; i < std::min((size_t)3, loop_results.size()); ++i)
    {
        int loop_kf_id = loop_results[i].first;
        double score = loop_results[i].second;

        KeyFrame kf_loop;
        if (!kf_manager_->getKeyFrame(loop_kf_id, kf_loop))
        {
            ROS_WARN("[LoopCandidate] Failed to get KF #%d", loop_kf_id);
            continue;
        }

        // Time window filter
        if (kf.timestamp - kf_loop.timestamp < 10.0)
        {
            ROS_INFO("Skipping: time diff too small (%.1fs < 10s)", kf.timestamp - kf_loop.timestamp);
            continue;
        }

        // GeoVerify score pre-filter (avoid ICP on weak candidates)
        if (score < cfg_.icp_threshold)
        {
            ROS_INFO("[LoopCandidate] KF #%d <-> KF #%d: GeoVerify %.3f < %.2f threshold, skipped",
                     kf.id, loop_kf_id, score, cfg_.icp_threshold);
            continue;
        }

        ROS_INFO("\033[32m[LoopCandidate] KF #%d <-> KF #%d (GeoVerify=%.3f, dt=%.1fs)\033[0m",
                 kf.id, loop_kf_id, score, kf.timestamp - kf_loop.timestamp);

        // ICP refines a world-frame correction delta for the current cloud.
        Eigen::Matrix4d T_icp_world_delta;
        double fitness;
        if (icpRefinement(kf, kf_loop, T_icp_world_delta, fitness))
        {
            ROS_INFO("\033[32m[LoopConfirmed] KF #%d <-> KF #%d, fitness=%.3f\033[0m",
                     kf.id, loop_kf_id, fitness);

            Eigen::Matrix4d T_W_curr = Eigen::Matrix4d::Identity();
            T_W_curr.block<3, 3>(0, 0) = kf.rotation;
            T_W_curr.block<3, 1>(0, 3) = kf.position;

            Eigen::Matrix4d T_W_loop = Eigen::Matrix4d::Identity();
            T_W_loop.block<3, 3>(0, 0) = kf_loop.rotation;
            T_W_loop.block<3, 1>(0, 3) = kf_loop.position;

            Eigen::Matrix4d T_W_curr_icp = T_icp_world_delta * T_W_curr;
            Eigen::Matrix4d T_loop_curr = T_W_loop.inverse() * T_W_curr_icp;

            // Store loop constraint
            LoopConstraint lc;
            lc.kf_curr_id = kf.id;
            lc.kf_loop_id = kf_loop.id;
            lc.T_loop = T_loop_curr;
            lc.fitness = fitness;
            {
                std::lock_guard<std::mutex> lock(loop_mutex_);
                loop_constraints_.push_back(lc);
            }

            // Publish visual marker
            visualization_msgs::MarkerArray markers;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "camera_init";
            marker.header.stamp = ros::Time::now();
            marker.ns = "loop_edges";
            marker.id = kf.id;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
            marker.color.a = 1.0;

            geometry_msgs::Point p1, p2;
            p1.x = kf.position.x(); p1.y = kf.position.y(); p1.z = kf.position.z();
            p2.x = kf_loop.position.x(); p2.y = kf_loop.position.y(); p2.z = kf_loop.position.z();
            marker.points.push_back(p1);
            marker.points.push_back(p2);
            markers.markers.push_back(marker);
            pub_loop_markers_.publish(markers);
        }
        else
        {
            ROS_WARN("\033[31m[LoopRejected] KF #%d <-> KF #%d, ICP rejected (fitness below threshold or ICP failed)\033[0m",
                     kf.id, loop_kf_id);
        }
    }
}

bool LoopDetectorNode::accumulateCloud(const KeyFrame &kf,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr &accumulated)
{
    auto all_kfs = kf_manager_->getAllKeyFrames();
    accumulated->clear();

    // All clouds are in world frame — just accumulate and voxel filter
    for (const auto &k : all_kfs)
    {
        if (k.id == kf.id) continue;
        if (kf.timestamp - k.timestamp > cfg_.accumulation_window_sec) continue;

        for (const auto &pt : k.cloud_world->points)
        {
            accumulated->push_back(pt);
        }
    }

    // Current frame cloud
    for (const auto &pt : kf.cloud_world->points)
    {
        accumulated->push_back(pt);
    }

    // Voxel filter
    if (!accumulated->empty())
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(accumulated);
        vg.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size);
        vg.filter(*voxel_out);
        *accumulated = *voxel_out;
    }

    return !accumulated->empty();
}

bool LoopDetectorNode::icpRefinement(const KeyFrame &kf_curr, const KeyFrame &kf_loop,
                                      Eigen::Matrix4d &T_icp, double &fitness)
{
    auto all_kfs = kf_manager_->getAllKeyFrames();

    // Build world-frame submap around loop keyframe (clouds already in world frame)
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &k : all_kfs)
    {
        if (k.id < kf_loop.id - cfg_.submap_window_size ||
            k.id > kf_loop.id + cfg_.submap_window_size)
            continue;
        if (k.id == kf_curr.id) continue;

        for (const auto &pt : k.cloud_world->points)
        {
            pcl::PointXYZ p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            submap->push_back(p);
        }
    }

    if (submap->size() < 100)
    {
        ROS_WARN("Submap too small (%zu points)", submap->size());
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter(submap, submap_filtered, cfg_.submap_voxel_size);

    // Source: current keyframe cloud in world frame (no transform needed)
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &pt : kf_curr.cloud_world->points)
    {
        pcl::PointXYZ p;
        p.x = pt.x; p.y = pt.y; p.z = pt.z;
        source->push_back(p);
    }

    // Source and target points are both in world frame, so ICP estimates a
    // world-frame correction delta for the current cloud.
    Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();

    // ICP refinement with world-frame identity initial guess
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> refine_transform;
    small_gicp_registration(source, submap_filtered, refine_transform, T_init);

    T_icp = Eigen::Matrix4d::Identity();
    T_icp.block<3, 3>(0, 0) = refine_transform.second;
    T_icp.block<3, 1>(0, 3) = refine_transform.first;

    // Fitness score: how many source points align with submap after ICP
    int inliers = 0;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(submap_filtered);
    std::vector<int> indices(1);
    std::vector<float> dists(1);

    for (const auto &pt : source->points)
    {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        p = T_icp.block<3, 3>(0, 0) * p + T_icp.block<3, 1>(0, 3);
        pcl::PointXYZ pp;
        pp.x = p[0]; pp.y = p[1]; pp.z = p[2];

        if (kdtree.nearestKSearch(pp, 1, indices, dists) > 0 && dists[0] < cfg_.icp_corr_distance)
            inliers++;
    }

    fitness = (double)inliers / (double)source->size();
    return fitness >= cfg_.fitness_threshold;
}

// Helper: color a list of clusters into XYZRGB cloud
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorClusterList(
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

// Publish debug clouds (trunks, centers, ground/nonground, discarded, triangle descriptors)
void LoopDetectorNode::publishDebugClouds(const KeyFrame &kf, const FrameInfo &frame_info)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "camera_init";

    // Ground / non-ground
    if (pub_ground_ && hash_reg_->ground_cloud && !hash_reg_->ground_cloud->empty())
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*hash_reg_->ground_cloud, msg);
        msg.header = header;
        pub_ground_.publish(msg);
    }
    if (pub_nonground_ && hash_reg_->nonground_cloud && !hash_reg_->nonground_cloud->empty())
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*hash_reg_->nonground_cloud, msg);
        msg.header = header;
        pub_nonground_.publish(msg);
    }

    // Trunk point clouds (green)
    if (pub_trunks_ && !hash_reg_->obj_clusters.empty())
    {
        pcl::PointCloud<pcl::PointXYZRGB> trunk_cloud = *colorClusterList(hash_reg_->obj_clusters, 36, 180, 94);
        if (!trunk_cloud.empty()) {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(trunk_cloud, msg);
            msg.header = header;
            pub_trunks_.publish(msg);
        }
    }

    // Trunk centers (blue points)
    if (pub_trunk_centers_)
    {
        pcl::PointCloud<pcl::PointXYZINormal> centers;
        for (const auto &cluster : hash_reg_->obj_clusters) {
            centers.push_back(cluster.p_center_);
        }
        if (!centers.empty()) {
            centers.width = centers.size();
            centers.height = 1;
            centers.is_dense = true;
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(centers, msg);
            msg.header = header;
            pub_trunk_centers_.publish(msg);
        }
    }

    // Discarded clusters (gray)
    if (pub_discarded_ && !hash_reg_->discarded_clusters.empty())
    {
        pcl::PointCloud<pcl::PointXYZRGB> discarded_cloud = *colorClusterList(hash_reg_->discarded_clusters, 150, 150, 150);
        if (!discarded_cloud.empty()) {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(discarded_cloud, msg);
            msg.header = header;
            pub_discarded_.publish(msg);
        }
    }

    // Triangle descriptors (LINE_LIST markers)
    if (pub_tri_descriptors_ && !frame_info.desc_.empty())
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
        line_marker.scale.x = 0.1;
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.78;
        line_marker.color.b = 0.05;
        line_marker.color.a = 0.95;

        for (const auto &desc : frame_info.desc_)
        {
            geometry_msgs::Point a, b, c;
            a.x = desc.vertex_A_.x(); a.y = desc.vertex_A_.y(); a.z = desc.vertex_A_.z();
            b.x = desc.vertex_B_.x(); b.y = desc.vertex_B_.y(); b.z = desc.vertex_B_.z();
            c.x = desc.vertex_C_.x(); c.y = desc.vertex_C_.y(); c.z = desc.vertex_C_.z();
            line_marker.points.push_back(a); line_marker.points.push_back(b);
            line_marker.points.push_back(b); line_marker.points.push_back(c);
            line_marker.points.push_back(c); line_marker.points.push_back(a);
        }

        if (!line_marker.points.empty()) {
            markers.markers.push_back(line_marker);
        }
        pub_tri_descriptors_.publish(markers);
    }
}

void LoopDetectorNode::pgoThread()
{
    ROS_INFO("\033[32m[PGO] Pose graph optimization thread started\033[0m");

    while (running_ && ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        double now = ros::Time::now().toSec();
        if (now - last_pgo_time_ < cfg_.pgo_optimize_interval)
            continue;

        runPGO();
        last_pgo_time_ = now;
    }
}

void LoopDetectorNode::runPGO()
{
    std::vector<LoopConstraint> loops;
    {
        std::lock_guard<std::mutex> lock(loop_mutex_);
        if (loop_constraints_.empty()) return;
        loops = loop_constraints_;
    }

    auto all_kfs = kf_manager_->getAllKeyFrames();
    if (all_kfs.size() < 2) return;

    // Build TLS pose list for GTSAM
    std::vector<TLSPos> tls_vec;
    for (const auto &kf : all_kfs)
    {
        TLSPos pos;
        pos.ID = kf.id;
        pos.t = kf.position;
        pos.R = kf.rotation;
        pos.isValued = true;
        tls_vec.push_back(pos);
    }

    // Build candidate info from loop constraints
    std::vector<CandidateInfo> candidates_vec;
    for (size_t i = 0; i < all_kfs.size(); ++i)
    {
        CandidateInfo ci;
        ci.currFrameID = all_kfs[i].id;
        candidates_vec.push_back(ci);
    }

    // Add loop constraints
    for (const auto &lc : loops)
    {
        for (auto &ci : candidates_vec)
        {
            if (ci.currFrameID == lc.kf_curr_id)
            {
                std::pair<int, double> id_score;
                id_score.first = lc.kf_loop_id;
                id_score.second = lc.fitness;

                std::pair<Eigen::Vector3d, Eigen::Matrix3d> rel_pose;
                rel_pose.first = lc.T_loop.block<3, 1>(0, 3);
                rel_pose.second = lc.T_loop.block<3, 3>(0, 0);

                ci.candidateIDScore.push_back(id_score);
                ci.relativePose.push_back(rel_pose);
                break;
            }
        }
    }

    // Build odometry constraints between adjacent keyframes
    std::vector<OdomConstraint> odom_vec;
    for (size_t i = 1; i < all_kfs.size(); ++i)
    {
        OdomConstraint oc;
        oc.fromID = all_kfs[i - 1].id;
        oc.toID = all_kfs[i].id;
        oc.t = Eigen::Vector3d(all_kfs[i].rel_pose_prev(0, 3),
                                all_kfs[i].rel_pose_prev(1, 3),
                                all_kfs[i].rel_pose_prev(2, 3));
        oc.R = all_kfs[i].rel_pose_prev.block<3, 3>(0, 0);
        odom_vec.push_back(oc);
    }

    // Check if there are any loop edges
    bool has_loops = false;
    for (const auto &ci : candidates_vec)
    {
        if (!ci.candidateIDScore.empty()) { has_loops = true; break; }
    }
    if (!has_loops) return;

    // Run GTSAM optimization
    gtsam::Values result;
    std::pair<double, double> var = {1e-4, 1e-2}; // rotation, translation variance
    GTSAMOptimization(tls_vec, candidates_vec, odom_vec, result, var);

    // --- Extract optimized poses and publish corrected path ---
    nav_msgs::Path corrected_path_msg;
    corrected_path_msg.header.frame_id = "camera_init";
    corrected_path_msg.header.stamp = ros::Time::now();

    bool alignment_computed = false;
    Eigen::Matrix4d T_align = Eigen::Matrix4d::Identity(); // aligns optimized frame back to original

    for (const auto &kf : all_kfs)
    {
        if (!result.exists(gtsam::Symbol(kf.id).key()))
        {
            ROS_WARN("[PGO] Keyframe %d not found in optimization result", kf.id);
            continue;
        }

        gtsam::Pose3 opt_pose = result.at<gtsam::Pose3>(gtsam::Symbol(kf.id).key());
        Eigen::Matrix4d T_opt = opt_pose.matrix();

        // Compute alignment from the first keyframe so the path doesn't jump
        if (!alignment_computed)
        {
            Eigen::Matrix4d T_orig = Eigen::Matrix4d::Identity();
            T_orig.block<3, 3>(0, 0) = kf.rotation;
            T_orig.block<3, 1>(0, 3) = kf.position;
            T_align = T_orig * T_opt.inverse();
            alignment_computed = true;
        }

        // Apply alignment transform
        Eigen::Matrix4d T_aligned = T_align * T_opt;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "camera_init";
        pose_stamped.header.stamp = ros::Time(kf.timestamp);
        pose_stamped.pose.position.x = T_aligned(0, 3);
        pose_stamped.pose.position.y = T_aligned(1, 3);
        pose_stamped.pose.position.z = T_aligned(2, 3);

        Eigen::Quaterniond q_aligned(T_aligned.block<3, 3>(0, 0));
        q_aligned.normalize();
        pose_stamped.pose.orientation.w = q_aligned.w();
        pose_stamped.pose.orientation.x = q_aligned.x();
        pose_stamped.pose.orientation.y = q_aligned.y();
        pose_stamped.pose.orientation.z = q_aligned.z();

        corrected_path_msg.poses.push_back(pose_stamped);
    }

    pub_corrected_path_.publish(corrected_path_msg);

    ROS_INFO("\033[32m[PGO] Optimization complete: %zu keyframes, %zu loop constraints, path published (%zu poses)\033[0m",
             tls_vec.size(), loops.size(), corrected_path_msg.poses.size());
}

void LoopDetectorNode::run()
{
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forest_loop_detector");
    ros::NodeHandle nh("~");

    auto node = std::make_shared<LoopDetectorNode>(nh);
    node->run();

    return 0;
}
