#pragma once
#ifndef HASHREG_OBJ_H
#define HASHREG_OBJ_H

#include "../dst/DST.h"
#include "patchwork/patchworkpp.h"

#include <mutex>
#include <memory>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <small_gicp/registration/registration_helper.hpp>

// Sort by vote number
bool sortByVoteNumber(const std::pair<int, int> a, const std::pair<int, int> b);

// Time increment
double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin);

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec);
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi);

// Convert point cloud to vector
void point_to_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr &pclPoints,
                     std::vector<Eigen::Vector3d> &vecPoints);

// Pair to matrix conversion
void matrix_to_pair(Eigen::Matrix4f &trans_matrix,
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &trans_pair);

// SOR outlier removal
void sor_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                int mean_k, double stddev_mul_thresh);

// Voxel filter
void voxel_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                  double voxel_size);

// Euclidean clustering
void euclidean_clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                          std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points,
                          double cluster_tolerance, int min_size, int max_size);

// PCA filtering for trunk detection
void pca_trunk_filter(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points,
                      std::vector<Cluster> &trunk_clusters,
                      std::vector<Cluster> &discarded_clusters,
                      double linearity_threshold,
                      double verticality_threshold,
                      double min_height);

// Transform point cloud
void transform_point_cloud(const Eigen::Matrix4d &T,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

// small_gicp registration with optional initial transform
void small_gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
                             std::pair<Eigen::Vector3d, Eigen::Matrix3d> &refine_transform,
                             const Eigen::Matrix4d &init_T = Eigen::Matrix4d::Identity());

class HashRegDescManager
{
public:
    ConfigSetting config_setting_;
    // Frame info keyed by sparse keyframe ID (from KeyFrameManager)
    std::unordered_map<int, FrameInfo> frame_info_map_;

    // Hash table: save all descriptors
    std::unordered_map<TriDesc_LOC, std::vector<TriDesc>> data_base_;

    std::vector<Cluster> obj_clusters;
    std::vector<Cluster> discarded_clusters;

    // Store the clusters per frame
    std::vector<std::vector<Cluster>> clusters_vec_;
    std::vector<std::vector<Cluster>> discarded_clusters_vec_;

    // PatchWork++ ground remover
    patchwork::PatchWorkpp patchworkpp_;

public:
    // Generate triangle descriptor from preprocessed point cloud (pure 3D pipeline)
    // T_world_kf: world-to-keyframe-body transform (4x4 matrix)
    // frame_id: external keyframe ID (from KeyFrameManager, sparse odom counter)
    void GenTriDescs(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                     FrameInfo &curr_frame_info,
                     int frame_id,
                     const Eigen::Matrix4d &T_world_kf = Eigen::Matrix4d::Identity());

    // Add triangle descriptors to hash table
    void AddTriDescs(const FrameInfo &curr_frame_info);

    // Search loop for current frame (multiple matches)
    void SearchMultiPosition(
        const FrameInfo &curr_frame, std::vector<std::pair<int, double>> &loop_result,
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &loop_transform,
        std::vector<std::vector<std::pair<TriDesc, TriDesc>>> &loop_std_pair_vec);

    // Select candidate frames from hash matching
    void candidate_frames_selector(const FrameInfo &curr_frame,
                                   std::vector<TriMatchList> &candidate_matcher_vec);

    // Verify candidates using SVD + geometric check
    void candidate_frames_verify(const FrameInfo &curr_frame,
                                 const TriMatchList &candidate_matcher, double &verify_score,
                                 std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
                                 std::vector<std::pair<TriDesc, TriDesc>> &sucess_match_vec,
                                 ConfigSetting &config_setting);

    // Solve transform from triangle vertices using SVD
    void triangle_solver(std::pair<TriDesc, TriDesc> &std_pair,
                         Eigen::Vector3d &t, Eigen::Matrix3d &rot);

    // Geometric verify: distance + normal check
    double geometric_verify(
        const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
        const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);

    // Build triangle descriptors from cluster centers
    void build_stdesc(FrameInfo &curr_frame_info);

    // Constructor
    HashRegDescManager(ConfigSetting &config_setting) : config_setting_(config_setting),
        patchworkpp_(setupPatchWorkParams())
    {
    }

private:
    static patchwork::Params setupPatchWorkParams()
    {
        patchwork::Params pp;
        pp.sensor_height = 1.7;     // typical Mid-360 mounting height
        pp.min_range = 0.5;         // close range for trunk detection
        pp.max_range = 80.0;        // max range
        pp.verbose = false;
        pp.enable_RNR = true;       // enabled: input cloud has intensity
        pp.enable_RVPF = true;      // vertical plane fitting
        pp.enable_TGR = true;       // temporal ground revert
        return pp;
    }
};

#endif // HASHREG_OBJ_H
