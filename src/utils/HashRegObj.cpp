#include "utils/HashRegObj.h"
#include "utils/Hlp.h"
#include "utils/patchwork/patchworkpp.h"

#include <omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>

// ============================================================
// Helper utilities
// ============================================================

bool sortByVoteNumber(const std::pair<int, int> a, const std::pair<int, int> b)
{
    return a.second > b.second;
}

double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin)
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count() * 1000;
}

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec)
{
    pcl::PointXYZI pi;
    pi.x = vec[0]; pi.y = vec[1]; pi.z = vec[2];
    return pi;
}

Eigen::Vector3d point2vec(const pcl::PointXYZI &pi)
{
    return Eigen::Vector3d(pi.x, pi.y, pi.z);
}

void transform_point_cloud(const Eigen::Matrix4d &T,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    for (auto &pt : cloud->points)
    {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        p = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
        pt.x = p[0]; pt.y = p[1]; pt.z = p[2];
    }
    cloud->width = cloud->size();
    cloud->height = 1;
}

// ============================================================
// Preprocessing: Voxel, SOR, Euclidean Clustering, PCA
// ============================================================

void voxel_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                  double voxel_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(input_cloud);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*output_cloud);
}

void sor_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                int mean_k, double stddev_mul_thresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul_thresh);
    sor.filter(*output_cloud);
}

void euclidean_clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                          std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points,
                          double cluster_tolerance, int min_size, int max_size)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto &ci : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cld(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : ci.indices)
            cld->push_back(cloud->points[idx]);
        cld->width = cld->size();
        cld->height = 1;
        cld->is_dense = true;
        cluster_points.push_back(cld);
    }
}

void pca_trunk_filter(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points,
                      std::vector<Cluster> &trunk_clusters,
                      std::vector<Cluster> &discarded_clusters,
                      double linearity_threshold,
                      double verticality_threshold,
                      double min_height)
{
    for (const auto &cloud_cluster : cluster_points)
    {
        if (cloud_cluster->empty()) continue;

        // Centroid
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        Eigen::Vector3d center = centroid.head<3>();

        // Covariance matrix
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto &pt : cloud_cluster->points)
        {
            Eigen::Vector3d p(pt.x, pt.y, pt.z);
            Eigen::Vector3d diff = p - center;
            cov += diff * diff.transpose();
        }
        cov /= cloud_cluster->size();

        // Eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        Eigen::Vector3d evals = solver.eigenvalues();       // sorted ascending
        Eigen::Matrix3d evecs = solver.eigenvectors();

        // λ1 ≥ λ2 ≥ λ3
        double lambda1 = evals[2];
        double lambda2 = evals[1];
        double lambda3 = evals[0];
        Eigen::Vector3d axis1 = evecs.col(2); // principal axis

        // Linearity
        double linearity = (lambda1 - lambda2) / lambda1;

        // Verticality: principal axis should align with Z
        double verticality = std::abs(axis1.dot(Eigen::Vector3d(0, 0, 1)));

        // Height
        double h_max = -1e10, h_min = 1e10;
        for (const auto &pt : cloud_cluster->points)
        {
            if (pt.z > h_max) h_max = pt.z;
            if (pt.z < h_min) h_min = pt.z;
        }
        double height = h_max - h_min;

        // Build cluster struct
        Cluster cluster_tmp;
        cluster_tmp.center_ = center;
        cluster_tmp.covariance_ = cov;
        cluster_tmp.normal_ = axis1;
        cluster_tmp.eig_value_ = evals;
        cluster_tmp.linearity_ = linearity;
        cluster_tmp.planarity_ = (lambda2 - lambda3) / lambda1;
        cluster_tmp.scatering_ = lambda3 / lambda1;
        cluster_tmp.points_ = *cloud_cluster;
        cluster_tmp.minZ = h_min;
        cluster_tmp.maxZ = h_max;

        // Trunk check
        if (linearity > linearity_threshold &&
            verticality > verticality_threshold &&
            height > min_height)
        {
            // Trunk cluster
            cluster_tmp.p_center_.x = center[0];
            cluster_tmp.p_center_.y = center[1];
            cluster_tmp.p_center_.z = h_min;  // base of trunk
            cluster_tmp.p_center_.normal_x = axis1[0];
            cluster_tmp.p_center_.normal_y = axis1[1];
            cluster_tmp.p_center_.normal_z = axis1[2];
            cluster_tmp.p_center_.intensity = 1; // trunk marker
            cluster_tmp.is_line_ = true;
            trunk_clusters.push_back(cluster_tmp);
        }
        else
        {
            cluster_tmp.p_center_.x = center[0];
            cluster_tmp.p_center_.y = center[1];
            cluster_tmp.p_center_.z = center[2];
            cluster_tmp.p_center_.normal_x = axis1[0];
            cluster_tmp.p_center_.normal_y = axis1[1];
            cluster_tmp.p_center_.normal_z = axis1[2];
            cluster_tmp.p_center_.intensity = 0; // non-trunk
            cluster_tmp.is_line_ = false;
            discarded_clusters.push_back(cluster_tmp);
        }
    }
}

// ============================================================
// ICP registration wrappers
// ============================================================

void small_gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
                             std::pair<Eigen::Vector3d, Eigen::Matrix3d> &refine_transform,
                             const Eigen::Matrix4d &init_T)
{
    std::vector<Eigen::Vector3d> target_points, source_points;
    point_to_vector(source, source_points);
    point_to_vector(target, target_points);

    small_gicp::RegistrationSetting setting;
    setting.num_threads = 4;
    setting.downsampling_resolution = 0.05;
    setting.max_correspondence_distance = 0.8;

    Eigen::Isometry3d T_init;
    T_init.matrix() = init_T;

    small_gicp::RegistrationResult result =
        small_gicp::align(target_points, source_points, T_init, setting);

    if (!result.converged)
    {
        refine_transform.first.setZero();
        refine_transform.second.setIdentity();
        return;
    }

    Eigen::Isometry3d T = result.T_target_source;
    refine_transform.first = T.translation();
    refine_transform.second = T.linear();
}

// ============================================================
// HashRegDescManager methods
// ============================================================
void HashRegDescManager::GenTriDescs(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                                     FrameInfo &curr_frame_info,
                                     int frame_id,
                                     const Eigen::Matrix4d &T_world_kf)
{
    auto t0 = std::chrono::high_resolution_clock::now();

    curr_frame_info.currCenter.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    curr_frame_info.currCenterFix.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    curr_frame_info.currPoints.reset(new pcl::PointCloud<pcl::PointXYZ>);
    curr_frame_info.T_world_kf = T_world_kf;
    curr_frame_info.has_pose = true;
    curr_frame_info.frame_id_ = frame_id;

    obj_clusters.clear();
    discarded_clusters.clear();

    // 1. Voxel filter on PointXYZI (preserve intensity)
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_out(new pcl::PointCloud<pcl::PointXYZI>);
    {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(input_cloud);
        vg.setLeafSize(config_setting_.side_resolution, config_setting_.side_resolution, config_setting_.side_resolution);
        vg.filter(*voxel_out);
    }

    // 2. SOR outlier removal on PointXYZI (preserve intensity)
    pcl::PointCloud<pcl::PointXYZI>::Ptr sor_out(new pcl::PointCloud<pcl::PointXYZI>);
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(voxel_out);
        sor.setMeanK(config_setting_.min_component_size);
        sor.setStddevMulThresh(config_setting_.sor_stddev);
        sor.filter(*sor_out);
    }

    // 3. Ground removal via PatchWork++ with intensity for RNR
    int sor_size = sor_out->size();
    Eigen::MatrixXf sor_eigen(sor_size, 4);
    for (int i = 0; i < sor_size; ++i)
    {
        sor_eigen.row(i) << sor_out->points[i].x, sor_out->points[i].y, sor_out->points[i].z, sor_out->points[i].intensity;
    }

    patchworkpp_.estimateGround(sor_eigen);
    Eigen::MatrixX3f nonground_eigen = patchworkpp_.getNonground();

    pcl::PointCloud<pcl::PointXYZ>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZ>);
    nonground->resize(nonground_eigen.rows());
    for (int i = 0; i < nonground_eigen.rows(); ++i)
    {
        nonground->points[i].x = nonground_eigen(i, 0);
        nonground->points[i].y = nonground_eigen(i, 1);
        nonground->points[i].z = nonground_eigen(i, 2);
    }

    // 4. Euclidean clustering (on non-ground points)
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_points;
    euclidean_clustering(nonground, cluster_points,
                         config_setting_.tolorance,   // cluster tolerance (reused)
                         config_setting_.min_component_size,
                         config_setting_.max_n);

    auto t1 = std::chrono::high_resolution_clock::now();

    // 5. PCA trunk filtering
    pca_trunk_filter(cluster_points, obj_clusters, discarded_clusters,
                     config_setting_.linearityThres,
                     1.0 - config_setting_.upThres,  // convert: upThres was discard threshold
                     config_setting_.clusterHeight);

    // 5b. Update trunk Z to ground level from PatchWork++.
    // Input cloud is already in world frame (subscribed from /cloud_registered),
    // so cluster centers are already in world coordinates — no transform needed.
    for (auto &cluster : obj_clusters)
    {
        if (cluster.is_line_)  // trunk cluster
        {
            double ground_z = patchworkpp_.getGroundZ(cluster.center_[0], cluster.center_[1]);
            if (!std::isnan(ground_z))
            {
                cluster.root = ground_z;
                cluster.p_center_.z = ground_z;  // tree root at ground level (world frame)
            }
        }
        // Sync center_ with p_center_ coordinates (already world frame)
        cluster.center_ = Eigen::Vector3d(
            cluster.p_center_.x, cluster.p_center_.y, cluster.p_center_.z);
        cluster.normal_ = Eigen::Vector3d(
            cluster.p_center_.normal_x, cluster.p_center_.normal_y, cluster.p_center_.normal_z);
    }

    auto t2 = std::chrono::high_resolution_clock::now();

    // 6. Build cluster center point cloud
    for (auto &cluster : obj_clusters)
    {
        curr_frame_info.currCenter->push_back(cluster.p_center_);
        curr_frame_info.currCenterFix->push_back(cluster.p_center_);
        *(curr_frame_info.currPoints) += cluster.points_;
    }

    // 7. Build triangle descriptors
    curr_frame_info.desc_.clear();
    if (curr_frame_info.currCenter->size() > 0)
        build_stdesc(curr_frame_info);

    clusters_vec_.push_back(obj_clusters);
    discarded_clusters_vec_.push_back(discarded_clusters);

    auto t3 = std::chrono::high_resolution_clock::now();

    std::cout << GREEN << "[GenTriDescs] KF#" << curr_frame_info.frame_id_
              << ": input=" << input_cloud->size()
              << " → voxel=" << voxel_out->size()
              << " → sor=" << sor_out->size()
              << " → clusters=" << cluster_points.size()
              << " → trunks=" << obj_clusters.size()
              << " → tridesc=" << curr_frame_info.desc_.size()
              << " | Time: preprocess=" << time_inc(t1, t0) << "ms"
              << ", PCA=" << time_inc(t2, t1) << "ms"
              << ", build_stdesc=" << time_inc(t3, t2) << "ms" << RESET << std::endl;
}

// Add descriptors to hash table
void HashRegDescManager::AddTriDescs(const FrameInfo &curr_frame_info)
{
    std::vector<TriDesc> trids_vec = curr_frame_info.desc_;
    for (const auto &single_std : trids_vec)
    {
        TriDesc_LOC position;
        position.x = (int64_t)(single_std.side_length_[0] + 0.5);
        position.y = (int64_t)(single_std.side_length_[1] + 0.5);
        position.z = (int64_t)(single_std.side_length_[2] + 0.5);
        position.a = (int64_t)(single_std.angle_[0]);
        position.b = (int64_t)(single_std.angle_[1]);
        position.c = (int64_t)(single_std.angle_[2]);

        auto iter = data_base_.find(position);
        if (iter != data_base_.end())
            data_base_[position].push_back(single_std);
        else
        {
            std::vector<TriDesc> descriptor_vec;
            descriptor_vec.push_back(single_std);
            data_base_[position] = descriptor_vec;
        }
    }
    frame_info_map_[curr_frame_info.frame_id_] = curr_frame_info;
}

// Search loop (multiple matches)
void HashRegDescManager::SearchMultiPosition(
    const FrameInfo &curr_frame, std::vector<std::pair<int, double>> &loop_result,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &loop_transform,
    std::vector<std::vector<std::pair<TriDesc, TriDesc>>> &loop_std_pair_vec)
{
    if (curr_frame.desc_.empty())
    {
        std::cout << BOLDRED << "No Triangle Descs!" << RESET << std::endl;
        loop_result.push_back({-1, 0});
        return;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    std::vector<TriMatchList> candidate_matcher_vec;
    candidate_frames_selector(curr_frame, candidate_matcher_vec);
    auto t2 = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < candidate_matcher_vec.size(); ++i)
    {
        double verify_score = -1;
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> relative_pose;
        std::vector<std::pair<TriDesc, TriDesc>> sucess_match_vec;
        int target_id = candidate_matcher_vec[i].match_id_.second;

        std::cout << BOLDCYAN << "[Verify] Candidate: KF " << curr_frame.frame_id_
                  << " -> KF " << target_id << " (" << candidate_matcher_vec[i].match_list_.size()
                  << " tri matches)" << RESET << std::endl;

        candidate_frames_verify(curr_frame, candidate_matcher_vec[i], verify_score,
                                relative_pose, sucess_match_vec, config_setting_);

        if (verify_score > 0)
        {
            std::cout << BOLDGREEN
                      << "\n=============================="
                      << "\n  LOOP DETECTED!"
                      << "\n  KF " << curr_frame.frame_id_
                      << "  <->  KF " << target_id
                      << "\n  score=" << verify_score
                      << "\n  triangles=" << sucess_match_vec.size()
                      << "\n=============================="
                      << RESET << std::endl;
            loop_result.push_back({candidate_matcher_vec[i].match_id_.second, verify_score});
            loop_transform.push_back(relative_pose);
            loop_std_pair_vec.push_back(sucess_match_vec);
        }
    }

    // Sort by score descending
    for (int i = (int)loop_result.size() - 1; i > 0; --i)
    {
        for (int j = 0; j < i; ++j)
        {
            if (loop_result[j].second < loop_result[j + 1].second)
            {
                std::swap(loop_result[j], loop_result[j + 1]);
                std::swap(loop_transform[j], loop_transform[j + 1]);
                std::swap(loop_std_pair_vec[j], loop_std_pair_vec[j + 1]);
            }
        }
    }

    auto t3 = std::chrono::high_resolution_clock::now();
    std::cout << "[Time] candidate selector: " << time_inc(t2, t1)
              << " ms, candidate verify: " << time_inc(t3, t2) << "ms" << std::endl;
}

// Candidate frames selector
void HashRegDescManager::candidate_frames_selector(const FrameInfo &curr_frame,
                                                   std::vector<TriMatchList> &candidate_matcher_vec)
{
    int descNum = curr_frame.desc_.size();

    // ±8 bins search (each bin ≈ 1m since side_length_ is ×10, so ±8m range)
    std::vector<Eigen::Vector3i> voxel_round;
    for (int x = -8; x <= 8; ++x)
        for (int y = -8; y <= 8; ++y)
            for (int z = -8; z <= 8; ++z)
                voxel_round.push_back(Eigen::Vector3i(x, y, z));

    int db_size = data_base_.size();
    int db_entries = 0;
    for (const auto& kv : data_base_) db_entries += kv.second.size();
    std::cout << "[DB] size=" << db_size << " entries=" << db_entries << std::endl;

    std::vector<bool> useful_match(descNum, false);
    std::vector<std::vector<size_t>> useful_match_index(descNum);
    std::vector<std::vector<TriDesc_LOC>> useful_match_position(descNum);

    // Diagnostic counters
    int hash_miss = 0, side_fail = 0, same_frame = 0, vex_fail = 0, center_fail = 0;

    for (int i = 0; i < descNum; ++i)
    {
        TriDesc src_std = curr_frame.desc_[i];
        double dis_threshold = src_std.side_length_.norm() * config_setting_.rough_dis_threshold;

        for (auto &voxel_inc : voxel_round)
        {
            TriDesc_LOC position;
            position.x = (int64_t)(src_std.side_length_[0] + voxel_inc[0]);
            position.y = (int64_t)(src_std.side_length_[1] + voxel_inc[1]);
            position.z = (int64_t)(src_std.side_length_[2] + voxel_inc[2]);

            // Hash bucket center (matches +0.5 rounding in AddTriDescs)
            Eigen::Vector3d voxel_center((double)position.x + 0.5,
                                         (double)position.y + 0.5,
                                         (double)position.z + 0.5);

            if ((src_std.side_length_ - voxel_center).norm() < 1.5)
            {
                auto iter = data_base_.find(position);
                if (iter == data_base_.end()) { hash_miss++; continue; }

                for (size_t j = 0; j < data_base_[position].size(); ++j)
                {
                    double dis = (src_std.side_length_ - data_base_[position][j].side_length_).norm();
                    double diffID = curr_frame.frame_id_ - data_base_[position][j].frame_id_;

                    if (dis >= dis_threshold) { side_fail++; continue; }
                    if (diffID == 0) { same_frame++; continue; }

                    double vex_diff = (src_std.vertex_attached_ -
                                       data_base_[position][j].vertex_attached_).norm();
                    if (vex_diff > config_setting_.vertex_diff_threshold) { vex_fail++; continue; }

                    double center_dist = (src_std.center_ -
                        data_base_[position][j].center_).norm();
                    if (center_dist > 15.0) { center_fail++; continue; }

                    useful_match[i] = true;
                    useful_match_position[i].push_back(position);
                    useful_match_index[i].push_back(j);
                }
            }
        }
    }

    int useful_count = 0;
    for (bool b : useful_match) if (b) useful_count++;
    std::cout << "[Selector] " << descNum << " descs → " << useful_count
              << " useful | hash_miss=" << hash_miss
              << " side_fail=" << side_fail
              << " same_frame=" << same_frame
              << " vex_fail=" << vex_fail
              << " center_fail=" << center_fail
              << " → ";

    // Diagnostic: show side length range of current frame
    if (descNum > 0) {
        double min_s = 1e9, max_s = 0;
        for (int i = 0; i < descNum; ++i) {
            double s = curr_frame.desc_[i].side_length_.norm();
            min_s = std::min(min_s, s);
            max_s = std::max(max_s, s);
        }
        std::cout << "side_norm range=[" << min_s << ", " << max_s << "] | ";
    }

    // Record match index and vote for frames (sparse map, no 80MB stack alloc)
    std::unordered_map<int, int> vote_count;
    std::vector<Eigen::Vector2i> index_recorder;
    std::vector<int> match_index_vec;
    for (size_t i = 0; i < useful_match.size(); ++i)
    {
        if (useful_match[i])
        {
            for (size_t j = 0; j < useful_match_index[i].size(); ++j)
            {
                int fid = data_base_[useful_match_position[i][j]]
                                    [useful_match_index[i][j]].frame_id_;
                vote_count[fid]++;
                index_recorder.push_back(Eigen::Vector2i(i, j));
                match_index_vec.push_back(fid);
            }
        }
    }

    // Select top-K candidates by votes
    for (int cnt = 0; cnt < config_setting_.candidate_num; ++cnt)
    {
        int max_vote = 1;
        int max_vote_index = -1;

        for (const auto &[fid, votes] : vote_count)
        {
            if (votes > max_vote)
            {
                max_vote = votes;
                max_vote_index = fid;
            }
        }

        TriMatchList match_triangle_list;
        if (max_vote_index >= 0 && max_vote >= config_setting_.lGrp_Ele_Min)
        {
            vote_count.erase(max_vote_index);
            match_triangle_list.match_id_.first = curr_frame.frame_id_;
            match_triangle_list.match_id_.second = max_vote_index;

            for (size_t i = 0; i < index_recorder.size(); ++i)
            {
                if (match_index_vec[i] == max_vote_index)
                {
                    std::pair<TriDesc, TriDesc> single_match_pair;
                    single_match_pair.first = curr_frame.desc_[index_recorder[i][0]];
                    single_match_pair.second =
                        data_base_[useful_match_position[index_recorder[i][0]][index_recorder[i][1]]]
                                  [useful_match_index[index_recorder[i][0]][index_recorder[i][1]]];
                    match_triangle_list.match_list_.push_back(single_match_pair);
                }
            }
            candidate_matcher_vec.push_back(match_triangle_list);
        }
        else
        {
            break;
        }
    }

    std::cout << "total_matches=" << index_recorder.size()
              << " → candidates=" << candidate_matcher_vec.size() << std::endl;
}

// Candidate frames verify — uses world-frame descriptor positions
// Voting: check if each matched triangle is consistent with the pose-derived transform
void HashRegDescManager::candidate_frames_verify(const FrameInfo &curr_frame,
    const TriMatchList &candidate_matcher, double &verify_score,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
    std::vector<std::pair<TriDesc, TriDesc>> &sucess_match_vec,
    ConfigSetting &config_setting)
{
    sucess_match_vec.clear();
    int use_size = candidate_matcher.match_list_.size();
    double dis_threshold = config_setting.dist_candi_frames_verify;

    int target_frame_id = candidate_matcher.match_id_.second;
    auto it = frame_info_map_.find(target_frame_id);
    if (it == frame_info_map_.end())
    {
        std::cerr << "[Verify] ERROR: target frame ID " << target_frame_id
                  << " not found in frame_info_map_ (map size: " << frame_info_map_.size() << ")" << std::endl;
        verify_score = -1;
        return;
    }
    auto &target_frame = it->second;
    if (!target_frame.currCenter || target_frame.currCenter->empty())
    {
        std::cerr << "[Verify] ERROR: target frame #" << target_frame_id
                  << " has empty currCenter" << std::endl;
        verify_score = -1;
        return;
    }
    if (!curr_frame.currCenter || curr_frame.currCenter->empty())
    {
        std::cerr << "[Verify] ERROR: current frame has empty currCenter" << std::endl;
        verify_score = -1;
        return;
    }

    // Compute pose-derived transform: T = T_world_target^-1 * T_world_curr
    // This represents the relative pose from current to target in world frame
    Eigen::Matrix4d T_loop = Eigen::Matrix4d::Identity();
    if (curr_frame.has_pose && target_frame.has_pose)
    {
        T_loop = target_frame.T_world_kf.inverse() * curr_frame.T_world_kf;
    }
    else
    {
        // Fallback: use SUD (shouldn't happen with world-frame descriptors)
        std::cerr << "[Verify] WARNING: missing pose info, falling back to SVD" << std::endl;
    }

    Eigen::Matrix3d R_loop = T_loop.block<3, 3>(0, 0);
    Eigen::Vector3d t_loop = T_loop.block<3, 1>(0, 3);

    // Vote: count how many matched triangles have vertices at similar world positions.
    // Since descriptors are built in world frame, both curr and tgt vertices are
    // already in the same coordinate system — compare directly without transform.
    std::vector<int> vote_list(use_size);
    double min_center_dist = 1e9, max_center_dist = 0;
    for (size_t i = 0; i < use_size; ++i)
    {
        auto verify_pair = candidate_matcher.match_list_[i];
        double dis_A = (verify_pair.first.vertex_A_ - verify_pair.second.vertex_A_).norm();
        double dis_B = (verify_pair.first.vertex_B_ - verify_pair.second.vertex_B_).norm();
        double dis_C = (verify_pair.first.vertex_C_ - verify_pair.second.vertex_C_).norm();

        double center_dist = (verify_pair.first.center_ - verify_pair.second.center_).norm();
        min_center_dist = std::min(min_center_dist, center_dist);
        max_center_dist = std::max(max_center_dist, center_dist);

        int vote = 0;
        if (dis_A < dis_threshold) vote++;
        if (dis_B < dis_threshold) vote++;
        if (dis_C < dis_threshold) vote++;
        vote_list[i] = vote;
    }

    // Count total triangles with all 3 vertices consistent
    int max_vote = 0;
    for (size_t i = 0; i < vote_list.size(); ++i)
    {
        if (vote_list[i] == 3) max_vote++;
    }

    std::cout << BOLDYELLOW << "[Verify] max_vote: " << max_vote << " / " << use_size
              << " | center_dist: " << min_center_dist << " ~ " << max_center_dist
              << " (thr=" << dis_threshold << ")" << RESET << std::endl;

    if (max_vote >= 1)
    {
        // Use pose-derived transform for GeoVerify (loop closure relative pose)
        relative_pose = {t_loop, R_loop};

        // Collect all matched pairs with consistent world-frame vertices
        for (size_t j = 0; j < candidate_matcher.match_list_.size(); ++j)
        {
            auto verify_pair = candidate_matcher.match_list_[j];
            double dis_A = (verify_pair.first.vertex_A_ - verify_pair.second.vertex_A_).norm();
            double dis_B = (verify_pair.first.vertex_B_ - verify_pair.second.vertex_B_).norm();
            double dis_C = (verify_pair.first.vertex_C_ - verify_pair.second.vertex_C_).norm();

            if (dis_A < dis_threshold && dis_B < dis_threshold && dis_C < dis_threshold)
            {
                sucess_match_vec.push_back(verify_pair);
            }
        }

        std::cout << "[Verify] sucess_match_vec: " << sucess_match_vec.size() << std::endl;

        verify_score = geometric_verify(
            curr_frame.currCenter,
            target_frame.currCenter, relative_pose);
    }
    else
    {
        verify_score = -1;
    }
}

// Geometric verify — compares cluster centroids in WORLD frame directly.
// Both source and target centroids are already in world coordinates, so we do
// NOT apply the body-to-body transform. Instead we use it as a sanity check:
// count how many source centroids have a close neighbor in the target frame
// when compared in the shared world frame.
double HashRegDescManager::geometric_verify(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform)
{
    // Build KD-tree on target centroids (world frame)
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &pt : target_cloud->points)
    {
        pcl::PointXYZ pi;
        pi.x = pt.x; pi.y = pt.y; pi.z = pt.z;
        input_cloud->push_back(pi);
    }
    kd_tree->setInputCloud(input_cloud);

    int K = 5;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    int useful_match = 0;
    double dis_threshold = config_setting_.dis_geo_verify;

    for (const auto &searchPoint : source_cloud->points)
    {
        pcl::PointXYZ use_search_point;
        use_search_point.x = searchPoint.x;
        use_search_point.y = searchPoint.y;
        use_search_point.z = searchPoint.z;

        if (kd_tree->nearestKSearch(use_search_point, K, pointIdxNKNSearch,
                                    pointNKNSquaredDistance) > 0)
        {
            for (int j = 0; j < (int)pointIdxNKNSearch.size(); ++j)
            {
                int idx = pointIdxNKNSearch[j];
                pcl::PointXYZINormal &nearstPoint = target_cloud->points[idx];
                Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
                Eigen::Vector3d src(searchPoint.x, searchPoint.y, searchPoint.z);

                if ((src - tpi).norm() < dis_threshold)
                {
                    useful_match++;
                    break;
                }
            }
        }
    }

    double final_score = useful_match / (double)source_cloud->size();

    std::cout << BOLDYELLOW << "[GeoVerify] " << useful_match << "/" << source_cloud->size()
              << " (score=" << final_score << ")" << RESET << std::endl;

    return final_score;
}

// Build triangle descriptors from cluster centers
void HashRegDescManager::build_stdesc(FrameInfo &curr_frame_info)
{
    curr_frame_info.desc_.clear();
    double scale = 1.0 / config_setting_.side_resolution;

    int near_num = config_setting_.descriptor_near_num;
    // Clamp near_num to actual point count (KD-tree can't return more points than exist)
    int actual_num = curr_frame_info.currCenter->size();
    if (near_num > actual_num) near_num = actual_num;
    if (near_num < 3) return; // Not enough points to form a triangle

    double max_dis_threshold = config_setting_.descriptor_max_len;
    double min_dis_threshold = config_setting_.descriptor_min_len;
    double len_dis_threshold = config_setting_.descriptor_len_diff;

    std::unordered_map<UNI_VOXEL_LOC, bool> feat_map;
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kd_tree(
        new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
    kd_tree->setInputCloud(curr_frame_info.currCenter);

    std::vector<int> pointIdxNKNSearch(near_num);
    std::vector<float> pointNKNSquaredDistance(near_num);

    for (size_t i = 0; i < curr_frame_info.currCenter->size(); ++i)
    {
        pcl::PointXYZINormal searchPoint = curr_frame_info.currCenter->points[i];
        int found = kd_tree->nearestKSearch(searchPoint, near_num, pointIdxNKNSearch,
                                            pointNKNSquaredDistance);
        if (found < 3) continue;

        for (int m = 1; m < found - 1; ++m)
        {
            for (int n = m + 1; n < found; ++n)
            {
                pcl::PointXYZINormal p1 = searchPoint;
                pcl::PointXYZINormal p2 = curr_frame_info.currCenter->points[pointIdxNKNSearch[m]];
                pcl::PointXYZINormal p3 = curr_frame_info.currCenter->points[pointIdxNKNSearch[n]];

                double a = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
                double b = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2) + pow(p1.z - p3.z, 2));
                double c = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2) + pow(p3.z - p2.z, 2));

                if (a > max_dis_threshold || b > max_dis_threshold || c > max_dis_threshold ||
                    a < min_dis_threshold || b < min_dis_threshold || c < min_dis_threshold)
                    continue;

                if (std::abs(a - b) <= len_dis_threshold ||
                    std::abs(a - c) <= len_dis_threshold ||
                    std::abs(b - c) <= len_dis_threshold)
                    continue;

                // Sort sides: a ≤ b ≤ c
                double temp;
                Eigen::Vector3d A, B, C;
                Eigen::Vector3i l1, l2, l3, l_temp;
                l1 << 1, 2, 0; l2 << 1, 0, 3; l3 << 0, 2, 3;

                if (a > b) { temp=a; a=b; b=temp; l_temp=l1; l1=l2; l2=l_temp; }
                if (b > c) { temp=b; b=c; c=temp; l_temp=l2; l2=l3; l3=l_temp; }
                if (a > b) { temp=a; a=b; b=temp; l_temp=l1; l1=l2; l2=l_temp; }

                pcl::PointXYZ d_p;
                d_p.x = a * 1000; d_p.y = b * 1000; d_p.z = c * 1000;
                UNI_VOXEL_LOC position((int64_t)d_p.x, (int64_t)d_p.y, (int64_t)d_p.z);
                auto iter = feat_map.find(position);

                if (iter == feat_map.end())
                {
                    Eigen::Vector3d normal_1, normal_2, normal_3;
                    Eigen::Vector3d vertex_attached;

                    // Determine vertex A
                    if (l1[0]==l2[0]) { A<<p1.x,p1.y,p1.z; normal_1<<p1.normal_x,p1.normal_y,p1.normal_z; vertex_attached[0]=p1.intensity; }
                    else if (l1[1]==l2[1]) { A<<p2.x,p2.y,p2.z; normal_1<<p2.normal_x,p2.normal_y,p2.normal_z; vertex_attached[0]=p2.intensity; }
                    else { A<<p3.x,p3.y,p3.z; normal_1<<p3.normal_x,p3.normal_y,p3.normal_z; vertex_attached[0]=p3.intensity; }
                    // Vertex B
                    if (l1[0]==l3[0]) { B<<p1.x,p1.y,p1.z; normal_2<<p1.normal_x,p1.normal_y,p1.normal_z; vertex_attached[1]=p1.intensity; }
                    else if (l1[1]==l3[1]) { B<<p2.x,p2.y,p2.z; normal_2<<p2.normal_x,p2.normal_y,p2.normal_z; vertex_attached[1]=p2.intensity; }
                    else { B<<p3.x,p3.y,p3.z; normal_2<<p3.normal_x,p3.normal_y,p3.normal_z; vertex_attached[1]=p3.intensity; }
                    // Vertex C
                    if (l2[0]==l3[0]) { C<<p1.x,p1.y,p1.z; normal_3<<p1.normal_x,p1.normal_y,p1.normal_z; vertex_attached[2]=p1.intensity; }
                    else if (l2[1]==l3[1]) { C<<p2.x,p2.y,p2.z; normal_3<<p2.normal_x,p2.normal_y,p2.normal_z; vertex_attached[2]=p2.intensity; }
                    else { C<<p3.x,p3.y,p3.z; normal_3<<p3.normal_x,p3.normal_y,p3.normal_z; vertex_attached[2]=p3.intensity; }

                    TriDesc single_descriptor;
                    single_descriptor.vertex_A_ = A;
                    single_descriptor.vertex_B_ = B;
                    single_descriptor.vertex_C_ = C;
                    single_descriptor.center_ = (A + B + C) / 3;
                    single_descriptor.vertex_attached_ = vertex_attached;
                    single_descriptor.side_length_ << scale*a, scale*b, scale*c;
                    single_descriptor.angle_[0] = fabs(5 * normal_1.dot(normal_2));
                    single_descriptor.angle_[1] = fabs(5 * normal_1.dot(normal_3));
                    single_descriptor.angle_[2] = fabs(5 * normal_3.dot(normal_2));
                    single_descriptor.frame_id_ = curr_frame_info.frame_id_;
                    curr_frame_info.desc_.push_back(single_descriptor);
                    feat_map[position] = true;
                }
            }
        }
    }
}
