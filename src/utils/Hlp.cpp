#include "utils/Hlp.h"

#include <type_traits>
#include <yaml-cpp/yaml.h>

void matrix_to_pair(Eigen::Matrix4f &trans_matrix,
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &trans_pair)
{
    Eigen::Matrix4d trans = trans_matrix.cast<double>();
    trans_pair.first = trans.block<3, 1>(0, 3);
    trans_pair.second = trans.block<3, 3>(0, 0);
}

void point_to_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr &pclPoints,
                     std::vector<Eigen::Vector3d> &vecPoints)
{
    vecPoints.reserve(pclPoints->size());
    for (size_t i = 0; i < pclPoints->size(); ++i)
    {
        vecPoints.emplace_back(pclPoints->points[i].x,
                               pclPoints->points[i].y,
                               pclPoints->points[i].z);
    }
}

// Read parameters from YAML (yaml-cpp)
void ReadParas(const std::string& file_path, ConfigSetting &config_setting)
{
    YAML::Node yaml_node;
    try {
        yaml_node = YAML::LoadFile(file_path);
    } catch (const std::exception& e) {
        std::cerr << RED << "Failed to load YAML: " << file_path
                  << " — " << e.what() << RESET << std::endl;
        return;
    }

    auto load = [&](const std::string& key, auto& val) {
        if (yaml_node[key]) val = yaml_node[key].as<std::remove_reference_t<decltype(val)>>(val);
    };

    // Clustering + PCA
    load("cluster_tolerance", config_setting.tolorance);
    load("cluster_min_size", config_setting.min_component_size);
    load("cluster_max_size", config_setting.max_n);
    load("sor_stddev", config_setting.sor_stddev);
    load("pca_linearity_threshold", config_setting.linearityThres);
    load("pca_min_height", config_setting.clusterHeight);
    // upThres in original code: |axis1·Z| < upThres → discard (0.3 means tilt < ~72°)
    // We use verticality_threshold = 0.866 (tilt < 30°), so upThres = 1 - 0.866 = 0.134
    double vert_thresh;
    if (yaml_node["pca_verticality_threshold"]) {
        vert_thresh = yaml_node["pca_verticality_threshold"].as<double>(0.866);
        config_setting.upThres = 1.0 - vert_thresh;
    }

    // Triangle descriptor
    load("descriptor_near_num", config_setting.descriptor_near_num);
    load("descriptor_min_len", config_setting.descriptor_min_len);
    load("descriptor_max_len", config_setting.descriptor_max_len);
    load("descriptor_len_diff", config_setting.descriptor_len_diff);
    load("side_resolution", config_setting.side_resolution);

    // Loop search
    load("rough_dis_threshold", config_setting.rough_dis_threshold);
    load("candidate_num", config_setting.candidate_num);
    load("min_frame_votes", config_setting.lGrp_Ele_Min);

    // Verification thresholds
    load("icp_threshold", config_setting.icp_threshold);
    load("fitness_threshold", config_setting.fitness_threshold);
    load("dist_candi_frames_verify", config_setting.dist_candi_frames_verify);
    load("dis_geo_verify", config_setting.dis_geo_verify);
    load("vertex_diff_threshold", config_setting.vertex_diff_threshold);

    // centerSelection: 0 = use cluster centroid
    load("centerSelection", config_setting.centerSelection);

    // Trunk cluster merging
    load("trunk_merge_dist", config_setting.trunk_merge_dist);
    load("trunk_merge_z_overlap", config_setting.trunk_merge_z_overlap);
    load("trunk_merge_max_z_gap", config_setting.trunk_merge_max_z_gap);

    // Point cloud preprocessing
    load("accumulation_window_sec", config_setting.accumulation_window_sec);
    load("voxel_size", config_setting.voxel_size);

    // ICP submap
    load("submap_window_size", config_setting.submap_window_size);
    load("submap_voxel_size", config_setting.submap_voxel_size);
    load("icp_corr_distance", config_setting.icp_corr_distance);

    std::cout << GREEN << "[Config] Loaded from: " << file_path << RESET << std::endl;
}
