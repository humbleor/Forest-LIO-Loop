#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <algorithm>
#include <cstring>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include "utils/patchwork/patchworkpp.h"

// ============================================================
// Terminal colors
// ============================================================
#define RESET   "\033[0m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define RED     "\033[31m"
#define CYAN    "\033[36m"
#define BOLDRED     "\033[1m\033[31m"
#define BOLDGREEN   "\033[1m\033[32m"
#define BOLDYELLOW  "\033[1m\033[33m"
#define BOLDCYAN    "\033[1m\033[36m"

// ============================================================
// Timing helper
// ============================================================
struct Timer {
    auto now() { return std::chrono::high_resolution_clock::now(); }
    double ms(std::chrono::high_resolution_clock::time_point t0) {
        return std::chrono::duration_cast<std::chrono::duration<double>>(now() - t0).count() * 1000.0;
    }
};

// ============================================================
// Pipeline stage 1: Voxel downsampling
// ============================================================
static void voxel_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &output,
                         double voxel_size)
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*output);
}

// ============================================================
// Pipeline stage 2: SOR outlier removal
// ============================================================
static void sor_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr &output,
                       int mean_k, double stddev_thresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(input);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_thresh);
    sor.filter(*output);
}

// ============================================================
// Pipeline stage 3: PatchWork++ ground removal
// ============================================================
static void patchwork_ground_remove(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &ground,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &nonground,
    patchwork::PatchWorkpp &patchworkpp)
{
    int n = input->size();
    Eigen::MatrixXf cloud_mat(n, 4);
    for (int i = 0; i < n; ++i) {
        cloud_mat.row(i) << input->points[i].x, input->points[i].y,
                            input->points[i].z, input->points[i].intensity;
    }

    patchworkpp.estimateGround(cloud_mat);
    Eigen::MatrixX3f g = patchworkpp.getGround();
    Eigen::MatrixX3f ng = patchworkpp.getNonground();

    ground->resize(g.rows());
    for (int i = 0; i < g.rows(); ++i)
        ground->points[i] = pcl::PointXYZ(g(i, 0), g(i, 1), g(i, 2));

    nonground->resize(ng.rows());
    for (int i = 0; i < ng.rows(); ++i)
        nonground->points[i] = pcl::PointXYZ(ng(i, 0), ng(i, 1), ng(i, 2));
}

// ============================================================
// Pipeline stage 4: Euclidean clustering
// ============================================================
static void euclidean_clustering(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters,
    double tolerance, int min_size, int max_size)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto &ci : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : ci.indices)
            c->push_back(cloud->points[idx]);
        c->width = c->size();
        c->height = 1;
        c->is_dense = true;
        clusters.push_back(c);
    }
}

struct UnionFind {
    std::vector<int> parent;
    std::vector<int> rank;
    UnionFind(int n) : parent(n), rank(n, 0) {
        for (int i = 0; i < n; i++) parent[i] = i;
    }
    int find(int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    }
    void unite(int x, int y) {
        int rx = find(x), ry = find(y);
        if (rx == ry) return;
        if (rank[rx] < rank[ry]) std::swap(rx, ry);
        parent[ry] = rx;
        if (rank[rx] == rank[ry]) rank[rx]++;
    }
};

struct PointIndex_NumberTag {
    int nPointIndex = 0;
    int nNumberTag = 0;
};

// compare the nuber tag of two points
bool NumberTag(const PointIndex_NumberTag& p0, const PointIndex_NumberTag& p1)
{
    return p0.nNumberTag < p1.nNumberTag;
}

std::vector<pcl::PointIndices> FEC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int min_component_size, double tolerance, int max_n) {

    unsigned long i, j;
    if (cloud->size() < min_component_size)
    {
        PCL_ERROR("Could not find any cluster");
        return {};
    }
    // KD treee, and the input data
    pcl::KdTreeFLANN<pcl::PointXYZ> cloud_kdtreeflann;
    cloud_kdtreeflann.setInputCloud(cloud);
    // the marked records
    int cloud_size = cloud->size();
    std::vector<int> marked_indices;
    marked_indices.resize(cloud_size);

    // set to zero
    memset(marked_indices.data(), 0, sizeof(int) * cloud_size);
    std::vector<int> pointIdx;
    std::vector<float> pointquaredDistance;

    UnionFind uf(cloud_size);
    int tag_num = 1;

    for (i = 0; i < cloud_size; i++)
    {
        // Clustering process
        if (marked_indices[i] == 0) // reset to initial value if this point has not been manipulated
        {
            pointIdx.clear();
            pointquaredDistance.clear();
            cloud_kdtreeflann.radiusSearch(cloud->points[i], tolerance, pointIdx, pointquaredDistance, max_n);
            /**
            * All neighbors closest to a specified point with a query within a given radius
            * para.tolerance is the radius of the sphere that surrounds all neighbors
            * pointIdx is the resulting index of neighboring points
            * pointquaredDistance is the final square distance to adjacent points
            * pointIdx.size() is the maximum number of neighbors returned by limit
            */
            int min_tag_num = tag_num;
            for (j = 0; j < pointIdx.size(); j++)
            {
                /**
                 * find the minimum label value contained in the field points, and tag it to this cluster label.
                 */
                if ((marked_indices[pointIdx[j]] > 0) && (marked_indices[pointIdx[j]] < min_tag_num))
                {
                    min_tag_num = marked_indices[pointIdx[j]];
                }
            }
            for (j = 0; j < pointIdx.size(); j++)
            {
                int p = pointIdx[j];
                // mark current point with the minimum tag
                if (marked_indices[p] == 0) marked_indices[p] = tag_num;
                // union: merge this point into the cluster with min_tag_num
                // find a representative point already carrying min_tag_num
                for (unsigned long k = 0; k < pointIdx.size(); k++) {
                    if (marked_indices[pointIdx[k]] == min_tag_num) {
                        uf.unite(p, pointIdx[k]);
                        break;
                    }
                }
            }
            marked_indices[i] = min_tag_num;
            tag_num++;
        }
    }

    // flatten union-find: each point gets its root as the final label
    for (i = 0; i < cloud_size; i++) {
        if (marked_indices[i] > 0) {
            marked_indices[i] = uf.find(i);
        }
    }

    std::vector<PointIndex_NumberTag> indices_tags;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    indices_tags.resize(cloud_size);

    PointIndex_NumberTag temp_index_tag;


    for (i = 0; i < cloud_size; i++)
    {
        /**
        * Put each point index and the corresponding tag value into the indices_tags
        */
        temp_index_tag.nPointIndex = i;
        temp_index_tag.nNumberTag = marked_indices[i];

        indices_tags[i] = temp_index_tag;
    }
    // sort the indices tag by NUM of tags
    sort(indices_tags.begin(), indices_tags.end(), NumberTag);

    unsigned long begin_index = 0;
    for (i = 0; i < indices_tags.size(); i++)
    {
        // Relabel each cluster
        if (indices_tags[i].nNumberTag != indices_tags[begin_index].nNumberTag)
        {
            if ((i - begin_index) >= min_component_size)
            {
                unsigned long m = 0;
                inliers->indices.resize(i - begin_index);
                for (j = begin_index; j < i; j++)
                    inliers->indices[m++] = indices_tags[j].nPointIndex;
                cluster_indices.push_back(*inliers);
            }
            begin_index = i;
        }
    }
    // the last cluster (determine whether is a inlier)
    if ((i - begin_index) >= min_component_size)
    {
        unsigned long m = 0;
        inliers->indices.resize(i - begin_index);
        for (j = begin_index; j < i; j++)
        {
            inliers->indices[m++] = indices_tags[j].nPointIndex;
        }
        cluster_indices.push_back(*inliers);
    }
    return cluster_indices;

}

// ============================================================
// Cluster struct (mirrors DST.h)
// ============================================================
struct ClusterInfo {
    pcl::PointCloud<pcl::PointXYZ> points_;
    Eigen::Vector3d center_;
    Eigen::Vector3d normal_;
    Eigen::Matrix3d covariance_;
    Eigen::Vector3d eig_value_;
    double minZ = 0, maxZ = 0;
    double linearity_ = 0, planarity_ = 0, scattering_ = 0;
    bool is_line_ = false;
};

// ============================================================
// Pipeline stage 5: PCA trunk filtering
// ============================================================
static void pca_trunk_filter(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points,
    std::vector<ClusterInfo> &trunks,
    std::vector<ClusterInfo> &discarded,
    double linearity_threshold,
    double verticality_threshold,
    double min_height)
{
    for (const auto &cloud_cluster : cluster_points) {
        if (cloud_cluster->empty()) continue;

        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        Eigen::Vector3d center = centroid.head<3>();

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto &pt : cloud_cluster->points) {
            Eigen::Vector3d p(pt.x, pt.y, pt.z);
            Eigen::Vector3d diff = p - center;
            cov += diff * diff.transpose();
        }
        cov /= cloud_cluster->size();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        Eigen::Vector3d evals = solver.eigenvalues();
        Eigen::Matrix3d evecs = solver.eigenvectors();

        double lambda1 = evals[2], lambda2 = evals[1], lambda3 = evals[0];
        Eigen::Vector3d axis1 = evecs.col(2);

        double linearity = (lambda1 - lambda2) / lambda1;
        double verticality = std::abs(axis1.dot(Eigen::Vector3d(0, 0, 1)));

        double h_max = -1e10, h_min = 1e10;
        for (const auto &pt : cloud_cluster->points) {
            if (pt.z > h_max) h_max = pt.z;
            if (pt.z < h_min) h_min = pt.z;
        }
        double height = h_max - h_min;

        ClusterInfo ci;
        ci.points_ = *cloud_cluster;
        ci.center_ = center;
        ci.covariance_ = cov;
        ci.normal_ = axis1;
        ci.eig_value_ = evals;
        ci.linearity_ = linearity;
        ci.planarity_ = (lambda2 - lambda3) / lambda1;
        ci.scattering_ = lambda3 / lambda1;
        ci.minZ = h_min;
        ci.maxZ = h_max;

        if (linearity > linearity_threshold &&
            verticality > verticality_threshold &&
            height > min_height) {
            ci.is_line_ = true;
            trunks.push_back(ci);
        } else {
            ci.is_line_ = false;
            discarded.push_back(ci);
        }
    }
}

// ============================================================
// HSL -> RGB helper for cluster coloring
// ============================================================
static uint32_t hsl_to_rgb(double h, double s, double l)
{
    h = fmod(h / 360.0, 1.0);
    double r, g, b;
    if (s == 0) { r = g = b = l; }
    else {
        auto hue2rgb = [](double p, double q, double t) {
            if (t < 0) t += 1;
            if (t > 1) t -= 1;
            if (t < 1.0/6.0) return p + (q - p) * 6.0 * t;
            if (t < 1.0/2.0) return q;
            if (t < 2.0/3.0) return p + (q - p) * (2.0/3.0 - t) * 6.0;
            return p;
        };
        double q = l < 0.5 ? l * (1 + s) : l + s - l * s;
        double p = 2 * l - q;
        r = hue2rgb(p, q, h + 1.0/3.0);
        g = hue2rgb(p, q, h);
        b = hue2rgb(p, q, h - 1.0/3.0);
    }
    uint8_t ri = (uint8_t)(r * 255);
    uint8_t gi = (uint8_t)(g * 255);
    uint8_t bi = (uint8_t)(b * 255);
    return ((uint32_t)ri << 16) | ((uint32_t)gi << 8) | (uint32_t)bi;
}

// ============================================================
// Print separator
// ============================================================
static void print_sep() {
    std::cout << "------------------------------------------------------------" << std::endl;
}

// ============================================================
// Main
// ============================================================
int main(int argc, char **argv)
{
    Timer timer;

    std::cout << BOLDGREEN << "============================================================" << std::endl;
    std::cout << "  Forest-LIO-Loop: Preprocessing Pipeline Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    // --- Parse input PCD path ---
    std::string input_path;
    if (argc > 1) {
        input_path = argv[1];
    } else {
        std::cout << YELLOW << "Usage: ./pipeline_test <input.pcd>" << std::endl;
        std::cout << YELLOW << "  No input file specified, looking for test_data/sample.pcd..." << RESET << std::endl;
        input_path = "test_data/sample.pcd";
    }

    // --- Load input cloud ---
    auto t_start = timer.now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_path, *cloud_in) < 0) {
        std::cerr << BOLDRED << "[ERROR] Failed to load: " << input_path << RESET << std::endl;
        return 1;
    }
    std::cout << CYAN << "[Input]  Loaded " << cloud_in->size() << " points from " << input_path
              << " (" << std::fixed << std::setprecision(1) << timer.ms(t_start) << "ms)" << RESET << std::endl;

    // --- Create output directory ---
    std::string out_dir = "output";
    std::filesystem::create_directories(out_dir);

    // ============================================================
    // Stage 1: Voxel downsampling
    // ============================================================
    print_sep();
    auto t1 = timer.now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    double voxel_size = 0.1;
    voxel_filter(cloud_in, cloud_voxel, voxel_size);
    std::cout << GREEN << "[Stage 1] Voxel Filter (leaf=" << voxel_size << "m): "
              << cloud_in->size() << " -> " << cloud_voxel->size() << " points"
              << " (" << std::fixed << std::setprecision(1) << timer.ms(t1) << "ms)" << RESET << std::endl;
    pcl::io::savePCDFileBinary(out_dir + "/01_voxel.pcd", *cloud_voxel);

    // ============================================================
    // Stage 2: SOR outlier removal
    // ============================================================
    auto t2 = timer.now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZI>);
    int sor_mean_k = 50;
    double sor_stddev = 1.0;
    sor_filter(cloud_voxel, cloud_sor, sor_mean_k, sor_stddev);
    std::cout << GREEN << "[Stage 2] SOR Filter (K=" << sor_mean_k << ", std=" << sor_stddev
              << "): " << cloud_voxel->size() << " -> " << cloud_sor->size() << " points"
              << " (" << std::fixed << std::setprecision(1) << timer.ms(t2) << "ms)" << RESET << std::endl;
    pcl::io::savePCDFileBinary(out_dir + "/02_sor.pcd", *cloud_sor);

    // ============================================================
    // Stage 3: PatchWork++ ground removal
    // ============================================================
    auto t3 = timer.now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZ>);

    patchwork::Params pp;
    pp.sensor_height = 1.7;
    pp.min_range = 0.5;
    pp.max_range = 80.0;
    pp.verbose = false;
    pp.enable_RNR = true;
    pp.enable_RVPF = true;
    pp.enable_TGR = true;
    patchwork::PatchWorkpp patchworkpp(pp);

    patchwork_ground_remove(cloud_sor, cloud_ground, cloud_nonground, patchworkpp);
    std::cout << GREEN << "[Stage 3] PatchWork++ Ground Removal:" << std::endl;
    std::cout << GREEN << "         ground=" << cloud_ground->size()
              << ", nonground=" << cloud_nonground->size()
              << " (" << std::fixed << std::setprecision(1) << timer.ms(t3) << "ms)" << RESET << std::endl;
    pcl::io::savePCDFileBinary(out_dir + "/03_ground.pcd", *cloud_ground);
    pcl::io::savePCDFileBinary(out_dir + "/03_nonground.pcd", *cloud_nonground);

    // ============================================================
    // Stage 4: Euclidean clustering
    // ============================================================
    auto t4 = timer.now();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_points;
    // double cluster_tolerance = 0.3;
    // int cluster_min_size = 30;
    // int cluster_max_size = 50000;
    // euclidean_clustering(cloud_nonground, cluster_points,
    //                      cluster_tolerance, cluster_min_size, cluster_max_size);
    // std::cout << GREEN << "[Stage 4] Euclidean Clustering (tol=" << cluster_tolerance
    //           << "m, min=" << cluster_min_size << ", max=" << cluster_max_size << "): "
    //           << cluster_points.size() << " clusters"
    //           << " (" << std::fixed << std::setprecision(1) << timer.ms(t4) << "ms)" << RESET << std::endl;

    double cluster_tolerance = 0.2;
    int cluster_min_size = 50;
    int cluster_max_size = 50000;
    int fec_max_neighbors = 50;
    std::vector<pcl::PointIndices> cluster_indices =
        FEC(cloud_nonground, cluster_min_size, cluster_tolerance, fec_max_neighbors);

    for (const auto &ci : cluster_indices) {
        if (static_cast<int>(ci.indices.size()) > cluster_max_size) {
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
        c->reserve(ci.indices.size());
        for (int idx : ci.indices) {
            c->push_back(cloud_nonground->points[idx]);
        }
        c->width = c->size();
        c->height = 1;
        c->is_dense = true;
        cluster_points.push_back(c);
    }

    std::cout << GREEN << "[Stage 4] FEC Clustering (tol=" << cluster_tolerance
              << "m, min=" << cluster_min_size << ", max=" << cluster_max_size
              << ", max_neighbors=" << fec_max_neighbors << "): "
              << cluster_points.size() << " clusters"
              << " (" << std::fixed << std::setprecision(1) << timer.ms(t4) << "ms)" << RESET << std::endl;

    // Save all clusters with per-cluster RGB coloring
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < cluster_points.size(); ++i) {
        uint32_t color = hsl_to_rgb((360.0 * i) / cluster_points.size(), 0.8, 0.55);
        uint8_t r = (color >> 16) & 0xFF;
        uint8_t g = (color >> 8) & 0xFF;
        uint8_t b = color & 0xFF;
        for (const auto &pt : cluster_points[i]->points) {
            pcl::PointXYZRGB pt_rgb;
            pt_rgb.x = pt.x; pt_rgb.y = pt.y; pt_rgb.z = pt.z;
            pt_rgb.r = r; pt_rgb.g = g; pt_rgb.b = b;
            cloud_all_clusters->push_back(pt_rgb);
        }
    }
    cloud_all_clusters->width = cloud_all_clusters->size();
    cloud_all_clusters->height = 1;
    pcl::io::savePCDFileBinary(out_dir + "/04_all_clusters.pcd", *cloud_all_clusters);

    // ============================================================
    // Stage 5: PCA trunk filtering
    // ============================================================
    auto t5 = timer.now();
    std::vector<ClusterInfo> trunks;
    std::vector<ClusterInfo> discarded;
    double linearity_threshold = 0.95;
    double verticality_threshold = 0.95;  // 1 - upThres(0.05)
    double min_height = 2.5;
    pca_trunk_filter(cluster_points, trunks, discarded,
                     linearity_threshold, verticality_threshold, min_height);
    std::cout << GREEN << "[Stage 5] PCA Trunk Filter:" << std::endl;
    std::cout << GREEN << "         trunks=" << trunks.size()
              << ", discarded=" << discarded.size()
              << " (" << std::fixed << std::setprecision(1) << timer.ms(t5) << "ms)" << RESET << std::endl;

    // Save trunks with RGB coloring
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trunks(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < trunks.size(); ++i) {
        uint32_t color = hsl_to_rgb((360.0 * i) / std::max((size_t)1, trunks.size()), 0.85, 0.55);
        uint8_t r = (color >> 16) & 0xFF;
        uint8_t g = (color >> 8) & 0xFF;
        uint8_t b = color & 0xFF;
        for (const auto &pt : trunks[i].points_) {
            pcl::PointXYZRGB pt_rgb;
            pt_rgb.x = pt.x; pt_rgb.y = pt.y; pt_rgb.z = pt.z;
            pt_rgb.r = r; pt_rgb.g = g; pt_rgb.b = b;
            cloud_trunks->push_back(pt_rgb);
        }
    }
    cloud_trunks->width = cloud_trunks->size();
    cloud_trunks->height = 1;
    pcl::io::savePCDFileBinary(out_dir + "/05_trunks.pcd", *cloud_trunks);

    // Save discarded clusters with RGB coloring
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_discarded(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < discarded.size(); ++i) {
        uint32_t color = hsl_to_rgb((360.0 * i) / std::max((size_t)1, discarded.size()), 0.3, 0.55);
        uint8_t r = (color >> 16) & 0xFF;
        uint8_t g = (color >> 8) & 0xFF;
        uint8_t b = color & 0xFF;
        for (const auto &pt : discarded[i].points_) {
            pcl::PointXYZRGB pt_rgb;
            pt_rgb.x = pt.x; pt_rgb.y = pt.y; pt_rgb.z = pt.z;
            pt_rgb.r = r; pt_rgb.g = g; pt_rgb.b = b;
            cloud_discarded->push_back(pt_rgb);
        }
    }
    cloud_discarded->width = cloud_discarded->size();
    cloud_discarded->height = 1;
    pcl::io::savePCDFileBinary(out_dir + "/05_discarded.pcd", *cloud_discarded);

    // Save trunk centers as PointXYZI (intensity=1 marker)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_centers(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &t : trunks) {
        pcl::PointXYZI pt;
        pt.x = t.center_[0];
        pt.y = t.center_[1];
        pt.z = t.minZ;  // root approximated as cluster min Z
        pt.intensity = 1.0f;  // trunk marker
        cloud_centers->push_back(pt);
    }
    cloud_centers->width = cloud_centers->size();
    cloud_centers->height = 1;
    pcl::io::savePCDFileBinary(out_dir + "/06_trunk_centers.pcd", *cloud_centers);

    // Save trunk info text file
    std::ofstream info_file(out_dir + "/trunks_info.txt");
    info_file << "# Forest-LIO-Loop: Trunk Detection Summary" << std::endl;
    info_file << "# Total trunks detected: " << trunks.size() << std::endl;
    info_file << "# Format: ID | x | y | z(center) | root_z | minZ | maxZ | height | linearity | verticality | lambda1 | lambda2 | lambda3" << std::endl;
    for (size_t i = 0; i < trunks.size(); ++i) {
        double verticality = std::abs(trunks[i].normal_.dot(Eigen::Vector3d(0, 0, 1)));
        info_file << std::fixed << std::setprecision(4)
                  << i << " | "
                  << trunks[i].center_[0] << " | "
                  << trunks[i].center_[1] << " | "
                  << trunks[i].center_[2] << " | "
                  << trunks[i].minZ << " | "  // root approximated as minZ
                  << trunks[i].minZ << " | "
                  << trunks[i].maxZ << " | "
                  << (trunks[i].maxZ - trunks[i].minZ) << " | "
                  << trunks[i].linearity_ << " | "
                  << verticality << " | "
                  << trunks[i].eig_value_[2] << " | "
                  << trunks[i].eig_value_[1] << " | "
                  << trunks[i].eig_value_[0] << std::endl;
    }
    info_file.close();

    // ============================================================
    // Summary
    // ============================================================
    double total_ms = timer.ms(t_start);
    print_sep();
    std::cout << BOLDGREEN << "[Summary] Pipeline completed in " << std::fixed << std::setprecision(0)
              << total_ms << "ms" << std::endl;
    std::cout << BOLDGREEN << "  Input points:       " << cloud_in->size() << std::endl;
    std::cout << BOLDGREEN << "  After voxel:         " << cloud_voxel->size() << std::endl;
    std::cout << BOLDGREEN << "  After SOR:           " << cloud_sor->size() << std::endl;
    std::cout << BOLDGREEN << "  Ground points:       " << cloud_ground->size() << std::endl;
    std::cout << BOLDGREEN << "  Non-ground points:   " << cloud_nonground->size() << std::endl;
    std::cout << BOLDGREEN << "  Clusters found:      " << cluster_points.size() << std::endl;
    std::cout << BOLDGREEN << "  Trunks detected:     " << trunks.size() << std::endl;
    std::cout << BOLDGREEN << "  Discarded clusters:  " << discarded.size() << std::endl;
    std::cout << BOLDGREEN << "  Output dir:          " << out_dir << "/" << std::endl;
    std::cout << BOLDGREEN << "  Files:" << std::endl;
    std::cout << BOLDGREEN << "    01_voxel.pcd       - After voxel downsampling" << std::endl;
    std::cout << BOLDGREEN << "    02_sor.pcd         - After SOR outlier removal" << std::endl;
    std::cout << BOLDGREEN << "    03_ground.pcd      - PatchWork++ ground points" << std::endl;
    std::cout << BOLDGREEN << "    03_nonground.pcd   - Non-ground points for clustering" << std::endl;
    std::cout << BOLDGREEN << "    04_all_clusters.pcd - All clusters (color-coded)" << std::endl;
    std::cout << BOLDGREEN << "    05_trunks.pcd      - Trunk clusters (color-coded)" << std::endl;
    std::cout << BOLDGREEN << "    05_discarded.pcd   - Discarded clusters (color-coded)" << std::endl;
    std::cout << BOLDGREEN << "    06_trunk_centers.pcd - Trunk center points" << std::endl;
    std::cout << BOLDGREEN << "    trunks_info.txt    - Trunk details (coordinates, PCA features)" << std::endl;
    std::cout << BOLDGREEN << "============================================================" << RESET << std::endl;

    return 0;
}
