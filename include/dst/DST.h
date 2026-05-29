#pragma once
#ifndef _DST_H_Included_
#define _DST_H_Included_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// for hash function
#define HASH_P 116101
#define MAX_N 10000000000

// parameters for preprocessing, std, and place recognition
typedef struct ConfigSetting {

	// for tri descriptor
	int descriptor_near_num = 10;
	double descriptor_min_len = 2;
	double descriptor_max_len = 50;
  double descriptor_len_diff = 0.1;
	double side_resolution = 0.1;

	// for place recognition
	int candidate_num = 50;
	double rough_dis_threshold = 0.1;
	double vertex_diff_threshold = 0;
	double icp_threshold = 0.15;
	double dist_candi_frames_verify = 2;
	double dis_geo_verify = 3.0;
	int lGrp_Ele_Min = 15;

	// for PCL Euclidean clustering
	int min_component_size = 30;
	double tolorance = 0.3;
	int max_n = 50;

	// for SOR outlier removal
	double sor_stddev = 1.0;

	// PCA trunk filtering
	double linearityThres = 0.95;
  double upThres = 0.3;
	double clusterHeight = 2.5;
  int centerSelection = 0;

	// Trunk cluster merging (post-PCA dedup)
	double trunk_merge_dist = 0.4;     // max horizontal distance between cluster centers (m)
	double trunk_merge_z_overlap = 0.1; // min height overlap ratio to merge
	double trunk_merge_max_z_gap = 3.0; // max Z gap between cluster ranges (m)

} ConfigSetting;

// structure for Cluster
typedef struct Cluster {
  pcl::PointXYZINormal p_center_;
  pcl::PointCloud<pcl::PointXYZ> points_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Vector3d eig_value_;
  double minZ;
  double maxZ;
  double root;
  double linearity_ = 0;
  double planarity_ = 0;
  double scatering_ = 0;
  bool is_plane_ = false;
  bool is_line_ = false;
} Cluster;

// Structure for Multi-level Descriptor
typedef struct TriDesc {
  // the side lengths of STDesc, arranged from short to long
  Eigen::Vector3d side_length_;

  // projection angle between vertices
  Eigen::Vector3d angle_;

  Eigen::Vector3d center_;
  unsigned int frame_id_;

  // three vertexs
  Eigen::Vector3d vertex_A_;
  Eigen::Vector3d vertex_B_;
  Eigen::Vector3d vertex_C_;

  // some other inform attached to each vertex,e.g., intensity
  Eigen::Vector3d vertex_attached_;
} TriDesc;

typedef struct FrameInfo{
  std::vector<TriDesc> desc_;
  unsigned int frame_id_;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr currCenter;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr currCenterFix;
  pcl::PointCloud<pcl::PointXYZ>::Ptr currPoints;
  Eigen::Matrix4d T_world_kf;  // world-to-keyframe-body transform
  bool has_pose = false;
}FrameInfo;

// std descriptor match lists
typedef struct TriMatchList {
  std::vector<std::pair<TriDesc, TriDesc>> match_list_;
  std::pair<int, int> match_id_;
  double mean_dis_;
} TriMatchList;

// candidate information
typedef struct CandidateInfo {
  int currFrameID;
  std::vector<std::pair<int, double>> candidateIDScore;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> relativePose;
  std::vector<std::vector<std::pair<TriDesc, TriDesc>>> triMatch;
} CandidateInfo;

// pose information of TLS stations
typedef struct TLSPos {
  int ID;
  Eigen::Vector3d t;
  Eigen::Matrix3d R;
  bool isValued = false;
}TLSPos;

// location of STD, and a operator to define whether is equal or not
class TriDesc_LOC {
public:
  int64_t x, y, z, a, b, c;

  TriDesc_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int64_t va = 0,
             int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const TriDesc_LOC &other) const {
    // use three attributes
    return (x == other.x && y == other.y && z == other.z);
    // use six attributes
    // return (x == other.x && y == other.y && z == other.z && a == other.a &&
    //         b == other.b && c == other.c);
  }
};

// hash mapping for TriDesc_LOC, input TriDesc_LOC, output int64
template <> struct std::hash<TriDesc_LOC> {
  int64_t operator()(const TriDesc_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

// location of uniform voxel
class UNI_VOXEL_LOC {
public:
    int64_t x, y, z;

    UNI_VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const UNI_VOXEL_LOC &other) const {
        return (x == other.x && y == other.y && z == other.z);
    }
};

// hash mapping for VOXEL_LOC, input VOXEL_LOC, output int64
template <> struct std::hash<UNI_VOXEL_LOC> {
  int64_t operator()(const UNI_VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

#endif // _DST_H_Included_
