#pragma once
#ifndef KEYFRAME_MANAGER_H
#define KEYFRAME_MANAGER_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>
#include <vector>
#include "dst/DST.h"

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

struct KeyFrame {
    int id;
    double timestamp;
    Eigen::Vector3d position;       // World frame position
    Eigen::Matrix3d rotation;       // World frame rotation

    PointCloudXYZI::Ptr cloud_world; // World frame point cloud (from /cloud_registered)

    // Relative transform to previous keyframe (for submap building)
    Eigen::Matrix4d rel_pose_prev; // T_{i-1->i} in body frame
};

class KeyframeManager
{
public:
    KeyframeManager(double min_distance, double min_angle_deg, double min_time_sec);

    // Add a new keyframe candidate; returns true if saved as keyframe
    bool addKeyFrame(int id, double timestamp,
                     const Eigen::Vector3d &position,
                     const Eigen::Quaterniond &orientation,
                     PointCloudXYZI::Ptr cloud_world);

    // Get keyframe by ID
    bool getKeyFrame(int id, KeyFrame &kf);

    // Get all keyframes (thread-safe copy)
    std::vector<KeyFrame> getAllKeyFrames();

    // Get keyframe count
    size_t size();

    // Get last keyframe
    bool getLastKeyFrame(KeyFrame &kf);

private:
    double min_distance_;
    double min_angle_deg_;
    double min_time_sec_;

    std::vector<KeyFrame> keyframes_;
    KeyFrame last_kf_;
    bool has_last_kf_ = false;
    std::mutex mtx_;

    // Check if current frame meets keyframe criteria
    bool isKeyFrame(const Eigen::Vector3d &position,
                    const Eigen::Quaterniond &orientation,
                    double timestamp);

    // Compute relative transform from last keyframe to current (body frame)
    Eigen::Matrix4d computeRelativeTransform(const Eigen::Vector3d &curr_pos,
                                              const Eigen::Quaterniond &curr_ori);
};

#endif
