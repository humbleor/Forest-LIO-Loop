#include "KeyframeManager.h"
#include <cmath>
#include <iostream>

KeyframeManager::KeyframeManager(double min_distance, double min_angle_deg, double min_time_sec)
    : min_distance_(min_distance),
      min_angle_deg_(min_angle_deg),
      min_time_sec_(min_time_sec)
{
}

bool KeyframeManager::isKeyFrame(const Eigen::Vector3d &position,
                                  const Eigen::Quaterniond &orientation,
                                  double timestamp)
{
    if (!has_last_kf_) return true; // First frame always a keyframe

    // Time check
    double dt = timestamp - last_kf_.timestamp;
    if (dt < min_time_sec_) {
        // std::cout << "[KF Reject] #" << "time=" << dt << "s < " << min_time_sec_ << "s" << std::endl;
        return false;
    }

    // Translation check
    double dx = position.x() - last_kf_.position.x();
    double dy = position.y() - last_kf_.position.y();
    double dz = position.z() - last_kf_.position.z();
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < min_distance_) {
        // std::cout << "[KF Reject] #" << "dist=" << distance << "m < " << min_distance_ << "m" << std::endl;
        return false;
    }

    // Rotation check: angle between rotations
    // Eigen::Matrix3d R_diff = last_kf_.rotation.transpose() * orientation.toRotationMatrix();
    // double angle = std::acos(std::min(1.0, std::max(-1.0, (R_diff.trace() - 1.0) / 2.0)));
    // double angle_deg = angle * 180.0 / M_PI;
    // if (angle_deg < min_angle_deg_) {
    //     // std::cout << "[KF Reject] #" << "angle=" << angle_deg << "deg < " << min_angle_deg_ << "deg" << std::endl;
    //     return false;
    // }

    // std::cout << "\033[32m[KF Accept] dt=" << dt << "s, dist=" << distance
    //           << "m, angle=" << angle_deg << "deg\033[0m" << std::endl;
    return true;
}

Eigen::Matrix4d KeyframeManager::computeRelativeTransform(const Eigen::Vector3d &curr_pos,
                                                           const Eigen::Quaterniond &curr_ori)
{
    Eigen::Matrix4d T_curr = Eigen::Matrix4d::Identity();
    T_curr.block<3, 3>(0, 0) = curr_ori.toRotationMatrix();
    T_curr.block<3, 1>(0, 3) = curr_pos;

    Eigen::Matrix4d T_last = Eigen::Matrix4d::Identity();
    T_last.block<3, 3>(0, 0) = last_kf_.rotation;
    T_last.block<3, 1>(0, 3) = last_kf_.position;

    // T_{last->curr} = T_last^{-1} * T_curr
    return T_last.inverse() * T_curr;
}

bool KeyframeManager::addKeyFrame(int id, double timestamp,
                                   const Eigen::Vector3d &position,
                                   const Eigen::Quaterniond &orientation,
                                   PointCloudXYZI::Ptr cloud_world)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (!isKeyFrame(position, orientation, timestamp))
        return false;

    KeyFrame kf;
    kf.id = id;
    kf.timestamp = timestamp;
    kf.position = position;
    kf.rotation = orientation.toRotationMatrix();
    kf.cloud_world = cloud_world;

    // Compute relative transform to previous keyframe
    if (has_last_kf_)
    {
        kf.rel_pose_prev = computeRelativeTransform(position, orientation);
    }
    else
    {
        kf.rel_pose_prev = Eigen::Matrix4d::Identity();
    }

    keyframes_.push_back(kf);
    last_kf_ = kf;
    has_last_kf_ = true;

    std::cout << "\033[32m[Keyframe] Saved KF #" << kf.id
              << " (total: " << keyframes_.size() << ")\033[0m" << std::endl;

    return true;
}

bool KeyframeManager::getKeyFrame(int id, KeyFrame &kf)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto &k : keyframes_)
    {
        if (k.id == id)
        {
            kf = k;
            return true;
        }
    }
    return false;
}

std::vector<KeyFrame> KeyframeManager::getAllKeyFrames()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return keyframes_;
}

size_t KeyframeManager::size()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return keyframes_.size();
}

bool KeyframeManager::getLastKeyFrame(KeyFrame &kf)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (keyframes_.empty()) return false;
    kf = keyframes_.back();
    return true;
}
