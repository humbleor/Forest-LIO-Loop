#pragma once
#ifndef _GTSAM_OPTI_H_Included_
#define _GTSAM_OPTI_H_Included_

#include "../dst/DST.h"

// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

void GTSAMResToPose(gtsam::Values &result, std::vector<TLSPos> &optiTLSVec);

// Odometry constraint between adjacent keyframes
typedef struct OdomConstraint {
  int fromID;
  int toID;
  Eigen::Vector3d t;
  Eigen::Matrix3d R;
} OdomConstraint;

void GTSAMOptimization(std::vector<TLSPos> tlsVec, std::vector<CandidateInfo> candidates_vec, std::vector<OdomConstraint> odom_vec, gtsam::Values &result, std::pair<double, double> var);

#endif // _GTSAM_OPTI_H_Included_
