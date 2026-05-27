#pragma once
#ifndef _HLP_H_Included_
#define _HLP_H_Included_

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <tuple>
#include <string>
#include <cstdlib>
#include <chrono>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/voxel_grid.h>

#include "../dst/DST.h"

#define INSIGNIFICANCE -99999
#define INFINITE 99999
#define INFINITESIMAL 0.00000001
#define MAX_INF 1e10
#define MIN_INF -1e10

void matrix_to_pair(Eigen::Matrix4f &trans_matrix,
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &trans_pair);

void point_to_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr &pclPoints,
                     std::vector<Eigen::Vector3d> &vecPoints);

// Read parameters from yaml
void ReadParas(const std::string& file_path, ConfigSetting &config_setting);

// Time increment
double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin);

// Terminal color codes
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define BOLDRED     "\033[1m\033[31m"
#define BOLDGREEN   "\033[1m\033[32m"
#define BOLDYELLOW  "\033[1m\033[33m"
#define BOLDBLUE    "\033[1m\033[34m"
#define BOLDCYAN    "\033[1m\033[36m"
#define BOLDWHITE   "\033[1m\033[37m"

#endif
