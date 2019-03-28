// QuickMCL - a computationally efficient MCL implementation for ROS
// Copyright (C) 2019  Arvid Norlander
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#pragma once

#include <Eigen/Core>

//! @file
//! @brief Mappings for ROS covariance matrices to Eigen.

namespace quickmcl_node {

//! Eigen mapping of ROS covariance matrix
using RosCovarianceMapping =
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>;

//! Read only Eigen mapping of ROS covariance matrix
using ConstRosCovarianceMapping =
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>;

} // namespace quickmcl_node
