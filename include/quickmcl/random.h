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
#include <Eigen/Eigenvalues>
#include <random>

//! @file
//! @brief Functionality for random number generation

namespace quickmcl {

//! @brief Generate multivariate Gaussian random vectors.
//!
//! Taken from
//! https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
//! but converted to be thread safe by explicitly taking a generator.
//!
//! @tparam VectorT Vector type
//! @tparam MatrixT Matrix type
template<typename VectorT = Eigen::VectorXd, typename MatrixT = Eigen::MatrixXd>
class normal_random_variable
{
public:
  //! Type of vector
  using VectorType = VectorT;
  //! Type of matrix
  using MatrixType = MatrixT;

  static_assert(std::is_same<typename VectorType::Scalar,
                             typename MatrixType::Scalar>::value,
                "Mismatching scalar types");

  //! Constructor with zero mean and specific covariance.
  normal_random_variable(const MatrixType &covar, std::mt19937 &gen)
    : normal_random_variable(VectorType::Zero(covar.rows()), covar, gen)
  {}

  //! Constructor with specific mean and covariance.
  normal_random_variable(const VectorType &mean,
                         const MatrixType &covar,
                         std::mt19937 &gen)
    : mean(mean)
    , gen(gen)
  {
    Eigen::SelfAdjointEigenSolver<MatrixType> eigenSolver(covar);
    transform = eigenSolver.eigenvectors() *
                eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  //! Generate vectors from distribution.
  VectorType operator()()
  {
    return mean + transform * VectorType{mean.size()}.unaryExpr(
                                  [this](auto) { return dist(gen); });
  }

private:
  //! Mean of this generator
  const VectorType mean;
  //! Transform to apply covariance to uniform normal distribution.
  MatrixType transform;

  //! Random number generator
  std::mt19937 &gen;
  //! Univariate normal distribution
  std::normal_distribution<typename VectorType::Scalar> dist;
};

} // namespace quickmcl
