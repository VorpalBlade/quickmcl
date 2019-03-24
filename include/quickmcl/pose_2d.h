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

#include "quickmcl/utils.h"

#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <iosfwd>
#include <tf2/utils.h>

//! @file
//! @brief Pose 2D and related operators

namespace quickmcl {

//! Simplified (compared to the one from geometry_msgs) 2D pose structure
//!
//! @tparam T Floating point type to use
template<typename T = float> struct Pose2D
{
  //! @name Position
  //! @{
  T x;
  T y;
  //! @}

  //! Angle in radians [-pi, pi]
  T theta;

  //! Default constructor
  Pose2D()
  {
    // Can't use "= default" due to
    // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728
    // (No idea why it works for the copy constructor below though...)
  }

  //! Value constructor
  Pose2D(T x, T y, T theta)
    : x(x)
    , y(y)
    , theta(theta)
  {}

  //! Copy constructor
  Pose2D(const Pose2D<T> &other) = default;

  //! Conversion constructor
  template<typename U>
  inline explicit Pose2D(const Pose2D<U> &other)
    : x(other.x)
    , y(other.y)
    , theta(other.theta)
  {}

  //! Constructor converting from a ROS pose message.
  inline explicit Pose2D(const geometry_msgs::Pose &p)
    : x(T(p.position.x))
    , y(T(p.position.y))
    , theta(T(tf2::getYaw(p.orientation)))
  {}

  //! Normalise the angle to [-pi, pi]
  void normalise() { theta = normalise_angle(theta); }

  //! Operator to convert to pose
  //!
  //! Note! Only sensible with normalised angle.
  inline explicit operator geometry_msgs::Pose() const
  {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.z = std::sin(theta / 2.0f);
    pose.orientation.w = std::cos(theta / 2.0f);
    return pose;
  }

  //! Operator to convert the pose to a 3D transform
  inline explicit operator Eigen::Affine3d() const
  {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translate(Eigen::Vector3d(x, y, 0));
    transform.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d(0, 0, 1)));
    return transform;
  }

  //! Operator to convert the pose to a 2D transform
  inline explicit operator Eigen::Affine2f() const
  {
    Eigen::Affine2f transform = Eigen::Affine2f::Identity();
    transform.translate(Eigen::Vector2f(x, y));
    transform.rotate(Eigen::Rotation2Df(theta));
    return transform;
  }

  //! Operator to convert the pose to a 2D transform
  inline explicit operator Eigen::Isometry2f() const
  {
    Eigen::Isometry2f transform = Eigen::Isometry2f::Identity();
    transform.translate(Eigen::Vector2f(x, y));
    transform.rotate(Eigen::Rotation2Df(theta));
    return transform;
  }

  //! @brief Convert to 4D vector.
  //!
  //! Useful for statistics, since this makes it possible to compute the mean in
  //! theta as well.
  //!
  //! @return Vector of x,y,sin(theta),cos(theta)
  inline Eigen::Vector4d to_decomposed() const
  {
    // We cannot sum angles and compute the mean like normal. There are multiple
    // possible definitions of a mean over a cyclic set, but a sensible one is
    // to sum the cos and sin separately (summing the vectors on the unit
    // circle), then taking the angle of that.
    //
    // See also
    // https://stackoverflow.com/questions/491738/how-do-you-calculate-the-average-of-a-set-of-circular-data
    // for further discussion on this.
    // x,y,sin(theta),cos(theta)
    return Eigen::Vector4d{
        x,
        y,
        std::sin(theta),
        std::cos(theta),
    };
  }

  //! @name Pose2D operator overloads
  //!
  //! @brief Various useful operator overloads.
  //!
  //! Note that none of these normalise the angle, since it isn't needed except
  //! at the very end of the computation.
  //! @{
  inline Pose2D<T> &operator+=(const Pose2D<T> &b)
  {
    x += b.x;
    y += b.y;
    theta += b.theta;
    return *this;
  }

  inline Pose2D<T> operator+(const Pose2D<T> &b) const
  {
    Pose2D<T> n(*this);
    n += b;
    return n;
  }

  inline Pose2D<T> &operator-=(const Pose2D<T> &b)
  {
    x -= b.x;
    y -= b.y;
    theta -= b.theta;
    return *this;
  }

  inline Pose2D<T> operator-(const Pose2D<T> &b) const
  {
    Pose2D n(*this);
    n -= b;
    return n;
  }

  inline Pose2D<T> &operator*=(T v)
  {
    x *= v;
    y *= v;
    theta *= v;
    return *this;
  }

  inline Pose2D<T> operator*(T v) const
  {
    Pose2D<T> b(*this);
    b *= v;
    return b;
  }
  //! @}
};

//! @brief Operator overload for scalar multiplication
//!
//! Note that angle will not be normalised.
template<typename T> Pose2D<T> operator*(T v, const Pose2D<T> &a)
{
  return a * v;
}

//! Stream output operator for logging and debugging
std::ostream &operator<<(std::ostream &os, const Pose2D<float> &pose);

//! Stream output operator for logging and debugging
std::ostream &operator<<(std::ostream &os, const Pose2D<double> &pose);

// Explicit instantiation in a single translation unit.
#ifdef POSE2D_CPP
template struct Pose2D<float>;
template struct Pose2D<double>;
#else
extern template struct Pose2D<float>;
extern template struct Pose2D<double>;
#endif
} // namespace quickmcl
