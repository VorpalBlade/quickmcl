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

#include <chrono>
#include <string>

//! @file
//! @brief Timer for measuring how long things are taking

namespace quickmcl {

//! @brief Class for timing how long a segment of code takes
//!
//! Reports in destructor (RAII) or prematurely via finish()
class CodeTimer
{
public:
  //! @brief Constructor
  //! @param name Name of timer (to identify in log message).
  explicit CodeTimer(const std::string &name);

  //! Destructor for reporting via RAII
  ~CodeTimer();

  //! Finish early, suppressing RAII style reporting
  void finish();

private:
  //! Clock type in use
  using Clock = std::chrono::system_clock;

  //! Name of timer
  const std::string name;

  //! Starting point of timer
  const Clock::time_point start;

  //! Disables timer reporting from destructor.
  bool finished_early = false;
};

} // namespace quickmcl
