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
#include "quickmcl/timer.h"

#include <ros/console.h>

//! @file
//! @brief Timer for measuring how long things are taking

namespace quickmcl {

CodeTimer::CodeTimer(const std::string &name)
  : name(name)
  , start(Clock::now())
{}

CodeTimer::~CodeTimer()
{
  if (!finished_early) {
    finish();
  }
}

void CodeTimer::finish()
{
  auto after = std::chrono::system_clock::now();

  auto delta =
      std::chrono::duration_cast<std::chrono::milliseconds>(after - start)
          .count();
  // Don't print things that are too fast to be of interest as ms
  if (delta >= 2) {
    ROS_INFO_STREAM_NAMED("timer", name << " took " << delta << " ms");
  } else {
    // If it is taking microseconds, print as debug, it is of less interest.
    auto delta_us =
        std::chrono::duration_cast<std::chrono::microseconds>(after - start)
            .count();
    ROS_DEBUG_STREAM_NAMED("timer", name << " took " << delta_us << " us");
  }
  finished_early = true;
}

} // namespace quickmcl
