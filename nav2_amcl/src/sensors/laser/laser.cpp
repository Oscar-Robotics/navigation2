/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <sys/types.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

Laser::Laser(size_t max_beams, map_t * map)
: max_samples_(0), max_obs_(0), temp_obs_(NULL)
{
  max_beams_ = max_beams;
  map_ = map;
}

Laser::~Laser()
{
  if (temp_obs_) {
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k];
    }
    delete[] temp_obs_;
  }
}

void
Laser::reallocTempData(int new_max_samples, int new_max_obs)
{
  if (temp_obs_) {
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k];
    }
    delete[] temp_obs_;
  }
  max_obs_ = new_max_obs;
  max_samples_ = fmax(max_samples_, new_max_samples);

  temp_obs_ = new double *[max_samples_]();
  for (int k = 0; k < max_samples_; k++) {
    temp_obs_[k] = new double[max_obs_]();
  }
}

void
Laser::SetLaserPose(pf_vector_t & laser_pose)
{
  laser_pose_ = laser_pose;
}

bool
Laser::CheckFootprintForObstacles(const pf_vector_t & pose)
{
  // Convert the pose into grid coordinates
  int i = MAP_GXWX(map_, pose.v[0]);
  int j = MAP_GYWY(map_, pose.v[1]);

  // Get the cell
  map_cell_t cell = map_->cells[MAP_INDEX(map_, i, j)];

  if (cell.occ_state == +1) {
    return true;
  }

  if (footprint_type_ == FootprintType::CIRCLE) {
    return footprint_radius_ < cell.occ_dist;
  } else if (footprint_type_ == FootprintType::SQUARE) {
    
    if (footprint_radius_ < cell.occ_dist)
      return false;

    // Calculate the direction of the obstacle relative to the robot's orientation
    double relative_occ_dir = cell.occ_dir - pose.v[2];

    // Normalize the relative direction to [-π, π] for consistency
    if (relative_occ_dir > M_PI) {
        relative_occ_dir -= 2 * M_PI;
    } else if (relative_occ_dir < -M_PI) {
        relative_occ_dir += 2 * M_PI;
    }

    // Decompose the distance to the obstacle along the robot's axes
    double dist_x = cell.occ_dist * cos(relative_occ_dir);
    double dist_y = cell.occ_dist * sin(relative_occ_dir);

    // Check if the obstacle is within the bounds of the square footprint
    return (fabs(dist_x) > footprint_half_side_length_ && fabs(dist_y) > footprint_half_side_length_);
  }

  return false;
}

void
Laser::getResidualErrors(const pf_vector_t & pose, const LaserData * laser_data, float * residuals)
{
  pf_vector_t updated_pose = pf_vector_coord_add(laser_pose_, pose);
  for (int i = 0; i < laser_data->range_count; i++) {
    // Get the range and angle
    double obs_range = laser_data->ranges[i][0];
    double obs_bearing = laser_data->ranges[i][1];

    if (obs_range >= laser_data->range_max) {
      residuals[i] = -1;
      continue;
    }

    // Check for NaN
    if (obs_range != obs_range) {
      residuals[i] = -1;
      continue;
    }

    // Transform the point into the map frame
    double x = updated_pose.v[0] + obs_range * cos(updated_pose.v[2] + obs_bearing);
    double y = updated_pose.v[1] + obs_range * sin(updated_pose.v[2] + obs_bearing);

    int mi, mj;
    mi = MAP_GXWX(map_, x);
    mj = MAP_GYWY(map_, y);

    // Get the distance to the nearest obstacle
    if (!MAP_VALID(map_, mi, mj)) {
      residuals[i] = -1;
    } else {
      residuals[i] = map_->cells[MAP_INDEX(map_, mi, mj)].occ_dist;
    }
  }
}

}  // namespace nav2_amcl
