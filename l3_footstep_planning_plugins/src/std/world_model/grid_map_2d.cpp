// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/l3_gridmap_2d/src/GridMap2D.cpp $
// SVN $Id: GridMap2D.cpp 3276 2012-09-27 12:39:16Z hornunga@informatik.uni-freiburg.de $

/*
 * A simple 2D gridmap structure
 *
 * Copyright 2011 Armin Hornung, University of Freiburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/console.h>
#include <l3_footstep_planning_plugins/std/world_model/grid_map_2d.h>

namespace l3_gridmap_2d
{
GridMap2D::GridMap2D()
  : frame_id_("/map")
{}

GridMap2D::GridMap2D(const nav_msgs::OccupancyGridConstPtr& gridMap) { setMap(gridMap); }

void GridMap2D::setMap(const nav_msgs::OccupancyGridConstPtr& gridMap)
{
  map_info_ = gridMap->info;
  frame_id_ = gridMap->header.frame_id;
  // allocate map structs so that x/y in the world correspond to x/y in the image
  // (=> cv::Mat is rotated by 90 deg, because it's row-major!)
  binary_map_ = cv::Mat(map_info_.width, map_info_.height, CV_8UC1);
  dist_map_ = cv::Mat(binary_map_.size(), CV_32FC1);

  std::vector<signed char>::const_iterator mapDataIter = gridMap->data.begin();

  // TODO check / param
  unsigned char map_occ_thres = 70;

  // iterate over map, store in image
  // (0,0) is lower left corner of OccupancyGrid
  for (unsigned int j = 0; j < map_info_.height; ++j)
  {
    for (unsigned int i = 0; i < map_info_.width; ++i)
    {
      if (*mapDataIter > map_occ_thres)
      {
        // m_mapInfo.height-1-i
        binary_map_.at<uchar>(i, j) = 0;
      }
      else
      {
        binary_map_.at<uchar>(i, j) = 255;
      }
      mapDataIter++;
    }
  }
  cv::distanceTransform(binary_map_, dist_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  // distance map now contains distance in meters:
  dist_map_ = dist_map_ * map_info_.resolution;

  // ROS_INFO("GridMap2D created with %d x %d cells at %f resolution.", m_mapInfo.width, m_mapInfo.height, m_mapInfo.resolution);
}

void GridMap2D::setMap(const cv::Mat& binaryMap)
{
  binary_map_ = binaryMap.clone();
  dist_map_ = cv::Mat(binary_map_.size(), CV_32FC1);

  cv::distanceTransform(binary_map_, dist_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  // distance map now contains distance in meters:
  dist_map_ = dist_map_ * map_info_.resolution;

  ROS_INFO("GridMap2D copied from existing cv::Mat with %d x %d cells at %f resolution.", map_info_.width, map_info_.height, map_info_.resolution);
}

void GridMap2D::inflateMap(double inflationRadius)
{
  binary_map_ = (dist_map_ <= inflationRadius);
  // recompute distance map with new binary map:
  cv::distanceTransform(binary_map_, dist_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  dist_map_ = dist_map_ * map_info_.resolution;
}

// See costmap2D for mapToWorld / worldToMap implementations:

void GridMap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = map_info_.origin.position.x + (mx + 0.5) * map_info_.resolution;
  wy = map_info_.origin.position.y + (my + 0.5) * map_info_.resolution;
}

void GridMap2D::worldToMapNoBounds(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  mx = (int)((wx - map_info_.origin.position.x) / map_info_.resolution);
  my = (int)((wy - map_info_.origin.position.y) / map_info_.resolution);
}

bool GridMap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < map_info_.origin.position.x || wy < map_info_.origin.position.y)
    return false;

  mx = (int)((wx - map_info_.origin.position.x) / map_info_.resolution);
  my = (int)((wy - map_info_.origin.position.y) / map_info_.resolution);

  if (mx < map_info_.width && my < map_info_.height)
    return true;

  return false;
}

bool GridMap2D::inMapBounds(double wx, double wy) const
{
  unsigned mx, my;
  return worldToMap(wx, wy, mx, my);
}

float GridMap2D::distanceMapAt(double wx, double wy) const
{
  unsigned mx, my;

  if (worldToMap(wx, wy, mx, my))
    return dist_map_.at<float>(mx, my);
  else
    return -1.0f;
}

uchar GridMap2D::binaryMapAt(double wx, double wy) const
{
  unsigned mx, my;

  if (worldToMap(wx, wy, mx, my))
    return binary_map_.at<uchar>(mx, my);
  else
    return 0;
}

bool GridMap2D::isOccupiedAt(double wx, double wy) const
{
  unsigned mx, my;
  if (worldToMap(wx, wy, mx, my))
    return isOccupiedAtCell(mx, my);
  else
    return true;
}
}  // namespace l3_gridmap_2d
