/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_costmap_2d
{
Costmap2D::Costmap2D(
  unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
  double origin_x, double origin_y, unsigned char default_value)
: size_x_(cells_size_x), size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x),
  origin_y_(origin_y), costmap_(NULL), default_value_(default_value)
{
  // 创建新锁
  access_ = new mutex_t();

  // 初始化地图
  // create the costmap
  initMaps(size_x_, size_y_);
  resetMaps();
}

Costmap2D::Costmap2D(const nav_msgs::msg::OccupancyGrid & map)
: default_value_(FREE_SPACE)
{
  // 创建新锁
  access_ = new mutex_t();

  // 从 map 中拿参数
  // fill local variables
  size_x_ = map.info.width;
  size_y_ = map.info.height;
  resolution_ = map.info.resolution;
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;

  // 创建地图
  // TODO: 为啥不用 initMaps 初始化地图?
  // create the costmap
  costmap_ = new unsigned char[size_x_ * size_y_];

  // 用地图数据来 fill costmap
  // fill the costmap with a data
  int8_t data;
  for (unsigned int it = 0; it < size_x_ * size_y_; it++) {
    data = map.data[it];
    if (data == nav2_util::OCC_GRID_UNKNOWN) {
      costmap_[it] = NO_INFORMATION;
    } else {
      // 把 0-100 数据线性变换到 0-254 之间的值, 然后给 costmap
      // Linear conversion from OccupancyGrid data range [OCC_GRID_FREE..OCC_GRID_OCCUPIED]
      // to costmap data range [FREE_SPACE..LETHAL_OBSTACLE]
      costmap_[it] = std::round(
        static_cast<double>(data) * (LETHAL_OBSTACLE - FREE_SPACE) /
        (nav2_util::OCC_GRID_OCCUPIED - nav2_util::OCC_GRID_FREE));
    }
  }
}

void Costmap2D::deleteMaps()
{
  // 锁上, 然后删除
  // clean up data
  std::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = NULL;
}

void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
  // 锁上, 删除 costmap, 根据给定尺寸创建新的 costmap
  std::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = new unsigned char[size_x * size_y];
}

void Costmap2D::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution,
  double origin_x, double origin_y)
{
  // 根据新的参数来重置地图
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  initMaps(size_x, size_y);

  // reset our maps to have no information
  resetMaps();
}

void Costmap2D::resetMaps()
{
  // 这里直接在存储里用默认值填充 costmap
  std::unique_lock<mutex_t> lock(*access_);
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

// 把 x0-xn, y0-yn 设置为默认值
void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  resetMapToValue(x0, y0, xn, yn, default_value_);
}

void Costmap2D::resetMapToValue(
  unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn, unsigned char value)
{
  std::unique_lock<mutex_t> lock(*(access_));
  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_) {
    memset(costmap_ + y, value, len * sizeof(unsigned char));
  }
}

bool Costmap2D::copyCostmapWindow(
  const Costmap2D & map, double win_origin_x, double win_origin_y,
  double win_size_x,
  double win_size_y)
{
  // 检查地图不是自己
  // check for self windowing
  if (this == &map) {
    // ROS_ERROR("Cannot convert this costmap into a window of itself");
    return false;
  }

  // 删除老地图
  // clean up old data
  deleteMaps();

  // TODO: 这里还没有定地图边界, 但是 worldToMap 需要地图边界信息判断, 这样合适吗?
  // 给新地图计算边界, 左下角和右上角
  // compute the bounds of our new map
  unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y) ||
    !map.worldToMap(
      win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x,
      upper_right_y))
  {
    // ROS_ERROR("Cannot window a map that the window bounds don't fit inside of");
    return false;
  }

  // 计算 costmap 参数
  // 本地图尺寸就是目标地图尺寸就是截取地图区域的尺寸
  size_x_ = upper_right_x - lower_left_x;
  size_y_ = upper_right_y - lower_left_y;
  resolution_ = map.resolution_;
  origin_x_ = win_origin_x;
  origin_y_ = win_origin_y;

  // 根据新的参数初始化地图
  // initialize our various maps and reset markers for inflation
  initMaps(size_x_, size_y_);

  // 复制静态地图选定区域
  // 这里是从 lower_left_x, lower_left_y 开始的 size_x 和 size_y 大小的原地图
  // 复制到 0, 0 开始的 size_x 和 size_y 大小的目标地图
  // 这里目标地图就是原地图截取的区域
  // copy the window of the static map and the costmap that we're taking
  copyMapRegion(
    map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_,
    size_x_,
    size_y_);
  return true;
}

bool Costmap2D::copyWindow(
  const Costmap2D & source,
  unsigned int sx0, unsigned int sy0, unsigned int sxn, unsigned int syn,
  unsigned int dx0, unsigned int dy0)
{
  const unsigned int sz_x = sxn - sx0;
  const unsigned int sz_y = syn - sy0;

  // source map 的窗口不应该超过 source map size
  if (sxn > source.getSizeInCellsX() || syn > source.getSizeInCellsY()) {
    return false;
  }

  // 当前的目标存放窗口的最大尺寸不应该超过当前 map size
  if (dx0 + sz_x > size_x_ || dy0 + sz_y > size_y_) {
    return false;
  }

  // 直接复制过去
  copyMapRegion(
    source.costmap_, sx0, sy0, source.size_x_,
    costmap_, dx0, dy0, size_x_,
    sz_x, sz_y);
  return true;
}

Costmap2D & Costmap2D::operator=(const Costmap2D & map)
{
  // check for self assignement
  if (this == &map) {
    return *this;
  }

  // 清除老的地图数据
  // clean up old data
  deleteMaps();

  // 复制地图参数
  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;

  // 初始化地图
  // initialize our various maps
  initMaps(size_x_, size_y_);

  // 直接复制地图, 操作地址
  // copy the cost map
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

  return *this;
}

Costmap2D::Costmap2D(const Costmap2D & map)
: costmap_(NULL)
{
  // 锁住, 直接替换自身
  access_ = new mutex_t();
  *this = map;
}

// 默认构造函数直接初始化所有参数
// just initialize everything to NULL by default
Costmap2D::Costmap2D()
: size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
  access_ = new mutex_t();
}

// 删除地图和锁
Costmap2D::~Costmap2D()
{
  deleteMaps();
  delete access_;
}

unsigned int Costmap2D::cellDistance(double world_dist)
{
  // 通过实际距离计算地图中 cell 的距离
  double cells_dist = std::max(0.0, ceil(world_dist / resolution_));
  return (unsigned int)cells_dist;
}

unsigned char * Costmap2D::getCharMap() const
{
  // 获取 char * 数组形式的 costmap
  return costmap_;
}

unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  // 获取指定 index 的 cost
  return costmap_[getIndex(mx, my)];
}

unsigned char Costmap2D::getCost(unsigned int undex) const
{
  // 也可以直接 1D index 拿 cost
  return costmap_[undex];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  // 设置指定 index 的 cost
  costmap_[getIndex(mx, my)] = cost;
}

void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  // 无限制的 map 上的坐标转换到世界坐标
  // 因为 wx, wy 转换到 mx, my 的时候是向下取整的
  // 所以 mapToWorld 的时候加个 0.5, 取 cell 中间位置的世界坐标
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool Costmap2D::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  // 世界点不应该小于地图原点, 原点就是左下角点
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }

  // 计算地图上的像素 cell 有多少, 也就是 wx, wy 在地图上的坐标 mx, my
  mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

  // 计算得到的 cell 不应该大于等于尺寸, 这里 size_x 和 size_y 就是目标地图的尺寸
  if (mx < size_x_ && my < size_y_) {
    return true;
  }
  return false;
}

void Costmap2D::worldToMapNoBounds(double wx, double wy, int & mx, int & my) const
{
  // 这里不考虑边界转换 worldToMap
  mx = static_cast<int>((wx - origin_x_) / resolution_);
  my = static_cast<int>((wy - origin_y_) / resolution_);
}

void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int & mx, int & my) const
{
  // 这里做强制边界转换，如果超过边界就取边界
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  if (wx < origin_x_) {
    mx = 0;
  } else if (wx > resolution_ * size_x_ + origin_x_) {
    mx = size_x_ - 1;
  } else {
    mx = static_cast<int>((wx - origin_x_) / resolution_);
  }

  if (wy < origin_y_) {
    my = 0;
  } else if (wy > resolution_ * size_y_ + origin_y_) {
    my = size_y_ - 1;
  } else {
    my = static_cast<int>((wy - origin_y_) / resolution_);
  }
}

void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
  // 更新地图原点, 将新地图原点计算到地图坐标
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // 计算和原点 cell 相关联的世界坐标, 靠左边界的坐标
  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // 截取边界
  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  // 计算 size
  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // 弄了一个 local map 来保存动态障碍物, 这个 window 就是原地图中原点变化后的地图
  // we need a map to store the obstacles in the window temporarily
  unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];

  // 把 costmap 中的原点变化后的窗口区域复制到 local_map 中
  // copy the local window in the costmap to the local map
  copyMapRegion(
    costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x,
    cell_size_x,
    cell_size_y);

  // 重置地图
  // now we'll set the costmap to be completely unknown if we track unknown space
  resetMaps();

  // 这里更新原点了
  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // 计算起始点, 因为 cell_ox 和 cell_oy 都是原点
  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // 在把 local_map 中的东西全部复制到新的 map 中, 新的 map size 保持不变
  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(
    local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x,
    cell_size_y);

  // 最后删掉这个临时创建的 map
  // make sure to clean up
  delete[] local_map;
}

bool Costmap2D::setConvexPolygonCost(
  const std::vector<geometry_msgs::msg::Point> & polygon,
  unsigned char cost_value)
{
  // 假设 polygon 是在 global frame 下给的, 就需要变换到地图坐标系下
  // we assume the polygon is given in the global_frame...
  // we need to transform it to map coordinates
  std::vector<MapLocation> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i) {
    MapLocation loc;
    if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y)) {
      // ("Polygon lies outside map bounds, so we can't fill it");
      return false;
    }
    map_polygon.push_back(loc);
  }

  std::vector<MapLocation> polygon_cells;

  // 从 map_polygon 获取所有其包含的 cell
  // get the cells that fill the polygon
  convexFillCells(map_polygon, polygon_cells);

  // 然后把每一个 cell 都填上指定 cost
  // set the cost of those cells
  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    costmap_[index] = cost_value;
  }
  return true;
}

void Costmap2D::polygonOutlineCells(
  const std::vector<MapLocation> & polygon,
  std::vector<MapLocation> & polygon_cells)
{
  // 先创建 polygon outline cells
  PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
  // 遍历所有相邻连线
  for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
    raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
  }
  if (!polygon.empty()) {
    // 如果不为空, 最后还需要取最后的点到初始点的连线段
    unsigned int last_index = polygon.size() - 1;
    // we also need to close the polygon by going from the last point to the first
    raytraceLine(
      cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x,
      polygon[0].y);
  }
}

void Costmap2D::convexFillCells(
  const std::vector<MapLocation> & polygon,
  std::vector<MapLocation> & polygon_cells)
{
  // 至少要三角形
  // we need a minimum polygon of a triangle
  if (polygon.size() < 3) {
    return;
  }

  // 首先拿到边缘线的 cells
  // first get the cells that make up the outline of the polygon
  polygonOutlineCells(polygon, polygon_cells);

  // NOTE: 快速冒泡排序简单实现
  // 采用快速冒泡排序
  // quick bubble sort to sort points by x
  MapLocation swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1) {
    if (polygon_cells[i].x > polygon_cells[i + 1].x) {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0) {
        --i;
      }
    } else {
      ++i;
    }
  }

  i = 0;
  MapLocation min_pt;
  MapLocation max_pt;
  // 最小 x 到最大 x
  unsigned int min_x = polygon_cells[0].x;
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

  // 遍历所有 x
  // walk through each column and mark cells inside the polygon
  for (unsigned int x = min_x; x <= max_x; ++x) {
    if (i >= polygon_cells.size() - 1) {
      break;
    }

    // TODO: 这里开始取的两个点并没有判断这两个点是否取在 x 的位置?
    // 先取相邻两个 cell 的点作为连接线
    if (polygon_cells[i].y < polygon_cells[i + 1].y) {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    } else {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    // 取完之后跳过这两个点, 遍历寻找 x 坐标为当前 x 值的 cell
    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].x == x) {
      if (polygon_cells[i].y < min_pt.y) {
        min_pt = polygon_cells[i];
      } else if (polygon_cells[i].y > max_pt.y) {
        max_pt = polygon_cells[i];
      }
      ++i;
    }

    // 获得了 min_y 和 max_y, 我们就可以把他们放入 polygon_cells 中了
    MapLocation pt;
    // loop though cells in the column
    for (unsigned int y = min_pt.y; y <= max_pt.y; ++y) {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

unsigned int Costmap2D::getSizeInCellsX() const
{
  return size_x_;
}

unsigned int Costmap2D::getSizeInCellsY() const
{
  return size_y_;
}

double Costmap2D::getSizeInMetersX() const
{
  return (size_x_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getOriginX() const
{
  return origin_x_;
}

double Costmap2D::getOriginY() const
{
  return origin_y_;
}

double Costmap2D::getResolution() const
{
  return resolution_;
}

bool Costmap2D::saveMap(std::string file_name)
{
  // 打开保存地图名称的 FILE
  FILE * fp = fopen(file_name.c_str(), "w");

  // 如果没打开, false
  if (!fp) {
    return false;
  }

  // 打印到文件 fp 中
  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  for (unsigned int iy = 0; iy < size_y_; iy++) {
    for (unsigned int ix = 0; ix < size_x_; ix++) {
      unsigned char cost = getCost(ix, iy);
      fprintf(fp, "%d ", cost);
    }
    fprintf(fp, "\n");
  }
  // 写完关闭 fp
  fclose(fp);
  return true;
}

}  // namespace nav2_costmap_2d
