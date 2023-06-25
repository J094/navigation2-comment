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
#include "nav2_costmap_2d/layered_costmap.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "nav2_costmap_2d/footprint.hpp"


using std::vector;

namespace nav2_costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown)
: primary_costmap_(), combined_costmap_(),
  global_frame_(global_frame),
  rolling_window_(rolling_window),
  current_(false),
  minx_(0.0),
  miny_(0.0),
  maxx_(0.0),
  maxy_(0.0),
  bx0_(0),
  bxn_(0),
  by0_(0),
  byn_(0),
  initialized_(false),
  size_locked_(false),
  circumscribed_radius_(1.0),
  inscribed_radius_(0.1)
{
  if (track_unknown) {
    // 如果是在未知空间探索, 那么开始都是未知的 NO_INFOMATION
    primary_costmap_.setDefaultValue(255);
    combined_costmap_.setDefaultValue(255);
  } else {
    // 如果是已知的, 那么开始都认为是可通行区域 FREE_SPACE
    primary_costmap_.setDefaultValue(0);
    combined_costmap_.setDefaultValue(0);
  }
}

LayeredCostmap::~LayeredCostmap()
{
  // 析构清空 plugins 和 filters
  while (plugins_.size() > 0) {
    plugins_.pop_back();
  }
  while (filters_.size() > 0) {
    filters_.pop_back();
  }
}

void LayeredCostmap::addPlugin(std::shared_ptr<Layer> plugin)
{
  // TODO: 貌似没有对 combined_costmap 造成直接修改, 为啥要锁?
  // 先锁住, 然后添加 plugin
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  plugins_.push_back(plugin);
}

void LayeredCostmap::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution,
  double origin_x,
  double origin_y,
  bool size_locked)
{
  // 先锁住
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  size_locked_ = size_locked;
  // resize 两个 costmap
  primary_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  combined_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);

  // 这里又要 resize 所有 plugins 和 filters, 为了让他们的 size 和变化后的 costmap 对应
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    (*plugin)->matchSize();
  }
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end(); ++filter)
  {
    // TODO: 这里奇怪, filter 明明没有实现 matchSize, 或者根本不需要?
    (*filter)->matchSize();
  }
}

bool LayeredCostmap::isOutofBounds(double robot_x, double robot_y)
{
  unsigned int mx, my;
  return !combined_costmap_.worldToMap(robot_x, robot_y, mx, my);
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // 锁住 costmap, 在更新完地图前不让其改变
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));

  // 如果是 rolling window, 那么地图原点应该跟着机器人移动, 机器人位置为地图中心
  // if we're using a rolling buffer costmap...
  // we need to update the origin using the robot's position
  if (rolling_window_) {
    double new_origin_x = robot_x - combined_costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - combined_costmap_.getSizeInMetersY() / 2;
    primary_costmap_.updateOrigin(new_origin_x, new_origin_y);
    combined_costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  // 检查机器人是否出界
  if (isOutofBounds(robot_x, robot_y)) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_costmap_2d"),
      "Robot is out of bounds of the costmap!");
  }

  // 如果没有 plugins 和 filters 就没必要更新
  if (plugins_.size() == 0 && filters_.size() == 0) {
    return;
  }

  // 获取 double 的下界和上界
  // 获取所有感知范围的外接矩形框 bbox
  minx_ = miny_ = std::numeric_limits<double>::max();
  maxx_ = maxy_ = std::numeric_limits<double>::lowest();

  // 遍历所有 plugin
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    // 保存更新前的 bbox
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    // 更新地图和 bbox
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    // 当前的 min 应该小于等于之前的 min, 当前的 max 应该大于等于之前的 max
    // 因为 bbox 在 touch 的时候都是扩张, bbox 代表了参与过更新的区域
    // 如果有静态地图参与, 那么 bbox 所有地图的外接矩形框
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "nav2_costmap_2d"), "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy,
        minx_, miny_, maxx_, maxy_,
        (*plugin)->getName().c_str());
    }
  }
  // 遍历所有 filter
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end(); ++filter)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    // 更新
    (*filter)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "nav2_costmap_2d"), "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending filter is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy,
        minx_, miny_, maxx_, maxy_,
        (*filter)->getName().c_str());
    }
  }

  // 把 bbox 投射到地图上的坐标
  int x0, xn, y0, yn;
  combined_costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  combined_costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsY()), yn + 1);

  RCLCPP_DEBUG(
    rclcpp::get_logger(
      "nav2_costmap_2d"), "Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0) {
    return;
  }

  if (filters_.size() == 0) {
    // 如果没有 filters, 就只更新 plugins 就可以
    // If there are no filters enabled just update costmap sequentially by each plugin
    // 重新设置地图指定局域内的地图
    // 相当于每次更新地图都是全部重新更新一遍
    combined_costmap_.resetMap(x0, y0, xn, yn);
    // 遍历更新指定区域内的 cost
    for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
      plugin != plugins_.end(); ++plugin)
    {
      (*plugin)->updateCosts(combined_costmap_, x0, y0, xn, yn);
    }
  } else {
    // 如果启用了 filters
    // Costmap Filters enabled
    // 先遍历 plugin 更新指定区域的 cost, 这里是 primary costmap
    // 1. Update costmap by plugins
    primary_costmap_.resetMap(x0, y0, xn, yn);
    for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
      plugin != plugins_.end(); ++plugin)
    {
      (*plugin)->updateCosts(primary_costmap_, x0, y0, xn, yn);
    }

    // 把 primary costmap 的更新区域复制到 combined costmap
    // 2. Copy processed costmap window to a final costmap.
    // primary_costmap_ remain to be untouched for further usage by plugins.
    if (!combined_costmap_.copyWindow(primary_costmap_, x0, y0, xn, yn, x0, y0)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("nav2_costmap_2d"),
        "Can not copy costmap (%i,%i)..(%i,%i) window",
        x0, y0, xn, yn);
      throw std::runtime_error{"Can not copy costmap"};
    }

    // 最后使用 filter 对 combined costmap 做处理
    // 3. Apply filters over the plugins in order to make filters' work
    // not being considered by plugins on next updateMap() calls
    for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
      filter != filters_.end(); ++filter)
    {
      (*filter)->updateCosts(combined_costmap_, x0, y0, xn, yn);
    }
  }

  // 最后更新和保存最新的 bbox
  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
  // 确保所有的 plugins 和 filters 都是 current, 所有数据都是及时更新的
  // 实际都是检查更新频率都满足需求
  current_ = true;
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    current_ = current_ && ((*plugin)->isCurrent() || !(*plugin)->isEnabled());
  }
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end(); ++filter)
  {
    current_ = current_ && ((*filter)->isCurrent() || !(*filter)->isEnabled());
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec)
{
  footprint_ = footprint_spec;
  // 计算内接圆和外接圆的半径, 基于 footprint
  nav2_costmap_2d::calculateMinAndMaxDistances(
    footprint_spec,
    inscribed_radius_, circumscribed_radius_);

  // 遍历 plugins 和 filters, 随着 footprint 改变更新
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end();
    ++filter)
  {
    (*filter)->onFootprintChanged();
  }
}

}  // namespace nav2_costmap_2d
