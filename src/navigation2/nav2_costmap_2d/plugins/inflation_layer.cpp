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
#include "nav2_costmap_2d/inflation_layer.hpp"

#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_events_filter.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::InflationLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

InflationLayer::InflationLayer()
: inflation_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  inflate_unknown_(false),
  inflate_around_unknown_(false),
  cell_inflation_radius_(0),
  cached_cell_inflation_radius_(0),
  resolution_(0),
  cache_length_(0),
  // 初始化上界和下界
  last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
{
  // 创建新的互斥锁
  access_ = new mutex_t();
}

InflationLayer::~InflationLayer()
{
  dyn_params_handler_.reset();
  delete access_;
}

void
InflationLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("inflation_radius", rclcpp::ParameterValue(0.55));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(10.0));
  declareParameter("inflate_unknown", rclcpp::ParameterValue(false));
  declareParameter("inflate_around_unknown", rclcpp::ParameterValue(false));

  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);
    node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor_);
    node->get_parameter(name_ + "." + "inflate_unknown", inflate_unknown_);
    node->get_parameter(name_ + "." + "inflate_around_unknown", inflate_around_unknown_);

    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(
        &InflationLayer::dynamicParametersCallback,
        this, std::placeholders::_1));
  }

  current_ = true;
  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_reinflation_ = false;
  // 计算地图上的膨胀半径
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  matchSize();
}

void
InflationLayer::matchSize()
{
  // 拿 costmap 的时候不希望被修改
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // 拿到 costmap
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  // 获取分辨率 m/cell
  resolution_ = costmap->getResolution();
  // 计算地图上的膨胀半径
  // TODO: 在这之前会有重复计算?
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  // 计算缓存, 也就是计算了某个点周围膨胀的范围和值
  computeCaches();
  // 表示是否遍历过了, 其 size 就是所有 cells
  seen_ = std::vector<bool>(costmap->getSizeInCellsX() * costmap->getSizeInCellsY(), false);
}

void
InflationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (need_reinflation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    // 如果重新膨胀, 这里 min 和 max 取下界和上界
    // 这是为了保证任何位置都可以被膨胀影响, 后续更新 cost 的时候不会超出地图边界
    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_reinflation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // 如果不需要重新膨胀, 这里更新 min 和 max 考虑膨胀半径
    // 因为膨胀过后只会比正常边界大膨胀半径, 所以只考虑膨胀半径即可
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void
InflationLayer::onFootprintChanged()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // TODO: 为什么要重新膨胀? 这里最小接触半径对于膨胀半径没有影响
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  // 获得膨胀半径
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  // 根据新的膨胀半径计算距离矩阵等
  computeCaches();
  // 由于轮廓改变, 需要重新膨胀
  need_reinflation_ = true;

  RCLCPP_DEBUG(
    logger_, "InflationLayer::onFootprintChanged(): num footprint points: %zu,"
    " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
    layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

void
InflationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_ || (cell_inflation_radius_ == 0)) {
    return;
  }

  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  for (auto & dist : inflation_cells_) {
    RCLCPP_FATAL_EXPRESSION(
      logger_,
      !dist.empty(), "The inflation list must be empty at the beginning of inflation");
  }

  // 拿到 char * 的地图
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (seen_.size() != size_x * size_y) {
    // 一定确保所有地图点都被考虑在 seen_ 中
    RCLCPP_WARN(
      logger_, "InflationLayer::updateCosts(): seen_ vector size is wrong");
    seen_ = std::vector<bool>(size_x * size_y, false);
  }

  // 首先初始化, 所有地图坐标位置都是没处理过的
  std::fill(begin(seen_), end(seen_), false);

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;
  // 首先需要将地图扩张膨胀半径
  min_i -= static_cast<int>(cell_inflation_radius_);
  min_j -= static_cast<int>(cell_inflation_radius_);
  max_i += static_cast<int>(cell_inflation_radius_);
  max_j += static_cast<int>(cell_inflation_radius_);

  // 不能超过地图边界
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Inflation list; we append cells to visit in a list associated with
  // its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before,
  // with a notable performance boost

  // Start with lethal obstacles: by definition distance is 0.0
  // obs_bin 就是障碍物
  auto & obs_bin = inflation_cells_[0];
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = static_cast<int>(master_grid.getIndex(i, j));
      // 遍历获取 bbox 更新区域的 cost
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE || (inflate_around_unknown_ && cost == NO_INFORMATION)) {
        // 只膨胀障碍物, 或者根据需求膨胀未知区域
        // index 为障碍物 index, i j 为障碍物的地图坐标
        obs_bin.emplace_back(index, i, j, i, j);
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the
  // corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  // 一层一层处理 cost, 从低 level 到高 level
  for (const auto & dist_bin : inflation_cells_) {
    for (std::size_t i = 0; i < dist_bin.size(); ++i) {
      // Do not use iterator or for-range based loops to
      // iterate though dist_bin, since it's size might
      // change when a new cell is enqueued, invalidating all iterators
      unsigned int index = dist_bin[i].index_;

      // ignore if already visited
      if (seen_[index]) {
        // 如果已经处理过了就跳过
        continue;
      }

      seen_[index] = true;

      unsigned int mx = dist_bin[i].x_;
      unsigned int my = dist_bin[i].y_;
      unsigned int sx = dist_bin[i].src_x_;
      unsigned int sy = dist_bin[i].src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      // 根据当前点到基点的距离, 获取 cost
      unsigned char cost = costLookup(mx, my, sx, sy);
      // 获取老的 cost
      unsigned char old_cost = master_array[index];
      // In order to avoid artifacts appeared out of boundary areas
      // when some layer is going after inflation_layer,
      // we need to apply inflation_layer only to inside of given bounds
      if (static_cast<int>(mx) >= base_min_i &&
        static_cast<int>(my) >= base_min_j &&
        static_cast<int>(mx) < base_max_i &&
        static_cast<int>(my) < base_max_j)
      {
        // 当前点应该在地图内
        if (old_cost == NO_INFORMATION &&
          (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        {
          // 如果老的没信息, \新的满足要求, 直接覆盖
          master_array[index] = cost;
        } else {
          // 如果已经有信息了, 取最大值
          master_array[index] = std::max(old_cost, cost);
        }
      }

      // 然后将该点附近的点放入, 原点保持 sx sy 不变, 取上下左右四个点
      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0) {
        enqueue(index - 1, mx - 1, my, sx, sy);
      }
      if (my > 0) {
        enqueue(index - size_x, mx, my - 1, sx, sy);
      }
      if (mx < size_x - 1) {
        enqueue(index + 1, mx + 1, my, sx, sy);
      }
      if (my < size_y - 1) {
        enqueue(index + size_x, mx, my + 1, sx, sy);
      }
      // 相当于是一层一层向外扩, 从深向浅填充, 已访问的不再访问
      // 而且这是相对于所有待扩张点来说的
    }
  }

  for (auto & dist : inflation_cells_) {
    // 这里 cost 处理完了, 清空待扩张点
    dist.clear();
    dist.reserve(200);
  }

  // 更新完了, 设置当前为最新
  current_ = true;
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
void
InflationLayer::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index]) {
    // 只处理没有处理过的
    // we compute our distance table one cell further than the
    // inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within
    // the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_) {
      // 只处理在膨胀半径内的地图坐标
      return;
    }

    const unsigned int r = cell_inflation_radius_ + 2;

    // push the cell data onto the inflation list and mark
    // 按照距离 level 加入
    inflation_cells_[distance_matrix_[mx - src_x + r][my - src_y + r]].emplace_back(
      index, mx, my, src_x, src_y);
  }
}

void
InflationLayer::computeCaches()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (cell_inflation_radius_ == 0) {
    // 如果膨胀半径为 0, 就是不膨胀
    return;
  }

  // 缓存的长度, 会根据这个计算方框中所有像素到原点的距离
  cache_length_ = cell_inflation_radius_ + 2;

  // 基于膨胀半径计算距离
  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);

    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        // 这里就是方框中素有像素到原点的距离
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }

    // 这里标记距离计算过了
    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i < cache_length_; ++i) {
    for (unsigned int j = 0; j < cache_length_; ++j) {
      // 根据距离计算花费
      cached_costs_[i * cache_length_ + j] = computeCost(cached_distances_[i * cache_length_ + j]);
    }
  }

  // 获得最大的距离, 也是就最大 level, 最大的层数
  int max_dist = generateIntegerDistances();
  // 先清空
  inflation_cells_.clear();
  // 再 resize, 层数加 1 因为圆心算一个
  inflation_cells_.resize(max_dist + 1);
  for (auto & dist : inflation_cells_) {
    // 每一层申请 200 的空间给 cells
    dist.reserve(200);
  }
}

int
InflationLayer::generateIntegerDistances()
{
  const int r = cell_inflation_radius_ + 2;
  // 2 * r + 1 就是这个圆的真正直径
  const int size = r * 2 + 1;

  std::vector<std::pair<int, int>> points;

  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      if (x * x + y * y <= r * r) {
        // 基于某点为圆心的圆囊括的坐标点
        points.emplace_back(x, y);
      }
    }
  }

  // 根据距离来排序, 升序从小到大
  std::sort(
    points.begin(), points.end(),
    [](const std::pair<int, int> & a, const std::pair<int, int> & b) -> bool {
      return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
    }
  );

  std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
  std::pair<int, int> last = {0, 0};
  int level = 0;
  for (auto const & p : points) {
    if (p.first * p.first + p.second * p.second !=
      last.first * last.first + last.second * last.second)
    {
      // 只要距离不相等, 那么 level 需要加一, 相当于向外扩了一圈
      level++;
    }
    // 然后基于左下角点为 0 0 点,  
    distance_matrix[p.first + r][p.second + r] = level;
    last = p;
  }

  // 保存距离矩阵, 距离矩阵就是左下角为坐标原点, 所有坐标距离圆中心点的距离
  distance_matrix_ = distance_matrix;
  return level;
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
InflationLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  bool need_cache_recompute = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "inflation_radius" &&
        inflation_radius_ != parameter.as_double())
      {
        inflation_radius_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
      } else if (param_name == name_ + "." + "cost_scaling_factor" && // NOLINT
        cost_scaling_factor_ != parameter.as_double())
      {
        cost_scaling_factor_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        need_reinflation_ = true;
        current_ = false;
      } else if (param_name == name_ + "." + "inflate_unknown" && // NOLINT
        inflate_unknown_ != parameter.as_bool())
      {
        inflate_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
      } else if (param_name == name_ + "." + "inflate_around_unknown" && // NOLINT
        inflate_around_unknown_ != parameter.as_bool())
      {
        inflate_around_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
      }
    }
  }

  if (need_cache_recompute) {
    matchSize();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
