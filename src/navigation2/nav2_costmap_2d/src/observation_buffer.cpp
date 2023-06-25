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
 *********************************************************************/
#include "nav2_costmap_2d/observation_buffer.hpp"

#include <algorithm>
#include <list>
#include <string>
#include <vector>
#include <chrono>

#include "tf2/convert.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
using namespace std::chrono_literals;

namespace nav2_costmap_2d
{
ObservationBuffer::ObservationBuffer(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string topic_name,
  double observation_keep_time,
  double expected_update_rate,
  double min_obstacle_height, double max_obstacle_height, double obstacle_max_range,
  double obstacle_min_range,
  double raytrace_max_range, double raytrace_min_range, tf2_ros::Buffer & tf2_buffer,
  std::string global_frame,
  std::string sensor_frame,
  tf2::Duration tf_tolerance)
: tf2_buffer_(tf2_buffer),
  observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time)),
  expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate)),
  global_frame_(global_frame),
  sensor_frame_(sensor_frame),
  topic_name_(topic_name),
  min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
  obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),
  raytrace_max_range_(raytrace_max_range), raytrace_min_range_(
    raytrace_min_range), tf_tolerance_(tf_tolerance)
{
  // 获得节点的 clock, logger
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  last_updated_ = node->now();
}

ObservationBuffer::~ObservationBuffer()
{
}

void ObservationBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  geometry_msgs::msg::PointStamped global_origin;

  // 创建一个新的观测, 这个观测是用来填充的
  // create a new observation on the list to be populated
  observation_list_.push_front(Observation());

  // 获取传感器坐标系名称
  // check whether the origin frame has been set explicitly
  // or whether we should get it from the cloud
  std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try {
    // given these observations come from sensors...
    // we'll need to store the origin pt of the sensor
    geometry_msgs::msg::PointStamped local_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = origin_frame;
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;
    // 这是获取传感器观测原点在世界坐标系的位姿
    tf2_buffer_.transform(local_origin, global_origin, global_frame_, tf_tolerance_);
    // 将原点给到新建的观测
    tf2::convert(global_origin.point, observation_list_.front().origin_);

    // 参数给到新建的观测
    // make sure to pass on the raytrace/obstacle range
    // of the observation buffer to the observations
    observation_list_.front().raytrace_max_range_ = raytrace_max_range_;
    observation_list_.front().raytrace_min_range_ = raytrace_min_range_;
    observation_list_.front().obstacle_max_range_ = obstacle_max_range_;
    observation_list_.front().obstacle_min_range_ = obstacle_min_range_;

    // 创建一个全局点云
    sensor_msgs::msg::PointCloud2 global_frame_cloud;

    // 把这个点云转换到世界坐标系中
    // transform the point cloud
    tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // 把点云给新建的观测
    // now we need to remove observations from the cloud that are below
    // or above our height thresholds
    sensor_msgs::msg::PointCloud2 & observation_cloud = *(observation_list_.front().cloud_);
    observation_cloud.height = global_frame_cloud.height;
    observation_cloud.width = global_frame_cloud.width;
    observation_cloud.fields = global_frame_cloud.fields;
    observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    observation_cloud.point_step = global_frame_cloud.point_step;
    observation_cloud.row_step = global_frame_cloud.row_step;
    observation_cloud.is_dense = global_frame_cloud.is_dense;

    // 获取到点云的尺寸
    unsigned int cloud_size = global_frame_cloud.height * global_frame_cloud.width;
    sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
    modifier.resize(cloud_size);
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    // 首先按照高度过滤, 获取合适高度的数据, 这里是遍历 z 轴的数据
    sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
    // 获得的点云数据的 iterator
    std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(),
      iter_global_end = global_frame_cloud.data.end();
    // 新观测数据的 iterator
    std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
    // 遍历获得的点云数据
    for (; iter_global != iter_global_end; ++iter_z, iter_global +=
      global_frame_cloud.point_step)
    {
      if ((*iter_z) <= max_obstacle_height_ &&
        (*iter_z) >= min_obstacle_height_)
      {
        // 这里是把合适的数据复制到新观测点云数据中
        std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
        iter_obs += global_frame_cloud.point_step;
        ++point_count;
      }
    }

    // resize the cloud for the number of legal points
    // 根据筛选过的点修改点云尺寸
    modifier.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  } catch (tf2::TransformException & ex) {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    RCLCPP_ERROR(
      logger_,
      "TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s",
      sensor_frame_.c_str(),
      cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // 记录更新时间
  // if the update was successful, we want to update the last updated time
  last_updated_ = clock_->now();

  // 最后把陈旧的观测删掉
  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

// 返回观测的复制
// returns a copy of the observations
void ObservationBuffer::getObservations(std::vector<Observation> & observations)
{
  // 确保没有过时的观测
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  std::list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
    // 就是把观测列表里的一个个放到观测里
    observations.push_back(*obs_it);
  }
}

void ObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty()) {
    std::list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == rclcpp::Duration(0.0s)) {
      // 如果我们保存观测的时间为 0, 那么我们只保留最后一个观测
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
      Observation & obs = *obs_it;
      // check if the observation is out of date... and if it is,
      // remove it and those that follow from the list
      if ((clock_->now() - obs.cloud_->header.stamp) >
        observation_keep_time_)
      {
        // 检查观测保留时间过长的就删掉
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::isCurrent() const
{
  if (expected_update_rate_ == rclcpp::Duration(0.0s)) {
    return true;
  }

  // 确保按照预期频率更新
  bool current = (clock_->now() - last_updated_) <=
    expected_update_rate_;
  if (!current) {
    RCLCPP_WARN(
      logger_,
      "The %s observation buffer has not been updated for %.2f seconds, "
      "and it should be updated every %.2f seconds.",
      topic_name_.c_str(),
      (clock_->now() - last_updated_).seconds(),
      expected_update_rate_.seconds());
  }
  return current;
}

void ObservationBuffer::resetLastUpdated()
{
  last_updated_ = clock_->now();
}
}  // namespace nav2_costmap_2d
