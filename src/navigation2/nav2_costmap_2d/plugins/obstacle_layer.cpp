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
 *         Steve Macenski
 *********************************************************************/
#include "nav2_costmap_2d/obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

ObstacleLayer::~ObstacleLayer()
{
  dyn_params_handler_.reset();
  for (auto & notifier : observation_notifiers_) {
    notifier.reset();
  }
}

void ObstacleLayer::onInitialize()
{
  bool track_unknown_space;
  double transform_tolerance;

  // The topics that we'll subscribe to from the parameter server
  std::string topics_string;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.0));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("transform_tolerance", transform_tolerance);
  node->get_parameter(name_ + "." + "observation_sources", topics_string);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ObstacleLayer::dynamicParametersCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Subscribed to Topics: %s", topics_string.c_str());

  // 是否是 rolling
  rolling_window_ = layered_costmap_->isRolling();

  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  // match layered_map 的尺寸
  ObstacleLayer::matchSize();
  current_ = true;
  was_reset_ = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  // topics 拆分成每一个 topic
  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source) {
    // 每一次都拿一个 topic 给 source
    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
    declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
    declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "data_type", rclcpp::ParameterValue(std::string("LaserScan")));
    declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
    declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
    declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "raytrace_max_range", rclcpp::ParameterValue(3.0));
    declareParameter(source + "." + "raytrace_min_range", rclcpp::ParameterValue(0.0));

    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
    node->get_parameter(
      name_ + "." + source + "." + "observation_persistence",
      observation_keep_time);
    node->get_parameter(
      name_ + "." + source + "." + "expected_update_rate",
      expected_update_rate);
    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
    node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
    // 对于每一个 topic 会有这两个标志, 影响该观测是属于增加障碍物还是清除障碍物
    node->get_parameter(name_ + "." + source + "." + "marking", marking);
    node->get_parameter(name_ + "." + source + "." + "clearing", clearing);

    if (!(data_type == "PointCloud2" || data_type == "LaserScan")) {
      RCLCPP_FATAL(
        logger_,
        "Only topics that use point cloud2s or laser scans are currently supported");
      throw std::runtime_error(
              "Only topics that use point cloud2s or laser scans are currently supported");
    }

    // get the obstacle range for the sensor
    double obstacle_max_range, obstacle_min_range;
    node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", obstacle_max_range);
    node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", obstacle_min_range);

    // get the raytrace ranges for the sensor
    double raytrace_max_range, raytrace_min_range;
    node->get_parameter(name_ + "." + source + "." + "raytrace_min_range", raytrace_min_range);
    node->get_parameter(name_ + "." + source + "." + "raytrace_max_range", raytrace_max_range);


    RCLCPP_DEBUG(
      logger_,
      "Creating an observation buffer for source %s, topic %s, frame %s",
      source.c_str(), topic.c_str(),
      sensor_frame.c_str());

    // 创建观测的 buffer, 相当于每一个传感器就有一个观测缓存
    // create an observation buffer
    observation_buffers_.push_back(
      std::shared_ptr<ObservationBuffer
      >(
        new ObservationBuffer(
          node, topic, observation_keep_time, expected_update_rate,
          min_obstacle_height,
          max_obstacle_height, obstacle_max_range, obstacle_min_range, raytrace_max_range,
          raytrace_min_range, *tf_,
          global_frame_,
          sensor_frame, tf2::durationFromSec(transform_tolerance))));

    // 用于标记障碍物的缓冲区, 机器人通过传感器探测到障碍物时, 这些观测数据会被添加到这里
    // 标记障碍物的位置和属性, 以便进行路径规划和避障
    // 增加当前检测到的障碍
    // check if we'll add this buffer to our marking observation buffers
    if (marking) {
      marking_buffers_.push_back(observation_buffers_.back());
    }

    // 用于清除障碍物的缓冲区, 机器人检测到环境中没有障碍物时, 这些观测数据会被添加到这里
    // 这些观测数据通常用于清除先前标记的障碍物, 及时更新反应环境的变化
    // 删除之前存在, 但是当前没检测到的障碍
    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing) {
      clearing_buffers_.push_back(observation_buffers_.back());
    }

    RCLCPP_DEBUG(
      logger_,
      "Created an observation buffer for source %s, topic %s, global frame: %s, "
      "expected update rate: %.2f, observation persistence: %.2f",
      source.c_str(), topic.c_str(),
      global_frame_.c_str(), expected_update_rate, observation_keep_time);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    // create a callback for the topic
    if (data_type == "LaserScan") {
      // 创建订阅, 订阅传感器消息
      auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
          rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
      // 取消订阅, 暂时停止接收消息
      sub->unsubscribe();

      // 定义 message filter
      // 这里是通过 tf 坐标转换来过滤消息
      // 队列大小为 50
      auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *sub, *tf_, global_frame_, 50,
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        tf2::durationFromSec(transform_tolerance));

      if (inf_is_valid) {
        // 如果 inf 是可用的
        filter->registerCallback(
          std::bind(
            &ObstacleLayer::laserScanValidInfCallback, this, std::placeholders::_1,
            observation_buffers_.back()));

      } else {
        // 只有非 inf 值
        filter->registerCallback(
          std::bind(
            &ObstacleLayer::laserScanCallback, this, std::placeholders::_1,
            observation_buffers_.back()));
      }

      // 专门保存观测的订阅
      observation_subscribers_.push_back(sub);

      // 使用 message_filter 来做 notifier
      observation_notifiers_.push_back(filter);
      // 设置消息同步容忍时间为 0.05s, 相当于 20hz, 如果 0.05s 内两个消息没有同步上就丢弃掉
      observation_notifiers_.back()->setTolerance(rclcpp::Duration::from_seconds(0.05));

    } else {
      auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
          rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
      sub->unsubscribe();

      if (inf_is_valid) {
        RCLCPP_WARN(
          logger_,
          "obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *sub, *tf_, global_frame_, 50,
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        tf2::durationFromSec(transform_tolerance));

      filter->registerCallback(
        std::bind(
          &ObstacleLayer::pointCloud2Callback, this, std::placeholders::_1,
          observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "") {
      // TODO: 这里设置的是什么?
      // 如果定义了 sensor_frame, 就需要设置下
      std::vector<std::string> target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }
}

rcl_interfaces::msg::SetParametersResult
ObstacleLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "min_obstacle_height") {
        min_obstacle_height_ = parameter.as_double();
      } else if (param_name == name_ + "." + "max_obstacle_height") {
        max_obstacle_height_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "combination_method") {
        combination_method_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

void
ObstacleLayer::laserScanCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr message,
  const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
{
  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  try {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
      global_frame_.c_str(),
      ex.what());
    projector_.projectLaser(*message, cloud);
  } catch (std::runtime_error & ex) {
    RCLCPP_WARN(
      logger_,
      "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
      " Ignore this message. what(): %s",
      ex.what());
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void
ObstacleLayer::laserScanValidInfCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
  const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
{
  // 把 inf 转换成最远处的距离
  // Filter positive infinities ("Inf"s) to max_range.
  // 0.1 毫米
  float epsilon = 0.0001;  // a tenth of a millimeter
  // 将测量数据拿出来给 message
  sensor_msgs::msg::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    // 获取消息的距离
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0) {
      // 只处理无穷的和距离大于 0 的
      // 并且还要减去 epsilon
      // 无穷的距离都变成最大测量距离
      // TODO: 为啥减去 epsilon
      // epsilon 对距离值进行微调和修正, 在后续处理中更准确地处理该距离值
      message.ranges[i] = message.range_max - epsilon;
    }
  }

  // 将激光扫描投影成点云
  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try {
    // 转换成点云
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
      global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  } catch (std::runtime_error & ex) {
    RCLCPP_WARN(
      logger_,
      "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
      " Ignore this message. what(): %s",
      ex.what());
    return;
  }

  // buffer the point cloud
  buffer->lock();
  // 把这次获的点云放到 buffer 里
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void
ObstacleLayer::pointCloud2Callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
  const std::shared_ptr<ObservationBuffer> & buffer)
{
  // buffer the point cloud
  buffer->lock();
  // 如果获得的是点云, 直接缓存就好
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void
ObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  // NOTE: 互斥锁管理工具, 会自动释放锁
  // 因为是对地图边界做修改, 需要锁住当前地图, 保证线程安全
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_) {
    // 如果本地图是 rolling 的, 那么就需要更新原点
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // 获取标记的观测, 包括静态和动态两种, 动态在前面, 静态在后面
  // get the marking observations
  current = current && getMarkingObservations(observations);

  // 获取清除障碍物的观测
  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // 如果都更新了, 那么就是当前的
  // update the global current status
  current_ = current;

  // 这是自由区域
  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    // 根据激光来清理自由区域, 同时更新了 bbox
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // 这是障碍区域
  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin();
    it != observations.end(); ++it)
  {
    const Observation & obs = *it;

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    // 平方, 后续距离就不需要开根号计算, 用于判断点是否超过区域
    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    // 遍历 x y z 所有点云
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too low, we won't add it
      if (pz < min_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too low");
        continue;
      }

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too high");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      // 计算距离, 3D 距离, 自带平方
      double sq_dist =
        (px -
        obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) +
        (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_max_range) {
        RCLCPP_DEBUG(logger_, "The point is too far away");
        continue;
      }

      // if the point is too close, do not conisder it
      if (sq_dist < sq_obstacle_min_range) {
        RCLCPP_DEBUG(logger_, "The point is too close");
        continue;
      }

      // 接下来才是我们要考虑的障碍点
      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      // 转换到地图上
      if (!worldToMap(px, py, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      // 获取 index, 也就是 data 一维上的 index
      unsigned int index = getIndex(mx, my);
      // 所有都设置为致命障碍
      costmap_[index] = LETHAL_OBSTACLE;
      // 更新 bbox, 需要覆盖障碍物点
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  // 最后更新轮廓, 并且利用轮廓更新 bbox
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
ObstacleLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  // 更新轮廓位置
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  // 根据轮廓更新 bbox
  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
ObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }

  // 如果要清除 footprint, 就把其包含区域设为 FREE_SPACE
  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  // 根据不同的模式更新
  switch (combination_method_) {
    case 0:  // Overwrite
      // 有信息的覆盖, 正常应该是这一个模式
      // 创建新的障碍和清除不存在的障碍
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      // 这种模式应该不会删除不存在的障碍物, 因为原本地图如果有障碍物, 那么应该是大于 0 FREE_SPACE 的
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void
ObstacleLayer::addStaticObservation(
  nav2_costmap_2d::Observation & obs,
  bool marking, bool clearing)
{
  if (marking) {
    static_marking_observations_.push_back(obs);
  }
  if (clearing) {
    static_clearing_observations_.push_back(obs);
  }
}

void
ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking) {
    static_marking_observations_.clear();
  }
  if (clearing) {
    static_clearing_observations_.clear();
  }
}

bool
ObstacleLayer::getMarkingObservations(std::vector<Observation> & marking_observations) const
{
  bool current = true;
  // get the marking observations
  // 遍历 buffer 中所有的观测, 加入到 marking_observations 中
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->lock();
    // 获取标记观测的每一个缓存
    marking_buffers_[i]->getObservations(marking_observations);
    // 需要确保该缓存是及时更新的
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  // 最后再把静态的观测加入
  marking_observations.insert(
    marking_observations.end(),
    static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool
ObstacleLayer::getClearingObservations(std::vector<Observation> & clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  // 遍历获取用于清除的观测
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  // 同样需要将静态清除的观测也加入进去
  clearing_observations.insert(
    clearing_observations.end(),
    static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void
ObstacleLayer::raytraceFreespace(
  const Observation & clearing_observation, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  // 现获取观测原点
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  const sensor_msgs::msg::PointCloud2 & cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  // 将观测原点转换到地图上
  if (!worldToMap(ox, oy, x0, y0)) {
    RCLCPP_WARN(
      logger_,
      "Sensor origin at (%.2f, %.2f) is out of map bounds (%.2f, %.2f) to (%.2f, %.2f). "
      "The costmap cannot raytrace for it.",
      ox, oy,
      origin_x_, origin_y_,
      origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY());
    return;
  }

  // 这里预先计算地图的边界点
  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;


  // 获取包围 ox oy 的最小 bbox, 先是利用起点更新 bbox
  touch(ox, oy, min_x, min_y, max_x, max_y);

  // 对于点云上的每一个点, 我们都希望将测量原点到该点的连线上的障碍物清除, 因为当前没有测量到障碍物
  // for each point in the cloud, we want to trace a line from the origin
  // and clear obstacles along it
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    double wx = *iter_x;
    double wy = *iter_y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // 如果 wx wy 有超出地图边界的, 计算射线在地图边界上的点
    // the minimum value to raytrace from is the origin
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // 如果 wx wy 有超出地图边界的, 计算地图边界点
    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // 把 wx xy 转换到地图上
    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1)) {
      continue;
    }

    // 清理空间的距离, 这是计算地图上的
    unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
    unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);
    // 定义 marker 为 costmap_ 上的 FREE_SPACE
    MarkCell marker(costmap_, FREE_SPACE);
    // 最终在这里处理清除射线上的障碍, 这里会对所有线上的格子赋值为 FREE_SPACE
    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_max_range, cell_raytrace_min_range);

    // 再用实际的清除情况和终点, 更新 bbox
    updateRaytraceBounds(
      ox, oy, wx, wy, clearing_observation.raytrace_max_range_,
      clearing_observation.raytrace_min_range_, min_x, min_y, max_x,
      max_y);
  }
}

void
ObstacleLayer::activate()
{
  for (auto & notifier : observation_notifiers_) {
    notifier->clear();
  }

  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->subscribe();
    }
  }
  resetBuffersLastUpdated();
}

void
ObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->unsubscribe();
    }
  }
}

void
ObstacleLayer::updateRaytraceBounds(
  double ox, double oy, double wx, double wy, double max_range, double min_range,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  if (full_distance < min_range) {
    // 如果距离小于最小距离, 就不做更新了
    return;
  }
  // 缩放获得终点, 然后更新地图边界
  double scale = std::min(1.0, max_range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void
ObstacleLayer::reset()
{
  resetMaps();
  resetBuffersLastUpdated();
  current_ = false;
  was_reset_ = true;
}

void
ObstacleLayer::resetBuffersLastUpdated()
{
  for (unsigned int i = 0; i < observation_buffers_.size(); ++i) {
    if (observation_buffers_[i]) {
      observation_buffers_[i]->resetLastUpdated();
    }
  }
}

}  // namespace nav2_costmap_2d
