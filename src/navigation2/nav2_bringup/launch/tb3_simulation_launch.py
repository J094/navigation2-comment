# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # 获取 bringup 包的文件夹, 然后找到 launch 的文件夹
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 创建一些 launch 用到的配置变量
    # Create the launch configuration variables
    # 指定使用的 slam 算法
    slam = LaunchConfiguration('slam')
    # 设置 ros 节点的命名空间
    # 使用命名空间可以更好地进行节点隔离和管理, 特别是在多机器人系统中
    namespace = LaunchConfiguration('namespace')
    # 是否使用命名空间
    use_namespace = LaunchConfiguration('use_namespace')
    # 加载地图的 yaml 文件路径
    map_yaml_file = LaunchConfiguration('map')
    # 是否使用仿真时间
    # 使用仿真时间可以将节点的时间同步到仿真器或记录的时间, 而不是实际的系统时间
    use_sim_time = LaunchConfiguration('use_sim_time')
    # 导航系统使用的参数文件的路径
    # 该参数文件用于配置各种参数, 如传感器参数, 导航算法参数
    params_file = LaunchConfiguration('params_file')
    # 是否自动启动导航系统
    autostart = LaunchConfiguration('autostart')
    # 是否使用组合启动方式
    # True: 使用组合 composition 启动方式, 将导航系统的节点组合成一个整体启动
    # False: 单独启动每个节点
    use_composition = LaunchConfiguration('use_composition')
    # 是否使用重启策略
    # 节点退出或崩溃后会重新启动节点的策略, 可以提高系统的鲁棒性和可靠性, 确保节点在出现故障时自动恢复
    use_respawn = LaunchConfiguration('use_respawn')

    # 针对于模拟场景的 launch 配置变量
    # Launch configuration variables specific to simulation
    # 指定要在 rviz 中加载的配置文件路径
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    # 是否使用仿真器
    # True: 表示机器人导航系统在仿真环境中运行
    # False: 表示在真实机器人上云霄
    use_simulator = LaunchConfiguration('use_simulator')
    # 是否发布机器人状态
    # True: 导航系统发布机器人状态信息
    # False: 不发布机器人状态信息
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    # 是否使用 rviz 可视化
    # True: 导航系统启动 rviz 可视化
    # False: 不用 rviz 可视化
    use_rviz = LaunchConfiguration('use_rviz')
    # 是否以无头的形式启动导航系统
    # True: 导航系统将在无界面显示的情况下运行, 适用于没有物理显示设备的机器人或远程操作场景
    # False: 常规模式启动, 可以在有显示器的环境中进行交互和调试
    headless = LaunchConfiguration('headless')
    # 指定要加载世界文件的路径
    # 世界文件定义了机器人导航环境的物理结构, 障碍物, 地形等信息
    world = LaunchConfiguration('world')
    # 包含机器人初始姿态信息的字典
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    # 机器人的名称
    robot_name = LaunchConfiguration('robot_name')
    # 机器人的 sdf 文件
    # sdf 文件是一种机器人模型的描述文件格式, 其中包含了机器人的几何形状, 连接关系, 传感器信息等
    robot_sdf = LaunchConfiguration('robot_sdf')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # 设置 topic 的 remappings
    # 这里 "/tf" 为绝对话题, 而实际在不同的导航系统中使用则需要命名空间隔离, 所以将绝对话题重映射为了相对话题 "tf"
    # 这里相对话题名会在前缀增加默认的命名空间 "/{namespace}/tf"
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # 上面使用 LaunchConfiguration 引用一系列启动配置参数
    # 下面使用 DeclareLaunchArgument 来什么这一系列启动配置参数
    # 这种分离的设计方式提供了更好的灵活性和可配置性
    # 通常使用 DeclareLaunchArgument 来声明启动参数
    # 然后再需要的地方使用 LaunchConfiguration 来引用参数值
    # Declare the launch arguments
    # 默认命名空间为空
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    # 默认不使用命名空间
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    # 默认不使用 slam
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    # 默认地图文件为 {bringup_dir}/maps/turtlebot3_world.yaml
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    # 默认使用仿真时间
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # 默认参数为 {bringup_dir}/params/nav2_params.yaml
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # 默认自动启动导航系统
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # 默认使用组合启动
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    # 默认不使用重启策略
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    # rviz 配置文件默认为 {bringup_dir}/rviz/nav2_default_view.rviz
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    # 默认使用模拟器
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    # 默认发布机器人状态
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    # 默认使用 rviz 可视化
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # 默认采用无界面的模式, 意味着 gazebo 没有可视化界面
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gzclient)')

    # 默认世界文件为 {bringup_dir}/worlds/world_only.model
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        # worlds/turtlebot3_worlds/waffle.model')
        default_value=os.path.join(bringup_dir, 'worlds', 'world_only.model'),
        description='Full path to world model file to load')

    # 默认的机器人名称为 turtlebot3_waffle
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle',
        description='name of the robot')

    # 默认机器人的 sdf 为 {bringup_dir}/worlds/waffle.model
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'worlds', 'waffle.model'),
        description='Full path to robot sdf file to spawn the robot in gazebo')

    # ExecuteProcess 执行外部的命令, 类似 shell 脚本执行
    # condition 定义了一些执行条件
    # 如下方 IfCondition(use_simulator) 表示只有 use_simulator 为 true 才执行
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='screen')

    # 这里表示只哟 headless 为 false 时才会启用 gzclient 可视化
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    # 这里是机器人 urdf 模型参数 {bringup_dir}/urdf/turtlebot3_waffle.urdf
    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    # robot_description 包含了 urdf 文件的内容, 这是一个字符串变量, 就是文件中的所有内容
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # Node 运行节点
    # 通过 executable 指定可执行文件
    # 通过 arguments=[..] 按顺序给出执行文件后续跟着的参数, 这个是直接在可执行文件中用的
    # 通过 parameters=[{..}] 给一个字典来传入节点所需要的参数, 这个是 ros 参数服务器管理的
    # 通过 remappings=[(..), (..)] 给一个列表来传入重映射规则
    # 这里只有在 use_robot_state_pub 时才会启用
    # robot_state_publisher 来发布机器人状态信息
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)

    # gazebo_ros 中的 spawn_entity.py 用于在 gazebo 世界中放置模型
    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    # PythonLaunchDescriptionSource 指定要包含的启动文件的源, 它接受一个启动文件的路径
    # 这里默认的位置为 {launch_dir}/rviz_launch.py
    # IncludeLaunchDescription 用于引用和包含其他启动文件
    # 可以定义 condition 来决定是否引用
    # 可以定义 launch_arguments 来给包含的 launch 文件传入参数, 这里给的是一个字典, items() 返回每一个键值对
    # 也就是 [[key1: value1], [key2: value2], ..]
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())

    # 这里创建 launch description 描述, 然后将各种执行命令顺序放入
    # Create the launch description and populate
    ld = LaunchDescription()

    # 首先是声明相关命令
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # 然后是 ExcecuteProcess 和 Node 相关命令
    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    # 最后是 Node 和 IncludeLaunchDescription 相关命令
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    # 返回 launch description 最终执行该 launch 文件
    return ld
