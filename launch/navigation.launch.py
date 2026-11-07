import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from icir_navigation.occupancy_map import cache_dir

PKG = 'icir_navigation'

NAV2_SERVERS = [
    ('nav2_controller', 'controller_server'),
    ('nav2_smoother', 'smoother_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
]


def namespaced_params(src, ns):
    with open(src) as f:
        data = yaml.safe_load(f.read().replace('<ns>', '/' + ns))
    out = os.path.join(cache_dir(), '%s_%s' % (ns, os.path.basename(src)))
    with open(out, 'w') as f:
        yaml.safe_dump({ns: data}, f, sort_keys=False)
    return out


def _setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    mapfile = LaunchConfiguration('map').perform(context)
    slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    initial = LaunchConfiguration('initial_pose').perform(context).strip()

    share = get_package_share_directory(PKG)
    params = namespaced_params(LaunchConfiguration('params_file').perform(context), ns)

    nodes = []
    lifecycle = []

    if slam:
        nodes.append(
            Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                 name='slam_toolbox', namespace=ns, output='screen',
                 parameters=[namespaced_params(
                     LaunchConfiguration('slam_params_file').perform(context), ns)]))
    else:
        if not mapfile:
            raise RuntimeError('map:=<yaml> is required unless slam:=true')
        if not os.path.isfile(mapfile):
            raise RuntimeError('map does not exist: %s' % mapfile)
        amcl_params = [params]
        if initial:
            x, y, yaw = (float(v) for v in initial.split(','))
            amcl_params.append({'initial_pose': {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}})
        nodes += [
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 namespace=ns, output='screen',
                 parameters=[params, {'yaml_filename': mapfile}]),
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 namespace=ns, output='screen', parameters=amcl_params),
        ]
        lifecycle += ['map_server', 'amcl']

    for pkg, exe in NAV2_SERVERS:
        nodes.append(Node(package=pkg, executable=exe, name=exe, namespace=ns,
                          output='screen', parameters=[params]))
        lifecycle.append(exe)

    nodes.append(
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=ns, output='screen',
             parameters=[{'use_sim_time': True,
                          'autostart': autostart,
                          'bond_timeout': 10.0,
                          'node_names': lifecycle}]))

    if rviz:
        nodes.append(
            Node(package='rviz2', executable='rviz2', name='rviz2', namespace=ns,
                 output='screen',
                 arguments=['-d', os.path.join(share, 'rviz', 'navigation.rviz')],
                 parameters=[{'use_sim_time': True}]))

    return nodes


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='ns1',
                              description='Nav2 namespace (must match controller robot_group)'),
        DeclareLaunchArgument('map', default_value='',
                              description='Map yaml (required if slam:=false)'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Use slam_toolbox instead of map_server + AMCL'),
        DeclareLaunchArgument('initial_pose', default_value='0,0,0',
                              description='AMCL initial pose "X,Y,YAW" in map frame'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(share, 'config', 'slam.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        OpaqueFunction(function=_setup),
    ])
