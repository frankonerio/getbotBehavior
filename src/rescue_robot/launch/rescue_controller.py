
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('rescue_robot')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/ObjHandling_Domain.pddl',
                        
          'namespace': namespace
          }.items())

    # Specify the actions for robot_1
    
    # Specify the actions

    battery_charge_1_cmd = Node(
        package='rescue_robot',
        executable ='battery_charge_1_action_node',
        name ='battery_charge_1_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[
            {
                'specialized_arguments': ["p6building","robot_1"]
            }
        ])

    charger_approach_cmd = Node(
        package='rescue_robot',
        executable ='charger_approach_action_node',
        name ='charger_approach_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[
            {
                'specialized_arguments': ["p6building","robot_1"]
            }
        ])

    item_move_cmd = Node(
        package='rescue_robot',
        executable ='item_move_action_node',
        name ='item_move_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[
            {
                'specialized_arguments': ["p6building","robot_1"]
            }
        ])
    
    item_victim_drop_cmd = Node(
        package='rescue_robot',
        executable ='item_victim_drop_action_node',
        name ='item_victim_drop_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[
            {
                'specialized_arguments': ["p6building","robot_1"]
            }
        ])

    item_victim_grab_cmd = Node(
        package='rescue_robot',
        executable ='item_victim_grab_action_node',
        name ='item_victim_grab_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[
            {
                'specialized_arguments': ["p6building","robot_1"]
            }
        ])

    obj_approach_cmd = Node(
        package='rescue_robot',
        executable ='obj_approach_action_node',
        name ='obj_approach_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[
            {
                'specialized_arguments': ["p6building","robot_1"]
            }
        ])

    target_approach_cmd = Node(
        package='rescue_robot',
        executable ='target_approach_action_node',
        name ='target_approach_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[
            {
                'specialized_arguments': ["p6building","robot_1"]
            }
        ])

    victim_rescue_cmd = Node(
        package='rescue_robot',
        executable ='victim_rescue_action_node',
        name ='victim_rescue_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[
            {
                'specialized_arguments': ["p6building","robot_1"]
            }
        ])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(battery_charge_1_cmd)
    ld.add_action(charger_approach_cmd)

    ld.add_action(item_move_cmd)
    ld.add_action(item_victim_drop_cmd)

    ld.add_action(item_victim_grab_cmd)
    ld.add_action(obj_approach_cmd)

    ld.add_action(target_approach_cmd)
    ld.add_action(victim_rescue_cmd)
 
    return ld
