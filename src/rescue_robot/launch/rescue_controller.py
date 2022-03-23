
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
          'model_file': example_dir + '/pddl/Domain.pddl',
          'namespace': namespace
          }.items())


    # Specify the actions for robot_1
    
    # Specify the actions

    ##########################################
    #           BATTERY NODES                #
    ##########################################

    battery_charge_1_cmd = Node(
        package='rescue_robot',
        executable ='battery_charge_1_action_node',
        name ='battery_charge_1_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    charger_approach_cmd = Node(
        package='rescue_robot',
        executable ='charger_approach_action_node',
        name ='charger_approach_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    ##########################################
    #      OBJECT HANDLING NODES             #
    ##########################################

    item_move_cmd = Node(
        package='rescue_robot',
        executable ='item_move_action_node',
        name ='item_move_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])
    
    item_victim_drop_cmd = Node(
        package='rescue_robot',
        executable ='item_victim_drop_action_node',
        name ='item_victim_drop_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    item_victim_grab_cmd = Node(
        package='rescue_robot',
        executable ='item_victim_grab_action_node',
        name ='item_victim_grab_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    obj_approach_cmd = Node(
        package='rescue_robot',
        executable ='obj_approach_action_node',
        name ='obj_approach_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    target_approach_cmd = Node(
        package='rescue_robot',
        executable ='target_approach_action_node',
        name ='target_approach_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    victim_rescue_cmd = Node(
        package='rescue_robot',
        executable ='victim_rescue_action_node',
        name ='victim_rescue_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    ##########################################
    #           BACKGROUND NODES             #
    ##########################################

    explore_start_cmd = Node(
        package='rescue_robot',
        executable ='explore_start_action_node',
        name ='explore_start_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    map_start_cmd = Node(
        package='rescue_robot',
        executable ='map_start_action_node',
        name ='map_start_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    wander_around_cmd = Node(
        package='rescue_robot',
        executable ='wander_around_action_node',
        name ='wander_around_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])


    obj_detect_start_cmd = Node(
        package='rescue_robot',
        executable ='obj_detect_start_action_node',
        name ='obj_detect_start_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    ##########################################
    #           NAVIGATION NODES             #
    ##########################################

    elevator_approach_cmd = Node(
        package='rescue_robot',
        executable ='elevator_approach_action_node',
        name ='elevator_approach_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])


    stairs_approach_cmd = Node(
        package='rescue_robot',
        executable ='stairs_approach_action_node',
        name ='stairs_approach_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    elevator_up_cmd = Node(
        package='rescue_robot',
        executable ='elevator_up_action_node',
        name ='elevator_up_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    elevator_down_cmd = Node(
        package='rescue_robot',
        executable ='elevator_down_action_node',
        name ='elevator_down_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    stairs_up_cmd = Node(
        package='rescue_robot',
        executable ='stairs_up_action_node',
        name ='stairs_up_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    stairs_down_cmd = Node(
        package='rescue_robot',
        executable ='stairs_down_action_node',
        name ='stairs_down_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])
    
    door_open_cmd = Node(
        package='rescue_robot',
        executable ='door_open_action_node',
        name ='door_open_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])
    
    door_destination_navigate_cmd = Node(
        package='rescue_robot',
        executable ='door_destination_navigate_node',
        name ='door_destination_navigate_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    destination_navigate_cmd = Node(
        package='rescue_robot',
        executable ='destination_navigate_node',
        name ='destination_navigate_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ##########################################
    #           BATTERY NODES                #
    ##########################################

    ld.add_action(battery_charge_1_cmd)
    ld.add_action(charger_approach_cmd)

    ld.add_action(item_move_cmd)
    ld.add_action(item_victim_drop_cmd)

    ##########################################
    #           BACKGROUND NODES             #
    ##########################################

    ld.add_action(map_start_cmd)
    ld.add_action(wander_around_cmd)

    ld.add_action(obj_detect_start_cmd)
    
    ##########################################
    #      OBJECT HANDLING NODES             #
    ##########################################

    ld.add_action(item_victim_grab_cmd)
    ld.add_action(explore_start_cmd)

    ld.add_action(obj_approach_cmd)
    ld.add_action(target_approach_cmd)

    ld.add_action(victim_rescue_cmd)

    ##########################################
    #           NAVIGATION NODES             #
    ##########################################

    ld.add_action(elevator_approach_cmd)
    ld.add_action(stairs_approach_cmd)
    ld.add_action(elevator_up_cmd)
    ld.add_action(elevator_down_cmd)
    ld.add_action(stairs_up_cmd)
    ld.add_action(stairs_down_cmd)
    ld.add_action(door_open_cmd)
    ld.add_action(door_destination_navigate_cmd)
    ld.add_action(destination_navigate_cmd)
    
    
 
    return ld
