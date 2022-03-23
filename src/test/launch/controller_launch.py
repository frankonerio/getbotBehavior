
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('controller')
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
          'model_file': example_dir + '/pddl/domain_1.pddl:' +
                        example_dir + '/pddl/domain_2.pddl:' +
                        example_dir + '/pddl/domain_objHandling.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions for robot_1
    
    # Specify the actions
    move_cmd = Node(
        package='controller',
        executable ='move_action_node',
        name ='move_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    approach_items_cmd = Node(
        package = 'controller',
        executable = 'approach_items_action_node',
        name = 'approach_items_action_node',
        namespace = namespace,
        output = 'screen',
        parameters = []) 

    approach_items_target_cmd = Node(
        package = 'controller',
        executable = 'approach_items_target_action_node',
        name = 'approach_items_target_action_node',
        namespace = namespace,
        output = 'screen',
        parameters = [])  
        
        
    drop_items_cmd = Node(
        package = 'controller',
        executable = 'drop_items_action_node',
        name = 'drop_items_action_node',
        namespace = namespace,
        output = 'screen',
        parameters = [])  
        
        
    grab_items_cmd = Node(
        package = 'controller',
        executable = 'grab_items_action_node',
        name = 'grab_items_action_node',
        namespace = namespace,
        output = 'screen',
        parameters = [])  

    handle_items_cmd = Node(
        package = 'controller',
        executable = 'handle_items_action_node',
        name = 'handle_items_action_node',
        namespace = namespace,
        output = 'screen',
        parameters = [])
        
        
    
    camera_active_cmd = Node(
        package='controller',
        executable ='camera_active_action_node',
        name ='camera_active_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    charger_approach_cmd = Node(
        package='controller',
        executable ='charger_approach_action_node',
        name ='charger_approach_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])

    battery_charge_1_cmd = Node(
        package='controller',
        executable ='battery_charge_1_action_node',
        name ='battery_charge_1_action_node',
        namespace = namespace,
        output ='screen',
        parameters=[])
        
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

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    #ld.add_action(charge_cmd)

   
    ld.add_action(approach_items_cmd)
    ld.add_action(approach_items_target_cmd)
    ld.add_action(drop_items_cmd)

    
    ld.add_action(grab_items_cmd)
    ld.add_action(handle_items_cmd)
    ld.add_action(camera_active_cmd)

    ld.add_action(battery_charge_1_cmd)
    ld.add_action(charger_approach_cmd)
    ld.add_action(explore_start_cmd)

    ld.add_action(map_start_cmd)
    ld.add_action(wander_around_cmd)
    ld.add_action(obj_detect_start_cmd)
 

    return ld
