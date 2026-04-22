import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    # Define workspace directory (one level up from package)
    workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    pkg_share = FindPackageShare('asr_autopilot')
    thyra_pkg_share = FindPackageShare('thyra')

    
    # directory which workspace is located in
    general_dir = os.path.abspath(os.path.join(workspace_dir, '..', '..', '..'))
    px4_dir = os.path.join(general_dir, 'PX4-Autopilot_thyra')
    
    # Path to the thyra simulation config file
    params_path = PathJoinSubstitution([thyra_pkg_share, 'config', 'thyra_params_sim.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    position_source = LaunchConfiguration('position_source', default='px4')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'position_source',
            default_value='px4',
            description='Position source: px4 or mocap'
        ),
        
        # ExecuteProcess(
        #     cmd=[
        #         'bash', '-c',
        #         f'''
        #         echo "Trying to cd into: {px4_dir}" && \
        #         cd {px4_dir} && \
        #         echo "Now in: $(pwd)" && \
        #         export GAZEBO_RESOURCE_PATH=~/PX4-Autopilot_thyra/Tools/simulation/gz/worlds:$GAZEBO_RESOURCE_PATH && \
        #         echo "Now running PX4 SITL with Gazebo X500 world" && \
        #         make px4_sitl gz_x500_erc > /dev/null 2>&1
        #         '''
        #     ],
        #     shell=True,
        #     output='screen',
        # ),
         ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'''
                echo "Trying to cd into: {px4_dir}" && \
                cd {px4_dir} && \
                echo "Now in: $(pwd)" && \
                export GAZEBO_RESOURCE_PATH=~/PX4-Autopilot_thyra/Tools/simulation/gz/worlds:$GAZEBO_RESOURCE_PATH && \
                echo "Now running PX4 SITL with Gazebo X500 world" && \
                make px4_sitl gz_x500_lidar_down > /dev/null 2>&1
                '''
            ],
            shell=True,
            output='screen',
        ),
        
        # Start MicroXRCEAgent (output suppressed)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='log',
        ),
        
      
        # Delay and launch FlightControllerInterface node
        TimerAction(
            period=20.0,  # Delay in seconds
            actions=[
                Node(
                    package='asr_autopilot',
                    executable='asr_autopilot',
                    name='autopilot',
                    namespace='asr/thyra',
                    remappings=[
                        #('/fmu/out/vehicle_status', '/fmu/out/vehicle_status_v1'),
                        #('/fmu/out/battery_status', '/fmu/out/battery_status_v1'),
                        #('/fmu/in/vehicle_attitude_setpoint', '/fmu/in/vehicle_attitude_setpoint_v1'),
                        ('/asr/thyra/out/distance_sensor', '/fmu/out/distance_sensor'),
                    ],
                    parameters=[
                        params_path,
                        {'use_sim_time': use_sim_time},
                        {'position_source': position_source}
                    ],
                    output='screen'
                )
            ]
        ),
    ])
    
    
    
    
    
    
    