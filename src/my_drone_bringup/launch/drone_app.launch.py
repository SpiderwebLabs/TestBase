from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import OpaqueFunction

def generate_launch_description():
    '''Create a launch description and add nodes to it'''
    ld = LaunchDescription()
    drone_comms_node = Node(
        package='my_station_pkg',
        executable='drone_comms_node',
        output='screen')

    telemetrypoll_node = Node(
        package='my_station_pkg',
        executable='telemetrypoll_node',
        output='screen'
    )

    mission_update = Node(
        package="my_station_pkg",
        executable="mission_update",
        output='screen'
    )

    arduino_node = Node(
        package='my_station_pkg',
        executable='arduino_node',
        output='screen'
    )

    sys_controller_node = Node(
        package='my_station_pkg',
        executable='sys_controller',
        output='screen'
    )

    message_receiver_node = Node(
        package='my_station_pkg',
        executable='message_receiver_node',
        output='screen'
    )

    api_update_node = Node(
        package='my_station_pkg',
        executable='api_update_node',
        output='screen'
    )

    missioncomplete = Node(
        package='my_station_pkg',
        executable='missioncomplete'
    )

    auto_mode = Node(
        package='my_station_pkg',
        executable='auto_mode'
    )

    manual_mode = Node(
        package='my_station_pkg',
        executable='manual_mode'
    )

    timer_node = Node(
        package='my_station_pkg',
        executable='timer_node'
    )
    ai_node = Node(
        package='my_station_pkg',
        executable='ai_node'
    )

    box_select_node = Node(
        package="my_station_pkg",
        executable="box_select_node"
    )

    # slave_node = Node (
    #     package= "my_station_pkg",
    #     executable = 'slave_node',
    #     output = 'screen'
    # )

    error_tracking_node = Node(
        package="my_station_pkg",
        executable="error_tracking_node",
        output='screen'
    )

    img_processing_node = Node(
        package="my_station_pkg",
        executable="img_processing_node"
    )
    streamer_node = Node(
        package="my_station_pkg",
        executable="streamer_node"
    )
    tracking_node = Node(
        package="my_station_pkg",
        executable="tracking_node"
    )

    '''Add the nodes to the launch description'''
    ld.add_action(drone_comms_node)
    ld.add_action(telemetrypoll_node)
    ld.add_action(mission_update)
    ld.add_action(arduino_node)
    ld.add_action(message_receiver_node)
    ld.add_action(api_update_node)
    ld.add_action(missioncomplete)
    ld.add_action(auto_mode)
    ld.add_action(manual_mode)
    ld.add_action(timer_node)
    ld.add_action(ai_node)
    ld.add_action(sys_controller_node)
    ld.add_action(streamer_node)
    ld.add_action(box_select_node)
    ld.add_action(img_processing_node)
    ld.add_action(tracking_node)
    # ld.add_action(slave_node)
    ld.add_action(error_tracking_node)

    ld.add_action(TimerAction(period=1.0,
                              actions=[OpaqueFunction(function=lambda context: print('Delay completed.'))]))

    return ld
