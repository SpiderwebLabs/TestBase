from setuptools import setup

package_name = 'my_station_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spiderweb',
    maintainer_email='Inno4863',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'message_receiver_node = my_station_pkg.message_receiver_node:main',
            'sys_controller = my_station_pkg.sys_controller:main',
            'drone_comms_node = my_station_pkg.drone_comms_node:main',
            "api_update_node = my_station_pkg.api_update_node:main",
            "arduino_node = my_station_pkg.arduino_node:main",
            "streamer_node = my_station_pkg.streamer_node:main",
            "img_processing_node = my_station_pkg.img_processing_node:main",
            "box_select_node = my_station_pkg.box_select_node:main",
            "tracking_node = my_station_pkg.tracking_node:main",
            "error_tracking_node = my_station_pkg.error_tracking_node:main",
            "ai_node = my_station_pkg.ai_node:main",
            "mission_update = my_station_pkg.mission_update:main",
            "missioncomplete = my_station_pkg.missioncomplete:main",
            "slave_node = my_station_pkg.slave_node:main",
            "telemetrypoll_node = my_station_pkg.telemetrypoll_node:main",
            "timer_node = my_station_pkg.timer_node:main",
            "timer_server = my_station_pkg.timer_server:main",
            "auto_mode = my_station_pkg.auto_mode:main",
            "manual_mode= my_station_pkg.manual_mode:main",
            "Batteryholder = my_station_pkg.Batteryholder:main",
            "rtk = my_station_pkg.rtk:main",
            "serial = my_station_pkg.serial:main",
            'test= my_station_pkg.test:main',
            'Camera_streamer = my_station_pkg.Camera_streamer:main',
            'ip_cam=my_station_pkg.ip_cam:main',
            'ip_cam_ai= my_station_pkg.ip_cam_ai:main',
            'Camera_streamer2 = my_station_pkg.Camera_streamer2:main',
            'Camera_streamer3 = my_station_pkg.Camera_streamer3:main',
            'Camera_control = my_station_pkg.Camera_control:main',
            'Plate_detention = my_station_pkg.Plate_detention:main',
            "add_vehicle = my_station_pkg.add_vehicle:main",
            "dataprocessing = my_station_pkg.dataprocessing:main"


        ],
    },
)
