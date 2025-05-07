import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "simple_diffdrive_differential_IK_odom_msg_broadcast_controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "True"
        }.items()
    )

    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py"
    )

    rp_laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[os.path.join(
            get_package_share_directory("bumperbot_bringup"),
            "config",
            "rplidar_a1.yaml"
        )],
        output="screen"
    )

    yd_laser_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ydlidar_ros2_driver"),
            "launch",
            "ydlidar_launch.py"
        )
    )

    # joystick = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("bumperbot_controller"),
    #         "launch",
    #         "joystick_teleop.launch.py"
    #     )
    # )

    return LaunchDescription([
        hardware_interface,
        controller,
        imu_driver_node,
        rp_laser_driver,
        # yd_laser_driver,
        # joystick
    ])
