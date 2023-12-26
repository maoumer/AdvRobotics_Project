from setuptools import setup

package_name = 'robot_drive'

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
    maintainer='Maleficent',
    maintainer_email='root@todo.todo',
    description='Robotics Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pubsub = robot_drive.midterm_proj:main',
                'srv = robot_drive.final_srv_test:main',
                'cli = robot_drive.final_cli_test:main',
                'camera = robot_drive.camera_repub:main',
                'drive_around = robot_drive.drive_around_open_loop:main',
                'drive_around2 = robot_drive.drive_around2_open_loop:main',
                'drive_forward_PID = robot_drive.drive_forward_PID:main',
                'drive_backward_PID = robot_drive.drive_backward_PID:main',
                'sector_PID = robot_drive.sector_PID:main',

                # 'battery = robot_drive.relevant_classes:main',
        ],
    },
)
