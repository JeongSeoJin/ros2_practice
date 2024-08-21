from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seojin',
    maintainer_email='seojin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main",
            "draw_circle = my_robot_controller.draw_circle:main",
            "pose_subscriber = my_robot_controller.pose_subscriber:main",
            "turtle_controller = my_robot_controller.turtle_controller:main",
            "talker = my_robot_controller.talker:main",
            "listener = my_robot_controller.listener:main",
            "publisher_node = my_robot_controller.publisher_node:main",
            "subscriber_node = my_robot_controller.subscriber_node:main",
            "data_subscriber = my_robot_controller.data_subscriber:main",
            "serial_reader = my_robot_controller.serial_reader:main"
        ],
    },
)
