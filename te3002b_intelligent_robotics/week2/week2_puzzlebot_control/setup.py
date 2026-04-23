from setuptools import find_packages, setup

package_name = 'week2_puzzlebot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mini_challenge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'open_loop_square = week2_puzzlebot_control.open_loop_square:main',
            'path_generator = week2_puzzlebot_control.path_generator:main',
            'waypoint_follower = week2_puzzlebot_control.waypoint_follower:main',
        ],
    },
)
