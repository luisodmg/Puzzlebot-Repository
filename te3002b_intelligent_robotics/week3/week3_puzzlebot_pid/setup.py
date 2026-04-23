from setuptools import find_packages, setup

package_name = 'week3_puzzlebot_pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mini_challenge2.launch.py']),
        ('share/' + package_name + '/config', ['config/controller.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='Week 3 PID control and path generation for Puzzlebot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_pose_controller = week3_puzzlebot_pid.pid_pose_controller:main',
            'path_generator = week3_puzzlebot_pid.path_generator:main',
        ],
    },
)
