from setuptools import setup
import os
from glob import glob

package_name = 'computer_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    scripts=['scripts/live_undistort.py'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@university.edu',
    description='Live undistortion node for Puzzlebot camera calibration',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
