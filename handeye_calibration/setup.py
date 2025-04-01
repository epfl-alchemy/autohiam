from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'handeye_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szhuang',
    maintainer_email='shengyangzhuang@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker = handeye_calibration.estimate_marker_pose:main',
            'robot = handeye_calibration.estimate_robot_pose:main',
            'handeye = handeye_calibration.calculate_handeye:main',
            'publish = handeye_calibration.publish_eye2hand:main'
        ],
    },
)
