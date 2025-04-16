from setuptools import find_packages, setup

package_name = 'marker_detection'

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
    maintainer='szhuang',
    maintainer_email='shengyangzhuang@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'estimate_marker_pose = marker_detection.estimate_marker_pose:main',
            'gripper_estimate_marker_pose = marker_detection.estimate_marker_pose_gripper:main',
            'gripper_pnp = marker_detection.estimate_marker_pose_gripper_pnp:main',
            'sim_estimtae_marker_pose = marker_detection.estimate_marker_gripper_sim:main'
        ],
    },
)
