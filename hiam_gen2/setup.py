from setuptools import find_packages, setup

package_name = 'hiam_gen2'

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
    description='ROS2 control package for HIAM Gen2 experiment',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pickup_tilt = hiam_gen2.pickup_tilt:main',
            'pickup = hiam_gen2.pickup:main',
            'moveto = hiam_gen2.moveto:main',
            'moveto_tilt = hiam_gen2.moveto_tilt:main',
            'moveto_marker = hiam_gen2.moveto_marker:main',
            'moveto_marker_tilt = hiam_gen2.moveto_marker_tilt:main',
            'shaking = hiam_gen2.shaking:main',
            'start_pose = hiam_gen2.start_pose:main',
            'disable_pose = hiam_gen2.disable_pose:main',
            'cartesian_control_moveit = hiam_gen2.cartesian_control_moveit:main',
            'cartesian_control_client = hiam_gen2.cartesian_control_client:main',
            'cartesian_x = hiam_gen2.cartesian_x:main',
            'cumotion = hiam_gen2.cumotion_service:main',
            'open_gripper = robochemist.gripper_open:main',
            'close_gripper = robochemist.gripper_close:main',
            'zero_gripper = robochemist.gripper_zero:main',
        ],
    },
)
