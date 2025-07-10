from setuptools import find_packages, setup

package_name = 'robochemist'

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
            'pickup = robochemist.pickup:main',
            'pickup_tilt = robochemist.pickup_tilt:main',
            'moveto = robochemist.moveto:main',
            'heater = robochemist.move_into_heater:main',
            'open_gripper = robochemist.gripper_open:main',
            'close_gripper = robochemist.gripper_close:main',
            'zero_gripper = robochemist.gripper_zero:main',
            'cumotion = robochemist.cumotion_service:main',
            'start_pose = robochemist.start_pose:main',
            'disable_pose = robochemist.disable_pose:main',
            'cartesian_control_moveit = robochemist.cartesian_control_moveit:main',
            'shaking = robochemist.shaking:main',
            'check_ammonia = robochemist.check_ammonia:main'
        ],
    },
)
