from setuptools import find_packages, setup

package_name = 'piper_control'

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
    maintainer_email='szhuang@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_space = piper_control.joint_space:main',
            'gripper_basic = piper_control.gripper_control_basic:main',
            'gen = piper_control.motion_gen_curobo:main',
            'exec = piper_control.execute_motion:main',
            'exec_single = piper_control.execute_single_motion:main'
        ],
    },
)
