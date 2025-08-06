from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dual_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/dual_arm.rviz']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dualpanda',
    maintainer_email='dualpanda@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_joint_angles = dual_arm_control.set_joint_angles:main',
            'set_sine_joint_angles = dual_arm_control.set_sine_joint_angles:main',
        ],
    },
)
