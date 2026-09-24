from setuptools import setup
from glob import glob
import os

package_name = 'steelhead_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'sshkeyboard'],
    zip_safe=True,
    maintainer='logan',
    maintainer_email='logan.fillo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'key_publisher = steelhead_teleop.key_publisher:main',
            'keyboard_teleop = steelhead_teleop.keyboard_teleop:main',
            'keyboard_pid_teleop = steelhead_teleop.keyboard_pid_teleop:main',
            'ssh_keyboard_teleop = steelhead_teleop.ssh_keyboard_teleop:main',
            'ssh_keyboard_pid_teleop = steelhead_teleop.ssh_keyboard_pid_teleop:main',
            'thruster_mapper_test = steelhead_teleop.thruster_mapper_test:main',
            'controller_teleop = steelhead_teleop.controller_teleop:main'
        ],
    },
)
