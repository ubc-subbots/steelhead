from setuptools import setup
from glob import glob
import os

package_name = 'spiderfish_teleop'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='logan',
    maintainer_email='logan.fillo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'key_publisher = spiderfish_teleop.key_publisher:main',
            'keyboard_teleop = spiderfish_teleop.keyboard_teleop:main',
            'ssh_keyboard_teleop = spiderfish_teleop.ssh_keyboard_teleop:main',
            'sim_thrust_mapper = spiderfish_teleop.sim_thrust_mapper:main',
            'controller_teleop = spiderfish_teleop.controller_teleop:main'
        ],
    },
)
