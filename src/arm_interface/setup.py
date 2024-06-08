from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py", recursive=True)
        )
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='nifley',
    maintainer_email='finnsbart@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_interface_node = arm_interface.arm_interface:main',
            'arm_spacenav_control_node = arm_interface.arm_spacenav_control:main',
        ],
    },
)
