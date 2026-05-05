from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'j100_nav2_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Theo',
    maintainer_email='tpingouin@gmail.com',
    description='One-command bringup for the Clearpath J100 Nav2 stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_goal_ui = j100_nav2_bringup.nav_goal_ui:main',
        ],
    },
)
