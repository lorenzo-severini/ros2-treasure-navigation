from setuptools import find_packages, setup
from setuptools import setup
import os
from glob import glob

package_name = 'ros2_navigation_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='lorenzo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_navigation_plan = ros2_navigation_control.service_navigation_plan:main',
            'client_async_navigation = ros2_navigation_control.client_async_navigation:main',
            'treasure_hunt = ros2_navigation_control.treasure_hunt:main'
        ],
    },
)
