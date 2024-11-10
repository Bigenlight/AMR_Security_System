from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_custom_publishers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Python 스크립트를 설치 대상으로 추가
        (os.path.join('share', package_name, package_name), glob(os.path.join(package_name, '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # 실제 이름으로 변경
    maintainer_email='your_email@example.com',  # 실제 이메일로 변경
    description='Publishers for /initialpose and /waypoints',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 노드 실행을 위한 entry point 추가
            'publish_initial_and_goal = turtlebot3_custom_publishers.publish_initial_and_goal:main',
        ],
    },
)
