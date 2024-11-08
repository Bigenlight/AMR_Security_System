from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_custom_publishers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # 필요 시 다른 데이터 파일 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Publishers for /initialpose and /goal_pose',
    license='Apache License 2.0',
    # tests_require=['pytest'],  # 이 라인을 제거합니다.
    entry_points={
        'console_scripts': [
            'publish_initial_and_goal = turtlebot3_custom_publishers.publish_initial_and_goal:main',
        ],
    },
)
