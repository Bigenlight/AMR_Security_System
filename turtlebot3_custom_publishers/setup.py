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
        # 런치 파일 포함
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # 실제 유지보수자 이름으로 변경
    maintainer_email='your_email@example.com',  # 실제 이메일로 변경
    description='Publishers for /initialpose and /goal_pose',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'publish_initial_and_goal = turtlebot3_custom_publishers.publish_initial_and_goal:main',
        ],
    },
)
