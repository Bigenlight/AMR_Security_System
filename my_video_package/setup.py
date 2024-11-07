from setuptools import setup
import os
from glob import glob

package_name = 'my_video_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='100rudtjs@naver.com',
    description='ROS2 비디오 캡처 노드 패키지',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'video_capture_node = my_video_package.video_capture_node:main',
        ],
    },
)

