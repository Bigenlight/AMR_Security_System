from setuptools import setup, find_packages
import os
import glob

package_name = 'my_image_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        (os.path.join('share', package_name, 'templates'), glob.glob(os.path.join(package_name, 'templates', '*.html'))),
    ],
    install_requires=['setuptools', 'flask', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='An image subscriber using ROS2 and Flask',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'image_subscriber = my_image_subscriber.image_subscriber:main',
            'yolo_publisher = my_image_subscriber.yolo_publisher:main',
            'trecking_mode = my_image_subscriber.trecking_mode:main',        
        ],
    },
)
