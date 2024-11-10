from setuptools import setup, find_packages
import os
import glob

package_name = 'my_image_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        # Include any necessary data files here
        (os.path.join('share', package_name, 'templates'), glob.glob(os.path.join(package_name, 'templates', '*.html'))),
        # Add launch files or other resources if needed
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'ultralytics',  # Added for YOLO
        'shapely',      # Added if used elsewhere
        # Remove 'flask' if it's not used by your nodes
    ],
    zip_safe=True,
    maintainer='theo',
    maintainer_email='tpingouin@gmail.com',
    description='An image subscriber using ROS2 and YOLO for tracking',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = my_image_subscriber.image_subscriber:main',
            'yolo_publisher = my_image_subscriber.yolo_publisher:main',
            'trecking_mode = my_image_subscriber.trecking_mode:main',
            'yolo_tracking_publisher = my_image_subscriber.yolo_tracking_publisher:main',  # New Entry Point
        ],
    },
)
