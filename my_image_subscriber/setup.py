from setuptools import setup, find_packages
import os
import glob

package_name = 'my_image_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Install configuration files
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join(package_name, 'config', '*.yaml'))),
        # Include other data files if necessary
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.launch.py'))),

        (os.path.join('share', package_name, 'templates'), glob.glob(os.path.join(package_name, 'templates', '*.html'))),
        # Include other data files if necessary
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'ultralytics',  # Added for YOLO
        'shapely',      # Added if used elsewhere
        'flask',
        # Remove 'flask' if it's not used by your nodes
    ],
    zip_safe=True,
    maintainer='theo',
    maintainer_email='tpingouin@gmail.com',
    description='An image subscriber using ROS2 and YOLO for tracking',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = my_image_subscriber.image_subscriber:main',
            'yolo_publisher = my_image_subscriber.yolo_publisher:main',
            'trecking_mode = my_image_subscriber.trecking_mode:main',
            'yolo_tracking_publisher = my_image_subscriber.yolo_tracking_publisher:main',  # New Entry Point
        ],
    },
)
