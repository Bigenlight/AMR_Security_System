from setuptools import setup, find_packages
import os
import glob

package_name = 'amr_yolo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.launch.py'))),
        # Include other data files if necessary
        #(os.path.join('share', package_name, 'yolo_models'), glob.glob(os.path.join('yolo_models', '*.pt'))),
        # Include other data files if necessary
    ],
    install_requires=['setuptools', 'opencv-python', 'ultralytics', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple YOLO detection node for publishing detection info',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'amr_yolo = amr_yolo.amr_yolo:main',
            'alarm_listener = amr_yolo.alarm_listener:main',  # Added new node
        ],
    },
)
