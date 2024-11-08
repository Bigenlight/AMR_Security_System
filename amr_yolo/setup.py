from setuptools import setup, find_packages

package_name = 'amr_yolo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=['setuptools', 'opencv-python', 'ultralytics', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple YOLO detection node for publishing detection info',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'amr_yolo = amr_yolo.amr_yolo:main',
        ],
    },
)
