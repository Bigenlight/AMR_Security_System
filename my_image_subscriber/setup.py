from setuptools import setup

package_name = 'my_image_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='An image subscriber using ROS2 and OpenCV',
    license='License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = my_image_subscriber.image_subscriber:main',
            'yolo_publisher = my_image_subscriber.yolo_publisher:main',        
        ],
    },
)
