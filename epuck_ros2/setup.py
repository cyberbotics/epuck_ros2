from setuptools import setup

package_name = 'epuck_ros2'
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/robot_launch.py']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Minimal example showing how to control a robot with ROS2 in Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'epuck2_driver = epuck_ros2.epuck2_driver:main',
            'camera_calibration = epuck_ros2.camera_calibration:main'
        ],
    }
)
