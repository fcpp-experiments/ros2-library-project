from setuptools import setup

package_name = 'robot_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Daniele Bortoluzzi',
    author_email='daniele.bortoluzzi@unito.it',
    maintainer='Giordano Scarso',
    maintainer_email='giordano.scarso@edu.unito.it',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Subscribes to /odom, /battery_state ROS2 topic and produces timed output file.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_reader = robot_reader.reader:main',
            'robot_reader_odom = robot_reader.odom_reader:main',
        ],
    },
)
