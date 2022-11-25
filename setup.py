from setuptools import setup

package_name = 'myRobot'

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
    maintainer='soluopomonoki',
    maintainer_email='pjikonomopoulos@utexas.edu',
    description='ROS2 package for ME369P Group U19 project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command = myRobot.movementPublisher:main',
        ],
    },
)
