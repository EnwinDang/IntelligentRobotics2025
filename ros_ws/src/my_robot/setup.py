from setuptools import find_packages, setup

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #oef1
            'battery_publisher = my_robot.battery_publisher:main',
            'battery_monitor = my_robot.battery_monitor:main',
            #oef2
            'key_publisher = my_robot.key_publisher:main',
            'movement_subscriber = my_robot.movement_subscriber:main',
            #oef3
            'velocity_publisher = my_robot.velocity_publisher:main',
            'velocity_subscriber = my_robot.velocity_subscriber:main',
        ],
    },
)
