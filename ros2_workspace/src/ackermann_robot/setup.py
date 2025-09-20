from setuptools import setup, find_packages

package_name = 'ackermann_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/ackermann_robot_hardware.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/hardware_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ackermann Robot Team',
    maintainer_email='robot@ackermann.dev',
    description='Professional 4-wheel independent drive Ackermann robot with steering control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_node = ackermann_robot.kinematics_node:main',
            'hardware_interface = ackermann_robot.hardware_interface:main',
        ],
    },
)