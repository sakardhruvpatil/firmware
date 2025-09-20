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
            'launch/ackermann_robot_with_viz.launch.py',
            'launch/ackermann_robot_hardware.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/robot_params.yaml',
            'config/hardware_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Developer',
    maintainer_email='dev@example.com',
    description='Four-wheel independent drive Ackermann robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_node = ackermann_robot.kinematics_node:main',
            'simulation_node = ackermann_robot.simulation_node:main',
            'visualization_node = ackermann_robot.visualization_node:main',
            'hardware_interface = ackermann_robot.hardware_interface:main',
        ],
    },
)