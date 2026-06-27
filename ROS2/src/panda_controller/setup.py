from setuptools import find_packages, setup

package_name = 'panda_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_sim.py']),
        ('share/' + package_name + '/config', [
            'config/panda.rviz',
            'config/fer_fake.urdf.xacro',
            'config/ros2_controllers.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'panda_controller = panda_controller.controller_node:main',
        ],
    },
)
