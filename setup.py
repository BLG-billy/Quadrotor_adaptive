from setuptools import find_packages, setup

package_name = 'circle_offboard_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='gqli',
    maintainer_email='gqli@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'circle_offboard = circle_offboard_py.circle_offboard:main',
            'controller_experiment = circle_offboard_py.controller_experiment:main',
            'controller_experiment_l1 = circle_offboard_py.controller_experiment_l1:main',
            'controller_experiment_ctbr = circle_offboard_py.ctbr.controller_experiment_ctbr:main',
            'ctbr_controller = circle_offboard_py.ctbr.ctbr_controller_node:main',
            'geometric_controller = circle_offboard_py.geometric_controller_node:main',
            'circle_trajectory = circle_offboard_py.trajectory_node:main',
            'flight_manager = circle_offboard_py.node_runner:flight_main',
            'trajectory_monitor = circle_offboard_py.trajectory_monitor:main',
        ],
    },
)
