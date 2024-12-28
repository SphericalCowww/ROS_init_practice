from setuptools import find_packages, setup

package_name = 'practice_controller'

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
    maintainer='kali',
    maintainer_email='kali@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node              = practice_controller.practice_node:main',
            'turtle_move_cycle_node = practice_controller.turtle_move_cycle:main',
            'turtle_read_pose_node  = practice_controller.turtle_read_pose:main',
            'turtle_controller_node = practice_controller.turtle_controller:main',
        ],
    },
)
