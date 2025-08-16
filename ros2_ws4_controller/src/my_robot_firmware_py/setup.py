from setuptools import find_packages, setup

package_name = 'my_robot_firmware_py'

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
    maintainer_email='tinglin194@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Arduino_serial_publisher = my_robot_firmware_py.Arduino_serial_publisher:main',
            'Arduino_serial_receiver  = my_robot_firmware_py.Arduino_serial_receiver:main',
            'Arduino_serial_lifecycle = my_robot_firmware_py.Arduino_serial_lifecycle:main',
        ],
    },
)
