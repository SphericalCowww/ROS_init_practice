from setuptools import find_packages, setup

package_name = 'lifecycle_scripts'

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
            "Number_publisher = lifecycle_scripts.Number_publisher: main",
            "Number_manager   = lifecycle_scripts.Number_manager:   main",
            "MoveDist_lifecycle      = lifecycle_scripts.MoveDist_lifecycle:      main",
            "MoveDist_lifecycleMulti = lifecycle_scripts.MoveDist_lifecycleMulti: main",
            "MoveDist_manager        = lifecycle_scripts.MoveDist_manager:        main",
        ],
    },
)
