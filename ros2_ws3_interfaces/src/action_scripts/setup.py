from setuptools import find_packages, setup

package_name = 'action_scripts'

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
            "CountUntil_server = action_scripts.CountUntil_server:main",
            "CountUntil_client = action_scripts.CountUntil_client:main",
            "MoveDist_server = action_scripts.MoveDist_server:main",
            "MoveDist_client = action_scripts.MoveDist_client:main",
        ],
    },
)
