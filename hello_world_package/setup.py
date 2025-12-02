from setuptools import find_packages, setup

package_name = 'hello_world_package'

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
    maintainer_email='americo.cherubini2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # NB Here we need to insert all entrypoints that we can call from the command line
        # for example >> ros2 run hello_world_package publisher -> runs main() from publisher_node.py
        'console_scripts': [
            'publisher = hello_world_package.publisher_node:main',
            'subscriber = hello_world_package.subscriber_node:main',
        ],
    },
)
