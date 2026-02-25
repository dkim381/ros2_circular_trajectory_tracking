from setuptools import find_packages, setup

package_name = 'mobile_controller'

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
    maintainer='donguk',
    maintainer_email='donguk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = mobile_controller.controller_node:main',
            'odom_logger = mobile_controller.odom_logger:main',
            'odom_filter_node = mobile_controller.odom_filter_node:main',

        ],
    },
)
