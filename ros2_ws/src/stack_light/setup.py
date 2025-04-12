from setuptools import setup

package_name = 'stack_light'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sahana',
    maintainer_email='you@example.com',
    description='Stack light status monitor for ROS 2 system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stack_node = stack_light.stack_node:main',
        ],
    },
)
