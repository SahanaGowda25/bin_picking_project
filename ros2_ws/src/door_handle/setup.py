from setuptools import setup

package_name = 'door_handle'

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
    description='Door state publisher with toggle service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_node = door_handle.door_node:main',
        ],
    },
)