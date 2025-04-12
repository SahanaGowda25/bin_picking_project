from setuptools import setup

package_name = 'emergency_button'

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
    description='Emergency button publisher with press/release services',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_node = emergency_button.emergency_node:main',
        ],
    },
)