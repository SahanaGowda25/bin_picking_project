from setuptools import setup

package_name = 'barcode_scanner'

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
    description='Barcode scanner publisher and service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'barcode_node = barcode_scanner.barcode_node:main',
        ],
    },
)
