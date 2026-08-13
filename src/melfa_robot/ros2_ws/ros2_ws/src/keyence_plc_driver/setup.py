from setuptools import setup, find_packages

package_name = 'keyence_plc_ethernet_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='Keyence PLC Ethernet driver for ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Add your node entry points here, e.g.:
            # 'plc_driver = keyence_plc_ethernet_driver.plc_driver_node:main',
        ],
    },
)
