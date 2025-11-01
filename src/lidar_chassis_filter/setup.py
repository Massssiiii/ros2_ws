from setuptools import setup

package_name = 'lidar_chassis_filter'

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
    maintainer='raspbotix',
    maintainer_email='rasp@raspbotix.local',
    description='Filtre LaserScan pour supprimer le châssis du robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_chassis_filter_node = lidar_chassis_filter.lidar_chassis_filter_node:main',
        ],
    },
)

