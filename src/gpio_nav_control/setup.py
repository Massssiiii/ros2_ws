from setuptools import setup

package_name = 'gpio_nav_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='votre_nom',
    maintainer_email='votre_email@domaine.com',
    description='Contrôle de navigation ROS2 avec boutons GPIO',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'button_control = gpio_nav_control.button_control:main',
        ],
    },
)
