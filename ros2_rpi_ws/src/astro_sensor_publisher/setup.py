from setuptools import find_packages
from setuptools import setup

package_name = 'astro_sensor'

setup(
    name=package_name,
    version='2.1.5',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    description=(
        'Sensor node to sync time and post process some computationally expensive data'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astro_sensor = astro_sensor.script.astro_sensor:main'
        ],
    },
)
