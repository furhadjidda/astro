from setuptools import find_packages
from setuptools import setup

package_name = 'astro_teleop'

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
        'Teleoperation node using keyboard for TurtleBot3.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = astro_teleop.script.teleop_keyboard:main'
        ],
    },
)
