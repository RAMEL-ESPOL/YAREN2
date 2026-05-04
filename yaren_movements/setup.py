from setuptools import setup
import os
from glob import glob

package_name = 'yaren_movements'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ramel',
    maintainer_email='ramel@todo.com',
    description='Paquete de movimientos para Yaren',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yaren_fullmovement = yaren_movements.yaren_fullmovement:main',
            'yaren_movement = yaren_movements.yaren_movement:main',
            'yaren_rutina1 = yaren_movements.yaren_rutina1:main',
        ],
    },
)