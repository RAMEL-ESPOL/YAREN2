import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yaren_paint'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roberto',
    maintainer_email='raestrad@espol.edu.ec',
    description='YAREN PAINT - juego de pintura',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'paint_game = yaren_paint.paint_node:main',
        ],
    },
)