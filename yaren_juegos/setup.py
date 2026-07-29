from setuptools import setup
import os
from glob import glob

package_name = 'yaren_juegos'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ── Instala el HTML del juego en share/yaren_juegos/web/ ──────────
        (os.path.join('share', package_name, 'web'),
            glob('web/*.html')),
        # ── Launch files ──────────────────────────────────────────────────
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RAMEL',
    maintainer_email='ramel@espol.edu.ec',
    description='Yaren Juegos — minijuegos para YAREN2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dance_game_node = yaren_juegos.dance_game:main',
            'memoria_node    = yaren_juegos.memoria_node:main',
        ],
    },
)