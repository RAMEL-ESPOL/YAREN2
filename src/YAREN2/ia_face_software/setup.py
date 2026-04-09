from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ia_face_software'

# 1. Definimos los archivos básicos que siempre van
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('*.yaml')),
    (os.path.join('share', package_name), ['basics_responses.json']),
]

# 2. Bucle inteligente para recorrer carpetas que tienen subcarpetas
carpetas_con_assets = ['faces', 'dances', 'games']

for carpeta in carpetas_con_assets:
    for ruta_actual, subcarpetas, archivos in os.walk(carpeta):
        lista_archivos = [os.path.join(ruta_actual, f) for f in archivos]
        if lista_archivos:
            # Crea la ruta exacta de destino en 'share'
            ruta_destino = os.path.join('share', package_name, ruta_actual)
            data_files.append((ruta_destino, lista_archivos))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RAMEL-ESPOL',
    maintainer_email='author@espol.edu.ec',
    description='Multi-Platform AI Software for Managing Children Stress - ROS2 Jazzy',
    license='TODO',
    entry_points={
        'console_scripts': [
            'final_face = ia_face_software.final_face:main',
        ],
    },
)