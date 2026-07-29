from setuptools import setup

package_name = 'yaren_chistes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/sonidos', ['sonidos/risa_sonido.mp3']),  # ← agregar
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roberto',
    maintainer_email='roberto@todo.todo',
    description='Yaren chistes node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chistes_node = yaren_chistes.chistes_node:main',
        ],
    },
)