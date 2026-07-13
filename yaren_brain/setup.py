from setuptools import find_packages, setup

package_name = 'yaren_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roberto',
    maintainer_email='raestrad@espol.edu.ec',
    description='Yaren Brain Node - Personalidad autonoma con gestos',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yaren_brain_node = yaren_brain.yaren_brain_node:main',
        ],
    },
)