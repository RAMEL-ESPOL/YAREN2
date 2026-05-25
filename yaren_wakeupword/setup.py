from setuptools import find_packages, setup

package_name = 'yaren_wakeupword'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wake_word_node    = yaren_wakeupword.wake_word_node:main',
            'voice_menu_node   = yaren_wakeupword.yaren_voice_menu:main',
        ],
    },
)