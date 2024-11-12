from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*')),
        ('share/' + package_name, glob('config/*')),
        ('share/' + package_name, glob('worlds/*')),
        ('share/' + package_name, glob('maps/*')),
        ('share/' + package_name,
         [f for f in glob('models/*') if os.path.isfile(f)]),
        ('share/' + package_name,
         [f for f in glob('models/service_robot/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtlewizard',
    maintainer_email='mate.laszlo703@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
