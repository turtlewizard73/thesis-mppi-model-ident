from setuptools import setup
from glob import glob

package_name = 'controller_benchmark'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*')),
        ('share/' + package_name, glob('config/*')),
        ('share/' + package_name, glob('worlds/*')),
        ('share/' + package_name, glob('maps/*')),
        ('share/' + package_name, glob('models/*')),
        ('lib/' + package_name,
         ['include/discrete-frechet/distances/discrete.py']),
        # ('lib/' + package_name, [package_name + '/utils.py']),
        # ('lib/' + package_name, [package_name + '/run_test.py']),
        # ('lib/' + package_name, [package_name + '/plot_results.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtlewizard',
    maintainer_email='mate.laszlo703@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        # 'console_scripts': [
        #     'run_test = controller_benchmark.run_test:main',
        #     'plot_results = controller_benchmark.plot_results:main'
        # ],
    },
)
