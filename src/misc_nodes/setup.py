from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'misc_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'matplotlib'],
    zip_safe=True,
    maintainer='Hammad',
    maintainer_email='simsim@todo.todo',
    description='misc nodes',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tf_odom = misc_nodes.tf_odom:main',
        ],
    },
)

