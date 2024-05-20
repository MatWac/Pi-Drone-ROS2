from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'icm_publisher'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name,'config'), glob('config/*.yaml'))
    ],
    install_requires=[
        'setuptools',
        'icm20948'
    ],
    zip_safe=True,
    maintainer='matwac',
    maintainer_email='mattwac59@gmail.com',
    description='Publish ICM datas',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'icm_publisher = icm_publisher.publisher:main',
        ],
    },
)
