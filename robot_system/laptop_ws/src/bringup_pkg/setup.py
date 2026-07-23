from setuptools import find_packages, setup
import os # THÊM DÒNG NÀY
from glob import glob # THÊM DÒNG NÀY
from setuptools import setup
package_name = 'bringup_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuyen',
    maintainer_email='tuyen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'system_launch = bringup_pkg.system_launch:main',
        ],
    },
)
