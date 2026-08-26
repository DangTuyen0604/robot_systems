from setuptools import find_packages, setup
import os          
from glob import glob
package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('perception_pkg/*.onnx')),
        (os.path.join('share', package_name), glob('perception_pkg/*.onnx.data')),
        (os.path.join('share', package_name), glob('perception_pkg/*.pt')),
        (os.path.join('share', package_name), glob('perception_pkg/*.pth')),
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
            'lane_detection_node =perception_pkg.lane_detection_node:main',
            'traffic_sign_node = perception_pkg.traffic_sign_node:main',
        ],
    },
)
