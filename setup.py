import os
from glob import glob
from setuptools import setup

package_name = 'iwr6843aop_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nm',
    maintainer_email='nhma@mmmi.sdu.dk',
    description='Publishes pointcloud data from IWR6843AOPEVM mmWave device',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'pcl_pub = iwr6843aop_pub.publisher_member_function:main',
        ],
    },
)
