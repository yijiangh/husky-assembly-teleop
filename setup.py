from setuptools import find_packages, setup

package_name = 'pybullet_mocap'

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
    maintainer='Jakob Genhart',
    maintainer_email='jakob.genhart@inf.ethz.ch',
    description='Monitor node for husky robots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'husky_monitor = pybullet_mocap.husky_monitor:main'
        ],
    },
)
