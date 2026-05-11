import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'husky_assembly_teleop'

def strip_first_dir(path):
    first_sep = path.strip('/').find('/')
    if first_sep < 0:
        return ''
    else:
        return path[first_sep+1:]

def copy_dir(dst, src):
    return [(os.path.join(dst, strip_first_dir(dirpath)), [os.path.join(dirpath, f) for f in files]) for (dirpath, dirnames, files) in os.walk(src)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/data',  glob(os.path.join('data', '**', '*.*'), recursive=True)),
        #('share/' + package_name + '/data',  [f for f in copy_dir('data', '')]),
    ], #copy_dir('share/' + package_name + '/data', 'data'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jakob Genhart',
    maintainer_email='jakob.genhart@inf.ethz.ch',
    description='Monitor node for husky robots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'husky_monitor = husky_assembly_teleop.husky_monitor:main',
            'optitrack_python_sample = husky_assembly_teleop.optitrack.PythonSample:main',
            'test_mocap = husky_assembly_teleop.optitrack.test_mocap:main',
            'test_setio = husky_assembly_teleop.test_setio:main',
            'mocap_experiment_analyze = husky_assembly_teleop.mocap_experiment:main_analyze',
            'mocap_experiment_report = husky_assembly_teleop.mocap_experiment:main_report',
        ],
    },
)
