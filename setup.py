from setuptools import find_packages, setup

package_name = 'husky_assembly_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # Shared BarAssemblyAction / Movement schema; pinned to a git SHA so
        # the interchange format is reproducible across the three consumer
        # repos (husky-assembly-teleop, husky_assembly_tamp,
        # bar_joint_rhino_design_workflow).
        'rs_data_structure @ git+https://github.com/yijiangh/rs_data_structure.git'
        '@36564dc494ecb48fa61c0fa31c894747e1274000',
    ],
    zip_safe=True,
    maintainer='Jakob Genhart',
    maintainer_email='jakob.genhart@inf.ethz.ch',
    description='Monitor node for husky robots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'husky_monitor = husky_assembly_teleop.monitor:main',
        ],
    },
)
