from setuptools import find_packages, setup

package_name = 'img_utilities'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/biomass_detector.launch.py']),
        ('share/' + package_name + '/config', ['config/params_ekf_node.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andre',
    maintainer_email='andre.rossignatti.faroni@estudiantat.upc.edu',
    description='All types of utilities to transfer input data to a usable format.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_to_file = img_utilities.img_to_file:main',
            'pc_subscriber = img_utilities.pc_subscriber:main',
            'camera_msg_filter = img_utilities.camera_msg_filter:main',
            'pc_snap = img_utilities.pc_snap:main',
            'pc_fuser = img_utilities.pc_fuser:main',
            'icp_server = img_utilities.icp_server:main',
        ],
    },
)
