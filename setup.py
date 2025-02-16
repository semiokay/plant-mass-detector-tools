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
            'pc_subscriber = img_utilities.pc_subscriber:main'
        ],
    },
)
