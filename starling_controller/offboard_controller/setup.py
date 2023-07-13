from setuptools import setup

package_name = 'offboard_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NDDuy',
    maintainer_email='duy.nd202357@sis.hust.edu.vn',
    description='offboard controller template',
    license='MIT License @ NDDuy 2023',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = offboard_controller.main:main'
        ],
    },
)
