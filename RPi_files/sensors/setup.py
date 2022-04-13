from setuptools import setup

package_name = 'sensors'
sub_packages = 'sensors/pn532'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, sub_packages],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_sensors = sensors.sensors:main',
        ],
    },
)
