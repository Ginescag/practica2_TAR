from setuptools import find_packages, setup

package_name = 'service_temp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name + '/srv', ['srv/Temperature.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clase-docker',
    maintainer_email='Clase-docker@todo.todo',
    description='Paquete cliente-servidor para cambio de unidad de temperatura',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_server = service_temp.temperature_server:main',
            'temperature_client = service_temp.temperature_client:main',
        ],
    },
)