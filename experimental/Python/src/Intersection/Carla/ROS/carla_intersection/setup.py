from setuptools import setup, find_packages

package_name = 'carla_intersection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soroush',
    maintainer_email='soroosh129@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rsu_node = src.rsu_node:main',
            'vehicle_node = src.vehicle_node:main',
            'carla_sim_node = src.carla_sim_node:main'
        ],
    },
)
