from setuptools import setup

package_name = 'sorting_cell_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vj',
    maintainer_email='viswajithmanthripragada44@gmail.com',
    description='Synthetic camera and HSV cube detector',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = sorting_cell_vision.vision_node:main',
        ],
    },
)
