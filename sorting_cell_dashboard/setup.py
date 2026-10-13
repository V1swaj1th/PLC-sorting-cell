from setuptools import setup

package_name = 'sorting_cell_dashboard'

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
    description='Browser dashboard for the sorting cell scene',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dashboard_node = sorting_cell_dashboard.dashboard_node:main',
        ],
    },
)
