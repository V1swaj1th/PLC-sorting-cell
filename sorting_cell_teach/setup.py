from setuptools import setup

package_name = 'sorting_cell_teach'

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
    maintainer='Viswajith',
    maintainer_email='viswajithmanthripragada44@gmail.com',
    description='Keyboard client for the teach pendant',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teach_keyboard = sorting_cell_teach.teach_keyboard:main',
        ],
    },
)
