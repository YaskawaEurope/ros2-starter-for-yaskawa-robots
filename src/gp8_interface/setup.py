from setuptools import setup

package_name = 'gp8_interface'

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
    maintainer='markus',
    maintainer_email='markus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'stream = gp8_interface.gp8_stream:main',
        'vision = gp8_interface.gp8_stream_vision:main',
        'move_on_traj = gp8_interface.gp8_node:main'
    ],
    },
)
