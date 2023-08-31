from setuptools import setup

package_name = 'dev_opencv_py'

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
          'cam_pub = dev_opencv_py.cam_pub:main',
          'cam_sub = dev_opencv_py.cam_sub:main',
        ],
    },
)
