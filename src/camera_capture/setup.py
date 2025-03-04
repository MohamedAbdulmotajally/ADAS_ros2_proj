from setuptools import find_packages, setup

package_name = 'camera_capture'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='mohamed',
    maintainer_email='mohamed.abedlmotajally@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_capture_node = camera_capture.camera_node:main'
        ],
    },
)
