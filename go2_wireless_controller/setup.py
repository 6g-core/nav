from setuptools import setup

package_name = 'go2_wireless_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@example.com',
    description='Go2 START button network sender',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wireless_controller_node = go2_wireless_controller.wireless_controller_node:main',
        ],
    },
)
