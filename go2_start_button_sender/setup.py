from setuptools import setup

package_name = 'go2_start_button_sender'

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
            'start_button_node = go2_start_button_sender.start_button_node:main',
        ],
    },
)
