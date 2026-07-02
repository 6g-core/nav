from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='haorunlai@huawei.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "nomap = nav.nomap:main",
            "nav_stubbing_enhanced_3gpp = nav_stubbing.nav_stubbing_enhanced_3gpp:main",
            "nav_stubbing_enhanced_planA = nav_stubbing.nav_stubbing_enhanced_planA:main",
            "nav_stubbing_enhanced_planB = nav_stubbing.nav_stubbing_enhanced_planB:main",
            "interaction_experience_dog = nav_stubbing.interaction_experience_dog:main",
            "nav_stubbing_with_map = nav_stubbing.nav_stubbing_with_map:main",
            "nav_no_move = nav_stubbing.nav_stubbing_no_move:main"
        ],
    },
)
