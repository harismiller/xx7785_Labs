from setuptools import find_packages, setup

package_name = 'team11_object_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haris',
    maintainer_email='haris@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotate_robot = team11_object_follower.rotate_robot:main',
            'find_object = team11_object_follower.find_object:main',
            # 'follow_obj = team11_object_follower.follow_obj.py:generate_launch_description'
        ],
    },
)
