from setuptools import find_packages, setup

package_name = 'team11_maze_runner'

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
    maintainer_email='19560137+harismiller@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_wall = team11_maze_runner.get_wall:main',
            'navigate_maze = team11_maze_runner.navigate_maze:main',
            'detect_sign = team11_maze_runner.detect_sign:main',
            'navigate_simple = team11_maze_runner.navigate_simple:main',
            'print_odometry = team11_maze_runner.print_odometry:main'
        ],
    },
)
