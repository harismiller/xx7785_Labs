from setuptools import find_packages, setup

package_name = 'team11_navigate_to_goal'

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
    description='xx7785 Lab 4 Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'getObjectRange = team11_navigate_to_goal.getObjectRange:main',
            'goToGoal = team11_navigate_to_goal.goToGoal:main'
        ],
    },
)
