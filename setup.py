from setuptools import find_packages, setup

package_name = 'chung_navigate_to_goal'

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
    maintainer='nchung',
    maintainer_email='nathanchung@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_ranger: chung_navigate_to_goal.get_object_range:main',
            'go_to_goal: chung_navigate_to_goal.go_to_goal:main',
            'odom_logger: chung_navigate_to_goal.print_fixed_odometry:main'
        ],
    },
)
