from setuptools import find_packages, setup

package_name = 'capella_multi_robots_info'

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
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "multi_robots_pub_node = capella_multi_robots_info.multi_robots_info_pub:main",
            "multi_robots_sub_node = capella_multi_robots_info.multi_robots_info_sub:main",
        ],
    },
)
