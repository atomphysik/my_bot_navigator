from setuptools import setup

package_name = 'my_bot_navigator'

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
    maintainer='kwon',
    maintainer_email='atomphysik01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'costmap_obstacle_avoidance.py = my_bot_navigator.costmap_obstacle_avoidance:main'
        ],
    },
)
