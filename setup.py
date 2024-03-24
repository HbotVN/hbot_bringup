from setuptools import find_packages, setup

package_name = 'hbot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hbot_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/config', ['config/slam_params.yaml']),
        ('share/' + package_name + '/config', ['config/slam_params_localize.yaml']),
        ('share/' + package_name + '/config', ['config/yahboom_driver_params.yaml']),
        ('share/' + package_name + '/config', ['config/hbot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='huyhust13@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
