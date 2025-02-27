from setuptools import find_packages, setup

package_name = 'crawling_sunghyuk'

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
    maintainer='sshyuk',
    maintainer_email='qaz73700@naver.com',
    description='Gripper control package for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_control=scripts.gripper_control:main',
        ],
    },
       
)
