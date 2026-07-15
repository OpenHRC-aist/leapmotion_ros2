from setuptools import find_packages, setup

package_name = 'ohrc_leap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ohrc_leap_teleoperation.launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shunki',
    maintainer_email='s.itadera@aist.go.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_topic_publisher = ohrc_leap.state_topic_publisher:main',
            'ur5_dual_palm_teleoperation_publisher = ohrc_leap.ur5_dual_palm_teleoperation_publisher:main',
            'fake_hands_publisher = ohrc_leap.fake_hands_publisher:main',
        ],
    },
)
