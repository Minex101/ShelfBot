from setuptools import find_packages, setup

package_name = 'slam_cartographer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/slam_cartographer']),
        ('share/slam_cartographer', ['package.xml']),

        # ✅ added BOTH launch files (important fix)
        ('share/slam_cartographer/launch', [
            'launch/cartographer.launch.py',
            'launch/forklift_localiser.launch.py'
        ]),

        ('share/slam_cartographer/config', ['config/forklift.lua']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='affan',
    maintainer_email='affan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'slam = slam_cartographer.slam:main'
        ],
    },
)