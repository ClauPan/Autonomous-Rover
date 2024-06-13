from setuptools import setup
from glob import glob

package_name = 'tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lokplart',
    maintainer_email='lokplart@gmail.com',
    description='tracker',
    entry_points={
        'console_scripts': [
            'detect_ball = tracker.detect_ball:main',
            'detect_ball_3d = tracker.detect_ball_3d:main',
            'follow_ball = tracker.follow_ball:main',
        ],
    },
)
