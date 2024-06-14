from setuptools import setup
from glob import glob

package_name = 'tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lokplart',
    maintainer_email='lokplart@gmail.com',
    description='tracker',
    entry_points={
        'console_scripts': [
            'tracker = tracker.tracker:main'
        ]
    },
)
