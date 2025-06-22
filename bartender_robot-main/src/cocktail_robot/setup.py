from glob import glob
from setuptools import find_packages, setup

package_name = 'cocktail_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
         ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [package_name + '/locations/' + 'pose.yaml']),
        ('share/cocktail_robot/image', glob('image/*')),
        ('share/cocktail_robot/image/maker', glob('image/maker/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ethica',
    maintainer_email='ethica@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'main = cocktail_robot.main:main',
        	'bartender_gui = cocktail_robot.bartender_gui:main'
        ],
    },
)
