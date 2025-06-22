from setuptools import find_packages, setup

package_name = 'dsr_practice'

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
    maintainer='rokey',
    maintainer_email='ljhwan1997@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "align = dsr_practice.align:main",
            "gear = dsr_practice.gear:main",
            "stack = dsr_practice.stack:main",
        ],
    },
)
