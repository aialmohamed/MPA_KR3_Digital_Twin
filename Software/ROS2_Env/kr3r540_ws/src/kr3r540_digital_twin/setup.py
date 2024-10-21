from setuptools import find_packages, setup

package_name = 'kr3r540_digital_twin'

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
    maintainer='robolab',
    maintainer_email='ahmaedibrahim311@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "digital_twin_data = kr3r540_digital_twin.digital_twin_data:main",
            "digital_twin_information = kr3r540_digital_twin.digital_twin_information:main",
        ],
    },
)
