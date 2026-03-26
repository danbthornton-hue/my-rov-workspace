from setuptools import find_packages, setup

package_name = 'rovultrasonic'

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
    maintainer='roverto',
    maintainer_email='dan.b.thornton@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publishd1 = rovultrasonic.publishd1:main',
            'publishd2 = rovultrasonic.publishd2:main',
        ],
    },
)
