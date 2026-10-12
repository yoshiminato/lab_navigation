from setuptools import find_packages, setup

package_name = 'hloc_localizer'

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
    maintainer='user',
    maintainer_email='c081ffw@yamaguchi-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'hloc_tf_publisher = hloc_localizer.hloc_tf_publisher:main',
        'hloc_localizer_node = hloc_localizer.hloc_localizer_node:main',
        'image_file_publisher = hloc_localizer.image_file_publisher:main',
    ],
},
)
