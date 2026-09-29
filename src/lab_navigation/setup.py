from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab_navigation'

def package_files(directory):
    paths = []

    for path, directories, filenames in os.walk(directory):
        files = []

        for filename in filenames:
            files.append(os.path.join(path, filename))

        if files:
            install_path = os.path.join(
                'share',
                package_name,
                path
            )

            paths.append((install_path, files))

    return paths

# 通常のdata_files
data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]
    ),

    (
        'share/' + package_name,
        ['package.xml']
    ),

    (
        os.path.join('share', package_name, 'launch'),
        glob('launch/*.py')
    ),

    (
        os.path.join('share', package_name, 'urdf'),
        glob('urdf/*.xacro')
    ),

    (
        os.path.join('share', package_name, 'params'),
        glob('params/*.yaml')
    ),

    (
        os.path.join('share', package_name, 'world'),
        glob('world/*.sdf')
    ),
    
]

# models以下をサブディレクトリ構造を保ったまま追加
data_files += package_files('map')
data_files += package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_baselink_tf = odom_baselink_tf_publisher.publisher:main',
        ],
    },
)
