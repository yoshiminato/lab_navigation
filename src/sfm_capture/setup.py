from glob import glob
from setuptools import setup

setup(
    name='sfm_capture', version='0.1.0', packages=['sfm_capture'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/sfm_capture']),
        ('share/sfm_capture', ['package.xml']),
        ('share/sfm_capture/launch', glob('launch/*.launch.py')),
        ('share/sfm_capture/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Workspace user', maintainer_email='user@example.com',
    description='Timestamped monocular image collection for COLMAP',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'recorder = sfm_capture.recorder:main',
        'export_bag = sfm_capture.export_bag:main',
        'check_dataset = sfm_capture.check_dataset:main',
    ]},
)
