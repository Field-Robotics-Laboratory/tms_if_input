import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'tms_if_input'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'),glob('data/*.yaml')),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'json_samples'),glob('json_samples/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='common',
    maintainer_email='kasahara.yuichiro.res@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loader_from_path = tms_if_input.loader_from_path:main',
            'scenario_segmentor = tms_if_input.scenario_segmentor:main',
            'taskset_compiler = tms_if_input.taskset_compiler:main',
            'db_task_writer = tms_if_input.db_task_writer:main',
            'db_param_writer = tms_if_input.db_param_writer:main',
        ],
    },
)
