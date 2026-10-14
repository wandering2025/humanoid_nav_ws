from glob import glob
from setuptools import find_packages, setup

setup(
    name='t1_bringup', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/t1_bringup']),
        ('share/t1_bringup', ['package.xml']),
        ('share/t1_bringup/launch', glob('launch/*.launch.py')),
        ('share/t1_bringup/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'initial_pose = t1_bringup.initial_pose:main', 'command_gate = t1_bringup.safety:main',
        'navigation_adapter = t1_bringup.navigation_adapter:main',
        'navigation_starter = t1_bringup.navigation_tools:starter_main',
        'navigate_to_pose = t1_bringup.navigation_tools:navigate_main']},
    maintainer='T1 navigation contributors', maintainer_email='codex@local.invalid',
    description='T1 launch skeleton and future evaluation tools.',
    license='Apache-2.0', tests_require=['pytest'],
)
