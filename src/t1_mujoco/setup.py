from glob import glob
from setuptools import find_packages, setup

setup(
    name='t1_mujoco',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/t1_mujoco']),
        ('share/t1_mujoco', ['package.xml']),
        ('share/t1_mujoco/config', glob('config/*.yaml') + glob('config/*.md')),
        ('lib/t1_mujoco', ['scripts/t1_diagnostic']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='T1 navigation contributors',
    maintainer_email='codex@local.invalid',
    description='T1 runtime diagnostics; physics implementation starts in W04.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        't1_diagnostic_python = t1_mujoco.diagnostic:main',
    ]},
)
