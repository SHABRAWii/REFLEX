from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'reflex_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'reflex_control', 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yousef ElShabrawii',
    maintainer_email='yousef.elshabrawii@gmail.com',
    description='Control package for the REFLEX wheeled-biped robot providing wheel velocity control, PID-based control modes, and extensible control nodes for balancing and locomotion experiments.',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'wheel_velocity_pid = reflex_control.wheel_velocity_pid:main',
          'reflex_balance_pid = reflex_control.balance_pid:main',
          'reflex_status_dashboard = reflex_control.status_dashboard:main',
          'reflex_pid_tuner = reflex_control.pid_tuner:main',
          'reflex_debug_plotter = reflex_control.reflex_debug_plotter:main',
          'reflex_status_dashboard_ui = reflex_control.reflex_status_dashboard_ui:main',
          'stance_controller = reflex_control.stance_controller:main',

        ],
    },
)
