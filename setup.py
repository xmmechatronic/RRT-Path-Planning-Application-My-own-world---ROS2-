from setuptools import setup
import os
from glob import glob

package_name = 'rl_maze_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyalarını ekleyin
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), 
         glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ceri',
    maintainer_email='ceri@todo.todo',
    description='Q-Learning Maze Navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maze_learner = rl_maze_nav.maze_learner:main',
            'rrt_planner = rl_maze_nav.rrt_planner:main',
            'plot_comparison = rl_maze_nav.plot_comparison:main',
        ],
    },
)